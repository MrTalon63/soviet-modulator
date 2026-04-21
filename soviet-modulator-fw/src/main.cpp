#include <Arduino.h>
#include "bpsk_modulator.h"
#include <Wire.h>
#include <si5351.h>
#include <hardware/watchdog.h>

#define MODULATING_PIN 20
#define LED_PIN 25
#define INPUT_BUFFER_SIZE 512
#define SI5351_SDA_PIN 16
#define SI5351_SCL_PIN 17

static constexpr uint64_t LO_FREQUENCY_HZ = 51000000ULL;
static constexpr uint64_t LO_FREQUENCY_01HZ = LO_FREQUENCY_HZ * SI5351_FREQ_MULT;

static BPSKModulator* modulator = nullptr;
static Si5351 si5351;
static bool filler_enabled = true;
static unsigned long frame_start = 0;
static const unsigned long FRAME_PERIOD_MS = 100;
static unsigned long last_blink = 0;
static bool led_state = false;

// Input state machine
static char input_buffer[INPUT_BUFFER_SIZE];
static uint16_t input_idx = 0;
static bool waiting_for_input = false;
static char input_mode = '\0';

/*
Command summary:
  r <rate> - Set symbol rate (1-100000 Hz)
  s - Start modulation
  p - Stop modulation
  i - Print current rate
  v - Toggle output inversion
  f - Toggle filler transmission
  c - Toggle CCSDS convolutional encoding
	n - Toggle CCSDS randomizer
  t <data> - Transmit message (ASCII text)
  x - Reinitialize Si5351 LO to 51 MHz
  m - Restart whole microcontroller
*/

void init_si5351_i2c() {
	Wire.setSDA(SI5351_SDA_PIN);
	Wire.setSCL(SI5351_SCL_PIN);
	Wire.begin();
	Serial.print("Si5351 I2C on SDA GP");
	Serial.print(SI5351_SDA_PIN);
	Serial.print(", SCL GP");
	Serial.println(SI5351_SCL_PIN);
}

void init_si5351_lo() {
	if (!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
		Serial.println("Si5351 init failed");
		return;
	}

	for (uint8_t clk = SI5351_CLK1; clk <= SI5351_CLK7; clk++) {
		si5351.set_clock_pwr((enum si5351_clock)clk, 0);
		si5351.output_enable((enum si5351_clock)clk, 0);
	}

	si5351.set_clock_pwr(SI5351_CLK0, 1);
	si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
	si5351.set_freq(LO_FREQUENCY_01HZ, SI5351_CLK0);
	si5351.output_enable(SI5351_CLK0, 1);
	Serial.print("Si5351 LO set to ");
	Serial.print(LO_FREQUENCY_HZ / 1000000ULL);
	Serial.println(" MHz on CLK0 at 2 mA drive");
}

[[noreturn]] void reboot_microcontroller() {
	Serial.println("Rebooting microcontroller");
	Serial.flush();
	delay(50);
	watchdog_reboot(0, 0, 0);
	while (true) {
	}
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	pinMode(23, OUTPUT);
	digitalWrite(23, HIGH); // Set pin 23 high so DC/DC isn't fucking shitfest
	
	Serial.begin(921600);
	delay(5000);
	modulator = new BPSKModulator(MODULATING_PIN, 1200);
	if (modulator == nullptr) {
		Serial.println("Modulator allocation failed");
		return;
	}
	init_si5351_i2c();
	init_si5351_lo();
	
	Serial.println("BPSK Modulator initialized");
#if USE_PIO_MODULATION
	Serial.println("  Mode: PIO-based (hardware)");
#else
	Serial.println("  Mode: Software Timer-based");
#endif
	Serial.print("Modulation GPIO: ");
	Serial.println(MODULATING_PIN);
	Serial.println("Commands:");
	Serial.println("  r <rate> - Set symbol rate (1-100000 Hz)");
	Serial.println("  s - Start modulation");
	Serial.println("  p - Stop modulation");
	Serial.println("  i - Print current rate");
	Serial.println("  v - Toggle output inversion");
	Serial.println("  f - Toggle filler transmission");
	Serial.println("  c - Toggle CCSDS convolutional encoding");
	Serial.println("  n - Toggle CCSDS randomizer");
	Serial.println("  t <data> - Transmit message (ASCII text)");
	Serial.println("  x - Reinitialize Si5351 LO to 51 MHz");
	Serial.println("  m - Restart whole microcontroller");
	
	frame_start = millis();
}

void process_input(char mode, const char* data) {
	Serial.println("");

  	if (mode == 'r') {
		uint32_t rate = atol(data);
		if (modulator != nullptr) modulator->set_symbolrate(rate);
		Serial.print("Symbol rate set to: ");
		Serial.println(rate);
  	} else if (mode == 't') {
		uint16_t len = strlen(data);
		if (len > MSG_BUFFER_SIZE) len = MSG_BUFFER_SIZE;
	
		if (len > 0) {
		  	uint8_t msg[MSG_BUFFER_SIZE];
	  		for (uint16_t i = 0; i < len; i++) {
				msg[i] = (uint8_t)data[i];
	  		}
	  		if (modulator != nullptr) modulator->queue_message(msg, len);
	  		Serial.print("Message queued: ");
	  		Serial.print(len);
	  		Serial.println(" bytes");
		}
  	}
}

void loop() {
	if (modulator == nullptr) {
		return;
	}

	// LED heartbeat: toggle every 500ms for 1s total blink cycle
	unsigned long now = millis();
	if (now - last_blink >= 500) {
		led_state = !led_state;
		digitalWrite(LED_PIN, led_state ? HIGH : LOW);
		last_blink = now;
	}

	// Inject pending messages at frame rate
	if (modulator->has_pending_message()) {
		if (now - frame_start >= FRAME_PERIOD_MS) {
			modulator->inject_message(filler_enabled);
			frame_start = now;
		}
	}

#if USE_PIO_MODULATION
	// Service PIO FIFO (PIO mode only)
	modulator->service();
#endif

	if (Serial.available()) {
		char c = Serial.read();
	
		if (waiting_for_input) {
			if (c == '\n' || c == '\r') {
				input_buffer[input_idx] = '\0';
				process_input(input_mode, input_buffer);
				waiting_for_input = false;
				input_idx = 0;
			} else if (input_idx < INPUT_BUFFER_SIZE - 1) {
				input_buffer[input_idx++] = c;
				Serial.write(c);  // Echo back
			}
		} else {
			switch (c) {
				case 'r': {
					Serial.println("Enter rate (1-100000):");
					waiting_for_input = true;
					input_mode = 'r';
					input_idx = 0;
					break;
				}
				case 's': {
					modulator->start();
					Serial.println("Modulation started");
#if USE_PIO_MODULATION
					Serial.print("SM: ");
					Serial.println(modulator->get_sm());
					Serial.print("FIFO state - empty: ");
					Serial.print(modulator->tx_fifo_empty());
					Serial.print(", full: ");
					Serial.println(modulator->tx_fifo_full());
#else
					Serial.print("Symbol rate: ");
					Serial.print(modulator->get_symbolrate());
					Serial.println(" Hz");
#endif
					break;
				}
				case 'p': {
					modulator->stop();
					Serial.println("Modulation stopped");
					break;
				}
				case 'i': {
					Serial.print("Current symbol rate: ");
					Serial.print(modulator->get_symbolrate());
					Serial.println(" Hz");
					break;
				}
				case 'v': {
					bool current = modulator->get_invert_output();
					modulator->set_invert_output(!current);
					Serial.print("Output inversion: ");
					Serial.println(!current ? "ON" : "OFF");
					break;
				}
				case 'f': {
					filler_enabled = !filler_enabled;
					if (filler_enabled) {
						modulator->init_frame();
						modulator->restart_transmission();
						Serial.println("Filler transmission enabled");
					} else {
						modulator->clear_frame();
						Serial.println("Filler transmission disabled");
					}
					break;
				}
				case 'c': {
					bool current = modulator->get_convolutional_encoding();
					modulator->set_convolutional_encoding(!current);
					Serial.print("CCSDS convolutional encoding: ");
					Serial.println(!current ? "ON" : "OFF");
					break;
				}
				case 'n': {
					bool current = modulator->get_randomizer_enabled();
					modulator->set_randomizer_enabled(!current);
					Serial.print("CCSDS randomizer: ");
					Serial.println(!current ? "ON" : "OFF");
					break;
				}
				case 't': {
					Serial.println("Enter message:");
					waiting_for_input = true;
					input_mode = 't';
					input_idx = 0;
					break;
				}
				case 'x': {
					Serial.println("Reinitializing Si5351 LO");
					init_si5351_lo();
					break;
				}
				case 'm': {
					reboot_microcontroller();
					break;
				}
				default:
					Serial.println("Unknown command");
					break;
			}
		}
	}
}