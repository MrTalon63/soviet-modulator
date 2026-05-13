#include <Arduino.h>
#include "bpsk_modulator.h"
#include <Wire.h>
#include <si5351.h>
#include <hardware/watchdog.h>
#include "pico/util/queue.h"

#define MODULATING_PIN 20
#define LED_PIN 25
#define INPUT_BUFFER_SIZE 1024
#define SI5351_SDA_PIN 16
#define SI5351_SCL_PIN 17

static constexpr uint64_t LO_FREQUENCY_HZ = 144500000ULL;
static constexpr uint64_t LO_FREQUENCY_01HZ = LO_FREQUENCY_HZ * SI5351_FREQ_MULT;

static BPSKModulator* modulator = nullptr;
static Si5351 si5351;
static unsigned long frame_start = 0;
static unsigned long last_blink = 0;
static bool led_state = false;

// Input state machine
static char input_buffer[INPUT_BUFFER_SIZE];
static uint16_t input_idx = 0;
static bool waiting_for_input = false;
static char input_mode = '\0';

static bool binary_upload_mode = false;
static unsigned long last_binary_rx_ms = 0;

static unsigned long current_frame_period_ms() {
	if (modulator == nullptr) return 100;
	uint32_t rate = modulator->get_symbolrate();
	if (rate == 0) rate = 1;

	uint32_t output_symbols_per_frame;
	if (modulator->get_convolutional_encoding()) {
		double code_rate = 1.0;
		switch(modulator->get_conv_rate()) {
			case 0: code_rate = 1.0/2.0; break;
			case 1: code_rate = 2.0/3.0; break;
			case 2: code_rate = 3.0/4.0; break;
			case 3: code_rate = 5.0/6.0; break;
			case 4: code_rate = 7.0/8.0; break;
		}
		output_symbols_per_frame = (uint32_t)((FRAME_SIZE * 8.0) / code_rate);
	} else {
		output_symbols_per_frame = FRAME_SIZE * 8U;
	}

	uint32_t period_ms = (output_symbols_per_frame * 1000U + rate - 1U) / rate;
	if (period_ms < 1U) period_ms = 1U;
	return (unsigned long)(period_ms + 2U);
}

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
	d - Toggle CCSDS dual basis conversion
	y - Toggle Reed-Solomon (255,223) I=4 encoding
	e - Toggle Frame Error Control Field (FECF)
	q - Print upload/FIFO status
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

void print_help() {
	Serial.println("Commands:");
	Serial.println("  r <rate> - Set symbol rate (1-1000000 Hz)");
	Serial.println("  s - Start modulation");
	Serial.println("  p - Stop modulation");
	Serial.println("  i - Print current rate");
	Serial.println("  c - Toggle CCSDS convolutional encoding");
	Serial.println("  k <rate> - Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)");
	Serial.println("  n - Toggle CCSDS randomizer");
	Serial.println("  d - Toggle CCSDS dual basis conversion");
	Serial.println("  q - Print upload/FIFO status");
	Serial.println("  t <data> - Transmit message (ASCII text)");
	Serial.println("  y - Toggle Reed-Solomon (255,223) I=4 encoding");
	Serial.println("  e - Toggle Frame Error Control Field (FECF)");
	Serial.println("  u - Binary upload mode (raw Space Packets, timeout 5s exits)");
	Serial.println("  x - Reinitialize Si5351 LO to 51 MHz");
	Serial.println("  o - Queue OID (Operational Idle Data) frame (VCID=0x3F)");
	Serial.println("  I <vcid> - Queue idle data MPDU frame with specified VCID (0-63)");
	Serial.println("  O - Output OID pattern (first 10 bytes) for verification");
	Serial.println("  m - Restart whole microcontroller");
	Serial.println("  h - Print this help menu");
}

void setup() {
	delay(100); // Allow hardware peripherals (Si5351) a fraction of a second to cleanly power up
	set_sys_clock_khz(250000, true); // Overclock RP2350 to 250 MHz for high-speed stability

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	pinMode(23, OUTPUT);
	digitalWrite(23, HIGH); // Set pin 23 high so DC/DC isn't fucking shitfest
	
	Serial.begin(921600);
	rs_init(); // Initialize Reed-Solomon tables BEFORE the modulator generates its OID frame!
	
	modulator = new BPSKModulator(MODULATING_PIN, 500000);
	if (modulator == nullptr) {
		Serial.println("Modulator allocation failed");
		return;
	}
	init_si5351_i2c();
	init_si5351_lo();
	
	Serial.println("BPSK Modulator initialized");
	Serial.println("  Mode: PIO-based (hardware)");
	Serial.print("Modulation GPIO: ");
	Serial.println(MODULATING_PIN);
	print_help();
	
	frame_start = millis();
}

void process_input(char mode, const char* data) {
	Serial.println("");

  	if (mode == 'r') {
		uint32_t rate = atol(data);
		if (modulator != nullptr) modulator->set_symbolrate(rate);
		Serial.print("Symbol rate set to: ");
		Serial.println(rate);
  	} else if (mode == 'k') {
		uint8_t rate = atoi(data);
		if (modulator != nullptr) modulator->set_conv_rate(rate);
		Serial.print("Convolutional rate set to: ");
		switch (modulator->get_conv_rate()) {
			case 0: Serial.println("1/2"); break;
			case 1: Serial.println("2/3"); break;
			case 2: Serial.println("3/4"); break;
			case 3: Serial.println("5/6"); break;
			case 4: Serial.println("7/8"); break;
		}
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
  	} else if (mode == 'I') {
		uint8_t vcid = atoi(data);
		if (vcid > 63) {
			Serial.println("ERROR: VCID must be 0-63");
		} else {
			if (modulator != nullptr) modulator->queue_idle_data_frame(vcid);
			Serial.print("Idle data frame queued with VCID=");
			Serial.println(vcid);
		}
	}
}

// setup1() and loop1() natively execute on Core 1 in Arduino-Pico
void setup1() {
	// Wait for modulator to be dynamically initialized by Core 0
	while (modulator == nullptr) {
		delay(10);
	}
}

void loop1() {
	if (modulator == nullptr) return;
	
	// Yield completely to Core 0 if settings are actively being changed
	__dmb(); // Ensure we see the latest config_lock state from Core 0
	if (modulator->config_lock) return;

	// 1. Pack the pending frame queue with as much MPDU user payload as possible
	while (modulator->can_queue_binary_frame()) {
		__dmb();
		if (modulator->config_lock) break;
		if (!modulator->has_data_to_send()) break;
		modulator->generate_and_queue_mpdu();
	}

	// 2. Drain baseband queue into DMA chunk queue
	while (true) {
		__dmb();
		if (modulator->config_lock) break;
		if (!modulator->process_baseband_to_dma()) break; // Break when DMA queue hits 14 chunks
		delayMicroseconds(50); // Prevent Core 1 from starving the system bus
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

	// Safety Timeout: Escape Binary Mode after 5 seconds of silence.
	if (binary_upload_mode && !Serial.available()) {
		if (now - last_binary_rx_ms > 5000) {
			binary_upload_mode = false;
			Serial.println("\n[MCU] Binary mode timeout/reset! Returning to command mode.");
		}
	}

	if (Serial.available()) {
		if (binary_upload_mode) {
			while (Serial.available()) {
				if (modulator->get_sp_fifo_free() > 0) {
					modulator->push_sp_byte((uint8_t)Serial.read());
					last_binary_rx_ms = now; // Only reset watchdog when we actually pull data
				} else {
					break; // Wait for space, yields to background tasks naturally
				}
			}
			return;
		}

		// Prevent reading -1 when binary mode exits and buffer is empty
		if (!Serial.available()) return;

		char c = Serial.read();

		if (c == 'u') {
			waiting_for_input = false;
			input_idx = 0;
			binary_upload_mode = true;
			
			modulator->reset_baseband_queues();
			
			last_binary_rx_ms = millis(); // Initialize watchdog timer
			Serial.write('B');
			return;
		}
	
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
			// Ignore line endings and whitespace chatter from terminal when NOT in input mode
			if (c == '\r' || c == '\n' || c == ' ') {
				return;
			}

			switch (c) {
				case 'r': {
					Serial.println("Enter rate (1-10000000):");
					waiting_for_input = true;
					input_mode = 'r';
					input_idx = 0;
					break;
				}
				case 's': {
					modulator->lock_config();
					modulator->start();
					modulator->unlock_config();
					Serial.println("Modulation started");
					Serial.print("SM: ");
					Serial.println(modulator->get_sm());
					Serial.print("FIFO state - empty: ");
					Serial.print(modulator->tx_fifo_empty());
					Serial.print(", full: ");
					Serial.println(modulator->tx_fifo_full());
					break;
				}
				case 'p': {
					modulator->lock_config();
					modulator->stop();
					modulator->unlock_config();
					Serial.println("Modulation stopped");
					break;
				}
				case 'i': {
					Serial.print("Current symbol rate: ");
					Serial.print(modulator->get_symbolrate());
					Serial.println(" Hz");
					break;
				}
				case 'c': {
					bool current = modulator->get_convolutional_encoding();
					modulator->set_convolutional_encoding(!current);
					Serial.print("CCSDS convolutional encoding: ");
					Serial.println(modulator->get_convolutional_encoding() ? "ON" : "OFF");
					break;
				}
				case 'k': {
					Serial.println("Enter puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8):");
					waiting_for_input = true;
					input_mode = 'k';
					input_idx = 0;
					break;
				}
				case 'n': {
					bool current = modulator->get_randomizer_enabled();
					modulator->set_randomizer_enabled(!current);
					Serial.print("CCSDS randomizer: ");
					Serial.println(modulator->get_randomizer_enabled() ? "ON" : "OFF");
					break;
				}
				case 'd': {
					bool current = modulator->get_dual_basis_enabled();
					modulator->set_dual_basis_enabled(!current);
					Serial.print("CCSDS dual basis: ");
					Serial.println(modulator->get_dual_basis_enabled() ? "ON" : "OFF");
					break;
				}
				case 'y': {
					bool current = modulator->get_reed_solomon_enabled();
					modulator->set_reed_solomon_enabled(!current);
					Serial.print("Reed-Solomon (255,223) I=4: ");
					Serial.println(modulator->get_reed_solomon_enabled() ? "ON" : "OFF");
					break;
				}
				case 'e': {
					bool current = modulator->get_fecf_enabled();
					modulator->set_fecf_enabled(!current);
					Serial.print("FECF (CRC-16): ");
					Serial.println(modulator->get_fecf_enabled() ? "ON" : "OFF");
					break;
				}
				case 't': {
					Serial.println("Enter message:");
					waiting_for_input = true;
					input_mode = 't';
					input_idx = 0;
					break;
				}
				case 'q': {
					Serial.println("Modulator status:");
					Serial.print("  RS (255,223): ");
					Serial.println(modulator->get_reed_solomon_enabled() ? "ON" : "OFF");
					Serial.print("  Dual Basis: ");
					Serial.println(modulator->get_dual_basis_enabled() ? "ON" : "OFF");
					Serial.print("  FECF (CRC-16): ");
					Serial.println(modulator->get_fecf_enabled() ? "ON" : "OFF");
					Serial.print("  Randomizer: ");
					Serial.println(modulator->get_randomizer_enabled() ? "ON" : "OFF");
					Serial.print("  Convolutional: ");
					Serial.print(modulator->get_convolutional_encoding() ? "ON" : "OFF");
					if (modulator->get_convolutional_encoding()) {
						Serial.print(" (Rate ");
						switch (modulator->get_conv_rate()) {
							case 0: Serial.print("1/2"); break;
							case 1: Serial.print("2/3"); break;
							case 2: Serial.print("3/4"); break;
							case 3: Serial.print("5/6"); break;
							case 4: Serial.print("7/8"); break;
						}
						Serial.print(")");
					}
					Serial.println();
					Serial.print("  Expected Payload: ");
					uint16_t ep = (modulator->get_reed_solomon_enabled() ? 892 : 1020) - (modulator->get_fecf_enabled() ? 2 : 0) - 8;
					Serial.print(ep);
					Serial.println(" bytes");
					Serial.println("Space Packet FIFO status:");
					Serial.print("  FIFO depth: ");
					Serial.print(modulator->get_sp_fifo_count());
					Serial.print("/");
					Serial.println("131072 bytes");
					Serial.print("  TX Frames pending: ");
					Serial.println(modulator->get_pending_frames_count());
					Serial.print("  TX Chunks pending: ");
					Serial.println(modulator->get_dma_chunks_count());
					Serial.print("  Active User Chunks: ");
					Serial.println(modulator->get_active_user_chunks());
					Serial.print("  upload active: ");
					Serial.println(binary_upload_mode ? "YES" : "NO");
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
				case 'O': {
					// Output first 10 bytes of OID frame for verification
					Serial.println("OID pattern (first 10 bytes):");
					uint8_t* oid_frame = modulator->get_oid_frame();
					for (int i = 0; i < 10; i++) {
						Serial.print(oid_frame[i], HEX);
						if (i < 9) Serial.print(" ");
					}
					Serial.println();
					Serial.println("Expected: FF FF FF FF 6D B6 D8 61 45 1F");
					break;
				}
				case 'o': {
					// Queue an OID frame for transmission
					if (modulator->queue_oid_frame()) {
						Serial.println("OID frame queued");
					} else {
						Serial.println("ERROR: Failed to queue OID frame");
					}
					break;
				}
				case 'I': {
					Serial.println("Enter VCID (0-63, default 0):");
					waiting_for_input = true;
					input_mode = 'I';
					input_idx = 0;
					break;
				}
				case 'h': {
					print_help();
					break;
				}
				default:
					Serial.print("Unknown command: ");
					Serial.println(c);
					break;
			}
		}
	}
}