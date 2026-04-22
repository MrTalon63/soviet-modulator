#include <Arduino.h>
#include "bpsk_modulator.h"
#include <Wire.h>
#include <si5351.h>
#include <hardware/watchdog.h>

#define MODULATING_PIN 20
#define LED_PIN 25
#define INPUT_BUFFER_SIZE 1024
#define SI5351_SDA_PIN 16
#define SI5351_SCL_PIN 17

static constexpr uint64_t LO_FREQUENCY_HZ = 51000000ULL;
static constexpr uint64_t LO_FREQUENCY_01HZ = LO_FREQUENCY_HZ * SI5351_FREQ_MULT;

static BPSKModulator* modulator = nullptr;
static Si5351 si5351;
static bool filler_enabled = true;
static unsigned long frame_start = 0;
static unsigned long last_blink = 0;
static bool led_state = false;

// Input state machine
static char input_buffer[INPUT_BUFFER_SIZE];
static uint16_t input_idx = 0;
static bool waiting_for_input = false;
static char input_mode = '\0';

static bool binary_upload_mode = false;
static bool binary_upload_finishing = false;
static unsigned long binary_finish_started_ms = 0;
static bool binary_upload_prepacked_asm = false;
static bool binary_upload_waiting_for_mode = false;
static bool binary_upload_chunk_pending_enqueue = false;
static uint8_t binary_len_buf[2];
static uint8_t binary_len_idx = 0;
static uint16_t binary_expected_len = 0;
static uint16_t binary_received_len = 0;
static uint8_t binary_payload_buf[FRAME_SIZE];
static uint16_t binary_pending_chunk_len = 0;
static bool binary_pending_chunk_prepacked_asm = false;
static uint8_t binary_pending_chunk_buf[FRAME_SIZE];

struct BinaryQueueItem {
    uint8_t data[FRAME_SIZE];
    uint16_t len;
    bool prepacked_asm;
};

static constexpr uint8_t BINARY_QUEUE_CAPACITY = 250; // 80 * 256 = 20480 bytes (~20 kB)
static BinaryQueueItem binary_queue_buf[BINARY_QUEUE_CAPACITY];
static uint8_t binary_queue_head = 0;
static uint8_t binary_queue_tail = 0;
static uint8_t binary_queue_count = 0;
static uint32_t binary_frames_enqueued_total = 0;
static uint32_t binary_frames_dequeued_total = 0;
static bool binary_tx_drain_complete = false;

static unsigned long current_frame_period_ms() {
	if (modulator == nullptr) return 100;
	uint32_t rate = modulator->get_symbolrate();
	if (rate == 0) rate = 1;
	uint32_t bits_per_frame = FRAME_SIZE * 8U;
	if (modulator->get_convolutional_encoding()) bits_per_frame *= 2U;
	uint32_t period_ms = (bits_per_frame * 1000U + rate - 1U) / rate;
	if (period_ms < 1U) period_ms = 1U;
	return (unsigned long)(period_ms + 2U);
}

static void reset_binary_queue() {
	binary_queue_head = 0;
	binary_queue_tail = 0;
	binary_queue_count = 0;
}

static bool push_binary_queue_frame(const uint8_t *data, uint16_t len, bool prepacked) {
	if (binary_queue_count >= BINARY_QUEUE_CAPACITY) return false;
	memcpy(binary_queue_buf[binary_queue_tail].data, data, len);
	binary_queue_buf[binary_queue_tail].len = len;
	binary_queue_buf[binary_queue_tail].prepacked_asm = prepacked;
	binary_queue_tail = (uint8_t)((binary_queue_tail + 1U) % BINARY_QUEUE_CAPACITY);
	binary_queue_count++;
	binary_frames_enqueued_total++;
	binary_tx_drain_complete = false;
	return true;
}

static bool pop_binary_queue_frame(BinaryQueueItem *out_item) {
	if (binary_queue_count == 0) return false;
	memcpy(out_item->data, binary_queue_buf[binary_queue_head].data, binary_queue_buf[binary_queue_head].len);
	out_item->len = binary_queue_buf[binary_queue_head].len;
	out_item->prepacked_asm = binary_queue_buf[binary_queue_head].prepacked_asm;
	binary_queue_head = (uint8_t)((binary_queue_head + 1U) % BINARY_QUEUE_CAPACITY);
	binary_queue_count--;
	binary_frames_dequeued_total++;
	return true;
}

static void reset_binary_upload_state() {
	binary_upload_waiting_for_mode = false;
	binary_upload_prepacked_asm = false;
	binary_upload_chunk_pending_enqueue = false;
	binary_len_idx = 0;
	binary_expected_len = 0;
	binary_received_len = 0;
	binary_pending_chunk_len = 0;
	binary_pending_chunk_prepacked_asm = false;
}

static void feed_modulator() {
	if (binary_queue_count > 0 && modulator->can_queue_binary_frame()) {
		BinaryQueueItem item;
		if (pop_binary_queue_frame(&item)) {
			modulator->queue_binary_frame(item.data, item.len, item.prepacked_asm);
		}
	}
}

static bool try_enqueue_pending_chunk() {
	if (!binary_upload_chunk_pending_enqueue) return true;

	if (!push_binary_queue_frame(binary_pending_chunk_buf, binary_pending_chunk_len, binary_pending_chunk_prepacked_asm)) {
		return false;
	}

	binary_upload_chunk_pending_enqueue = false;
	binary_pending_chunk_len = 0;
	Serial.write('K');
	return true;
}

static void process_binary_upload_byte(uint8_t b) {
	if (binary_upload_chunk_pending_enqueue) {
		return;
	}

	if (binary_upload_waiting_for_mode) {
		binary_upload_prepacked_asm = (b != 0);
		binary_upload_waiting_for_mode = false;
		return;
	}

	if (binary_expected_len == 0) {
		binary_len_buf[binary_len_idx++] = b;
		if (binary_len_idx < 2) return;

		binary_expected_len = (uint16_t)binary_len_buf[0] | ((uint16_t)binary_len_buf[1] << 8);
		binary_len_idx = 0;

		if (binary_expected_len == 0) {
			binary_upload_mode = false;
			binary_upload_finishing = true;
			binary_finish_started_ms = 0;
			binary_tx_drain_complete = false;
			reset_binary_upload_state();
			return;
		}

		uint16_t binary_chunk_max = binary_upload_prepacked_asm ? FRAME_SIZE : (FRAME_SIZE - CADU_ASM_SIZE);
		if (binary_expected_len > binary_chunk_max) {
			binary_upload_mode = false;
			binary_upload_finishing = false;
			binary_finish_started_ms = 0;
			reset_binary_upload_state();
			Serial.write('E');
			return;
		}
		return;
	}

	binary_payload_buf[binary_received_len++] = b;
	if (binary_received_len >= binary_expected_len) {
		memcpy(binary_pending_chunk_buf, binary_payload_buf, binary_expected_len);
		binary_pending_chunk_len = binary_expected_len;
		binary_pending_chunk_prepacked_asm = binary_upload_prepacked_asm;
		binary_upload_chunk_pending_enqueue = true;
		(void)try_enqueue_pending_chunk();
		feed_modulator(); // Feed modulator immediately after enqueueing
		binary_expected_len = 0;
		binary_received_len = 0;
	}
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
	Serial.println("  v - Toggle output inversion");
	Serial.println("  f - Toggle filler transmission");
	Serial.println("  c - Toggle CCSDS convolutional encoding");
	Serial.println("  n - Toggle CCSDS randomizer");
	Serial.println("  q - Print upload/FIFO status");
	Serial.println("  t <data> - Transmit message (ASCII text)");
	Serial.println("  u - Binary upload mode (mode byte, len16 + data, len=0 ends)");
	Serial.println("  x - Reinitialize Si5351 LO to 51 MHz");
	Serial.println("  m - Restart whole microcontroller");
	Serial.println("  h - Print this help menu");
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	pinMode(23, OUTPUT);
	digitalWrite(23, HIGH); // Set pin 23 high so DC/DC isn't fucking shitfest
	
	Serial.begin(921600);
	delay(5000);
	modulator = new BPSKModulator(MODULATING_PIN, 5000);
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

	// Feed queued binary chunks as soon as modulator has a free pending slot.
	feed_modulator();

	if (binary_upload_mode && binary_upload_chunk_pending_enqueue) {
		(void)try_enqueue_pending_chunk();
	}

	if (binary_upload_finishing) {
		if (binary_queue_count == 0 && modulator->can_queue_binary_frame()) {
			if (binary_finish_started_ms == 0) {
				binary_finish_started_ms = now;
			} else if (now - binary_finish_started_ms >= current_frame_period_ms()) {
				binary_upload_finishing = false;
				binary_finish_started_ms = 0;
				binary_tx_drain_complete = true;
				Serial.write('D');
			}
		} else {
			binary_finish_started_ms = 0;
		}
	}

	// Keep text message injection on frame cadence.
	if (modulator->has_pending_message()) {
		if (now - frame_start >= current_frame_period_ms()) {
			modulator->inject_message(filler_enabled);
			frame_start = now;
		}
	}

	if (Serial.available()) {
		if (binary_upload_mode) {
			while (binary_upload_mode && Serial.available()) {
				if (binary_upload_chunk_pending_enqueue) {
					break; // Queue full, stop pulling bytes from Serial buffer to prevent dropping
				}
				process_binary_upload_byte((uint8_t)Serial.read());
			}
			if (binary_upload_mode) {
				return;
			}
		}

		// Prevent reading -1 when binary mode exits and buffer is empty
		if (!Serial.available()) return;

		char c = Serial.read();

		if (c == 'u') {
			waiting_for_input = false;
			input_idx = 0;
			binary_upload_mode = true;
			binary_upload_finishing = false;
			binary_finish_started_ms = 0;
			binary_tx_drain_complete = false;
			reset_binary_queue();
			reset_binary_upload_state();
			binary_upload_waiting_for_mode = true;
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
				case 'q': {
					Serial.println("Upload/FIFO status:");
					Serial.print("  queue depth: ");
					Serial.print(binary_queue_count);
					Serial.print("/");
					Serial.println(BINARY_QUEUE_CAPACITY);
					Serial.print("  enqueued total: ");
					Serial.println(binary_frames_enqueued_total);
					Serial.print("  dequeued total: ");
					Serial.println(binary_frames_dequeued_total);
					Serial.print("  upload active: ");
					Serial.println(binary_upload_mode ? "YES" : "NO");
					Serial.print("  upload finishing: ");
					Serial.println(binary_upload_finishing ? "YES" : "NO");
					Serial.print("  tx drain complete: ");
					Serial.println(binary_tx_drain_complete ? "YES" : "NO");
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