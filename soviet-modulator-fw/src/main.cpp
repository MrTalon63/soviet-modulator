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

// Expose ARM GCC Linker symbols to calculate exact stack boundaries
extern "C" char __StackLimit;
extern "C" char __StackTop;
extern "C" char __StackOneBottom;
extern "C" char __StackOneTop;

static volatile uint32_t core1_min_free_stack = 0xFFFFFFFF;
static constexpr uint64_t LO_FREQUENCY_HZ = 144500000ULL;
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
alignas(4) static uint8_t binary_payload_buf[MAX_FRAME_SIZE];
static uint16_t binary_pending_chunk_len = 0;
static bool binary_pending_chunk_prepacked_asm = false;
alignas(4) static uint8_t binary_pending_chunk_buf[MAX_FRAME_SIZE];

struct BinaryQueueItem {
    alignas(4) uint8_t data[MAX_FRAME_SIZE];
    uint16_t len;
    bool prepacked_asm;
};

// Set to ~256 KB queue to use more of our free RAM for maximum USB jitter resilience
static constexpr uint16_t BINARY_QUEUE_CAPACITY = (262144 / sizeof(BinaryQueueItem)) > 0 ? (262144 / sizeof(BinaryQueueItem)) : 2; 
static queue_t binary_queue;
static uint32_t binary_frames_enqueued_total = 0;
static uint32_t binary_frames_dequeued_total = 0;
static bool binary_tx_drain_complete = false;
static volatile bool request_fec_flush = false;
static unsigned long last_binary_rx_ms = 0;
static bool binary_error_sink_mode = false;

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
		output_symbols_per_frame = (uint32_t)((modulator->get_frame_size() * 8.0) / code_rate);
	} else {
		output_symbols_per_frame = modulator->get_frame_size() * 8U;
	}

	uint32_t period_ms = (output_symbols_per_frame * 1000U + rate - 1U) / rate;
	if (period_ms < 1U) period_ms = 1U;
	return (unsigned long)(period_ms + 2U);
}

static uint16_t get_binary_queue_count() {
	return queue_get_level(&binary_queue);
}

static void reset_binary_queue() {
	static BinaryQueueItem dummy;
	while (queue_try_remove(&binary_queue, &dummy));
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

static bool try_enqueue_pending_chunk() {
	if (!binary_upload_chunk_pending_enqueue) return true;

	static BinaryQueueItem item;
	memcpy(item.data, binary_pending_chunk_buf, binary_pending_chunk_len);
	item.len = binary_pending_chunk_len;
	item.prepacked_asm = binary_pending_chunk_prepacked_asm;
	
	if (!queue_try_add(&binary_queue, &item)) {
		return false;
	}

	binary_frames_enqueued_total++;
	binary_tx_drain_complete = false;
	binary_upload_chunk_pending_enqueue = false;
	binary_pending_chunk_len = 0;
	binary_expected_len = 0;
	binary_received_len = 0;
	Serial.write('K'); // Acknowledge successful enqueue to perfectly pace the Python script
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
			request_fec_flush = true; // Signal Core 1 to flush when queue empties safely
			reset_binary_upload_state();
			return;
		}

		uint16_t binary_chunk_max = binary_upload_prepacked_asm ? modulator->get_frame_size() : modulator->get_payload_size();
		if (binary_expected_len > binary_chunk_max) {
			binary_upload_mode = false;
			binary_upload_finishing = false;
			binary_finish_started_ms = 0;
			binary_error_sink_mode = true; // Safely absorb and destroy remaining garbage data from USB
			reset_binary_upload_state();
			Serial.write('E');
			return;
		}
		return;
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
	d - Toggle CCSDS dual basis conversion
	y - Toggle Reed-Solomon (255,223) encoding
	l <depth> - Set RS interleaving depth (1, 2, 4, 8, 16)
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
	Serial.println("  f - Toggle filler transmission");
	Serial.println("  c - Toggle CCSDS convolutional encoding");
	Serial.println("  k <rate> - Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)");
	Serial.println("  n - Toggle CCSDS randomizer");
	Serial.println("  d - Toggle CCSDS dual basis conversion");
	Serial.println("  q - Print upload/FIFO status");
	Serial.println("  t <data> - Transmit message (ASCII text)");
	Serial.println("  y - Toggle Reed-Solomon (255,223) encoding");
	Serial.println("  l <depth> - Set RS interleaving depth (1, 2, 4, 8, 16)");
	Serial.println("  u - Binary upload mode (mode byte, len16 + data, len=0 ends)");
	Serial.println("  x - Reinitialize Si5351 LO to 51 MHz");
	Serial.println("  m - Restart whole microcontroller");
	Serial.println("  h - Print this help menu");
}

void setup() {
	delay(100); // Allow hardware peripherals (Si5351) a fraction of a second to cleanly power up
	set_sys_clock_khz(250000, true); // Overclock RP2350 to 250 MHz for high-speed stability

	queue_init(&binary_queue, sizeof(BinaryQueueItem), BINARY_QUEUE_CAPACITY);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	pinMode(23, OUTPUT);
	digitalWrite(23, HIGH); // Set pin 23 high so DC/DC isn't fucking shitfest
	
	Serial.begin(921600);
	rs_init(); // Initialize Reed-Solomon tables BEFORE the modulator generates its filler frame!
	
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
  	} else if (mode == 'l') {
		uint8_t depth = atoi(data);
		if (modulator != nullptr) modulator->set_rs_interleave(depth);
		Serial.print("RS interleaving depth set to: ");
		Serial.println(modulator->get_rs_interleave());
  	} else if (mode == 't') {
		uint16_t len = strlen(data);
		if (len > MAX_MSG_BUFFER_SIZE) len = MAX_MSG_BUFFER_SIZE;
	
		if (len > 0) {
		  	uint8_t msg[MAX_MSG_BUFFER_SIZE];
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

// setup1() and loop1() natively execute on Core 1 in Arduino-Pico
void setup1() {
	// Wait for modulator to be dynamically initialized by Core 0
	while (modulator == nullptr) {
		delay(10);
	}
}

void loop1() {
	if (modulator == nullptr) return;

	// Track the "High Water Mark" of Core 1's stack (the lowest the free space ever gets)
	uint32_t core1_sp;
	asm volatile("mov %0, sp" : "=r"(core1_sp));
	uint32_t current_free = core1_sp - (uint32_t)&__StackOneBottom;
	if (current_free < core1_min_free_stack) core1_min_free_stack = current_free;
	
	// Yield completely to Core 0 if settings are actively being changed
	if (modulator->config_lock) return;

	// Drain as many incoming USB chunks as physically possible to maximize payload speed
	static BinaryQueueItem item;
	while (get_binary_queue_count() > 0 && modulator->can_queue_binary_frame()) {
		if (modulator->config_lock) break;
		if (queue_try_remove(&binary_queue, &item)) {
			modulator->queue_binary_frame(item.data, item.len, item.prepacked_asm);
			binary_frames_dequeued_total++;
		}
	}

	if (request_fec_flush && get_binary_queue_count() == 0) {
		if (!modulator->config_lock && modulator->can_queue_binary_frame()) { 
			modulator->flush_fec_buffer();
			request_fec_flush = false;
		}
	}

	// Drain baseband queue into DMA chunk queue
	while (!modulator->config_lock) {
		if (!modulator->process_baseband_to_dma()) break; // Break when DMA queue hits 14 chunks
	}

	if (modulator->has_pending_message()) {
		unsigned long now = millis();
		if (now - frame_start >= current_frame_period_ms()) {
			modulator->inject_message(filler_enabled);
			frame_start = now;
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

	if (binary_upload_mode && binary_upload_chunk_pending_enqueue) {
		(void)try_enqueue_pending_chunk();
	}

	if (binary_upload_finishing) {
		if (get_binary_queue_count() == 0 && modulator->get_pending_frames_count() == 0 && !request_fec_flush) {
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

	// Safety Timeout: If Python crashes or drops, automatically escape Binary Mode after 1 second of silence
	if ((binary_upload_mode || binary_error_sink_mode) && !Serial.available()) {
		if (now - last_binary_rx_ms > 1000) {
			binary_upload_mode = false;
			binary_upload_finishing = false;
			binary_upload_chunk_pending_enqueue = false;
			binary_error_sink_mode = false;
			reset_binary_upload_state();
			request_fec_flush = true;
			Serial.println("\n[MCU] Binary mode timeout/reset! Returning to command mode.");
		}
	}

	if (Serial.available()) {
		if (binary_error_sink_mode) {
			last_binary_rx_ms = now; // Reset the timeout watchdog
			while (Serial.available()) Serial.read(); // Dump garbage so it isn't parsed as reboot commands
			return;
		}

		if (binary_upload_mode) {
			last_binary_rx_ms = now; // Reset the timeout watchdog
			while (binary_upload_mode && Serial.available()) {
				if (binary_upload_chunk_pending_enqueue) {
					break; // Queue full, stop pulling bytes from Serial buffer to prevent dropping
				}
				
				if (binary_expected_len > 0) {
					size_t to_read = binary_expected_len - binary_received_len;
					size_t avail = Serial.available();
					if (avail < to_read) to_read = avail;
					
					// Use a tight loop to bypass the massive millis() overhead inside Stream::readBytes()
					for (size_t i = 0; i < to_read; i++) {
						binary_payload_buf[binary_received_len++] = (uint8_t)Serial.read();
					}
					
					if (binary_received_len >= binary_expected_len) {
						memcpy(binary_pending_chunk_buf, binary_payload_buf, binary_expected_len);
						binary_pending_chunk_len = binary_expected_len;
						binary_pending_chunk_prepacked_asm = binary_upload_prepacked_asm;
						binary_upload_chunk_pending_enqueue = true;
						
						if (!try_enqueue_pending_chunk()) {
							break; // Queue actually full, yield execution to system
						}
						
						// Force loop() to return after every chunk to service TinyUSB background tasks!
						// This prevents tud_task() starvation and USB driver deadlocks during high-speed blasts.
						break;
					}
				} else {
					process_binary_upload_byte((uint8_t)Serial.read());
				}
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
			request_fec_flush = false; // Cancel any pending flush from aborted runs
			
			modulator->lock_config();
			reset_binary_queue(); // Safely reset queue pointers while Core 1 is paused
			reset_binary_upload_state();
			modulator->unlock_config();
			
			modulator->clear_fec_buffer(); // Only clear the FEC buffer to seamlessly transition from filler to data
			
			// Buffers naturally clear on mode toggle or flush
			binary_upload_waiting_for_mode = true;
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
				case 'f': {
					filler_enabled = !filler_enabled;
					if (filler_enabled) {
						modulator->lock_config();
						modulator->init_frame();
						modulator->unlock_config();
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
				case 'l': {
					Serial.println("Enter RS interleaving depth (1, 2, 4, 8, 16):");
					waiting_for_input = true;
					input_mode = 'l';
					input_idx = 0;
					break;
				}
				case 'y': {
					bool current = modulator->get_reed_solomon_enabled();
					modulator->set_reed_solomon_enabled(!current);
					Serial.print("Reed-Solomon (255,223): ");
					Serial.println(modulator->get_reed_solomon_enabled() ? "ON" : "OFF");
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
					if (modulator->get_reed_solomon_enabled()) {
						Serial.print("ON (I=");
						Serial.print(modulator->get_rs_interleave());
						Serial.println(")");
					} else {
						Serial.println("OFF");
					}
					Serial.print("  Dual Basis: ");
					Serial.println(modulator->get_dual_basis_enabled() ? "ON" : "OFF");
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
					Serial.print(modulator->get_payload_size());
					Serial.print(" bytes (");
					Serial.print(modulator->get_frame_size());
					Serial.println(" if prepacked ASM)");
					Serial.println("Upload/FIFO status:");
					Serial.print("  queue depth: ");
					Serial.print(get_binary_queue_count());
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
					
					// Measure exact stack boundaries
					uint32_t core0_sp;
					asm volatile("mov %0, sp" : "=r"(core0_sp));
					uint32_t core0_total = (uint32_t)&__StackTop - (uint32_t)&__StackLimit;
					uint32_t core0_free = core0_sp - (uint32_t)&__StackLimit;
					uint32_t core1_total = (uint32_t)&__StackOneTop - (uint32_t)&__StackOneBottom;
					
					Serial.println("Memory status:");
					Serial.print("  Core 0 Stack Free: ");
					Serial.print(core0_free); Serial.print(" / "); Serial.println(core0_total);
					Serial.print("  Core 1 Stack Free (Min seen): ");
					Serial.print(core1_min_free_stack); Serial.print(" / "); Serial.println(core1_total);
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