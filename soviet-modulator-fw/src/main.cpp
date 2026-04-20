#include <Arduino.h>
#include "bpsk_modulator.h"

#define MODULATING_PIN 20
#define LED_PIN 25
#define INPUT_BUFFER_SIZE 512

BPSKModulator modulator(MODULATING_PIN, 1000);
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

// Test mode
static bool test_mode = false;
static unsigned long last_pulse = 0;
static bool pulse_state = false;

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	pinMode(23, OUTPUT);
	digitalWrite(23, HIGH); // Set pin 23 high so DC/DC isn't fucking shitfest
	
	Serial.begin(115200);
	delay(1000);
	
	Serial.println("BPSK Modulator initialized");
	Serial.print("Modulation GPIO: ");
	Serial.println(MODULATING_PIN);
	Serial.println("Commands:");
	Serial.println("  r <rate> - Set symbol rate (1-100000 Hz)");
	Serial.println("  s - Start modulation");
	Serial.println("  p - Stop modulation");
	Serial.println("  i - Print current rate");
	Serial.println("  f - Toggle filler transmission");
	Serial.println("  t <data> - Transmit message (ASCII text)");
	Serial.println("  d - Toggle debug pulse (software 100ms toggle on pin 20)");
	
	frame_start = millis();
}

void process_input(char mode, const char* data) {
	Serial.println("");

  	if (mode == 'r') {
		uint32_t rate = atol(data);
		modulator.set_symbolrate(rate);
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
	  		modulator.queue_message(msg, len);
	  		Serial.print("Message queued: ");
	  		Serial.print(len);
	  		Serial.println(" bytes");
		}
  	}
}

void loop() {
  	// LED heartbeat: toggle every 500ms for 1s total blink cycle
  	unsigned long now = millis();
  	if (now - last_blink >= 500) {
		led_state = !led_state;
		digitalWrite(LED_PIN, led_state ? HIGH : LOW);
		last_blink = now;
  	}

  	// Debug pulse on pin 20 (100ms toggle)
  	if (test_mode) {
		if (now - last_pulse >= 100) {
  			pulse_state = !pulse_state;
  			digitalWrite(MODULATING_PIN, pulse_state ? HIGH : LOW);
  			last_pulse = now;
  		}
  	}

  	if (modulator.has_pending_message()) {
		if (now - frame_start >= FRAME_PERIOD_MS) {
	  	modulator.inject_message(filler_enabled);
	  	frame_start = now;
		}
  	}

	modulator.service();

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
				  	modulator.start();
				  	Serial.println("Modulation started");
				  	Serial.print("SM: ");
				  	Serial.println(modulator.get_sm());
				  	Serial.print("FIFO state - empty: ");
				  	Serial.print(modulator.tx_fifo_empty());
				  	Serial.print(", full: ");
				  	Serial.println(modulator.tx_fifo_full());
				  	break;
				}
			case 'p': {
			  modulator.stop();
			  Serial.println("Modulation stopped");
			  break;
			}
			case 'i': {
			 	 Serial.print("Current symbol rate: ");
			 	 Serial.print(modulator.get_symbolrate());
			 	 Serial.println(" Hz");
			 	 break;
			}
			case 'f': {
				filler_enabled = !filler_enabled;
				if (filler_enabled) {
					modulator.init_frame();
					Serial.println("Filler transmission enabled");
				} else {
					memset(modulator.frame, 0, FRAME_SIZE);
					Serial.println("Filler transmission disabled");
				}
			  	break;
			}
			case 't': {
			  	Serial.println("Enter message:");
			  	waiting_for_input = true;
			  	input_mode = 't';
			  	input_idx = 0;
			  	break;
			}
			case 'd': {
				test_mode = !test_mode;
				if (test_mode) {
					modulator.release_pin_to_sio();
					Serial.println("Debug pulse ON - SIO pin toggle every 100ms");
					pulse_state = false;
					digitalWrite(MODULATING_PIN, LOW);
					last_pulse = millis();
				} else {
					Serial.println("Debug pulse OFF");
					digitalWrite(MODULATING_PIN, LOW);
					modulator.start();
				}
				break;
			}
			default:
			  Serial.println("Unknown command");
			  break;
		  }
		}
  	}
}