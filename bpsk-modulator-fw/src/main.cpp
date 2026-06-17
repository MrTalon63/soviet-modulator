#include "bpsk_modulator.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <hardware/adc.h>
#include <hardware/sync.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <new>
#include <pico.h>
#include <si5351.h>

#define MODULATING_PIN 20
#define LED_PIN 25
#define INPUT_BUFFER_SIZE 1024
#define SI5351_SDA_PIN 26
#define SI5351_SCL_PIN 27

#ifndef PIN_SMPS_MODE
#define PIN_SMPS_MODE 23 // Fallback for RP2040 boards if macro is missing
#endif

#ifndef ADC_CHANNEL_TEMPSENSE
#if defined(ARDUINO_ARCH_RP2350) || defined(PICO_RP2350)
#define ADC_CHANNEL_TEMPSENSE 8
#else
#define ADC_CHANNEL_TEMPSENSE 4
#endif
#endif

// --- Si5351 calibration stored in EEPROM ---
struct Si5351Cal {
    uint32_t magic;      // CALIB_MAGIC if valid
    uint64_t lo_freq_hz; // LO frequency in Hz
    int32_t correction;  // Crystal correction in units of 0.01 Hz (as si5351 expects)
};
static constexpr uint32_t CALIB_MAGIC = 0xB5CAEC51UL;
static constexpr uint64_t LO_FREQ_DEFAULT = 144500000ULL;
static constexpr int EEPROM_CAL_ADDR = 0;

static uint64_t lo_freq_hz = LO_FREQ_DEFAULT;
static int32_t lo_correction = 0; // 0.01 Hz units

void calib_load() {
    EEPROM.begin(256);
    Si5351Cal cal;
    EEPROM.get(EEPROM_CAL_ADDR, cal);
    if (cal.magic == CALIB_MAGIC) {
        lo_freq_hz = cal.lo_freq_hz;
        lo_correction = cal.correction;
        Serial.print("[CALIB] Loaded: ");
        Serial.print((uint32_t)(lo_freq_hz / 1000000ULL));
        Serial.print(".");
        Serial.print((uint32_t)(lo_freq_hz % 1000000ULL));
        Serial.print(" MHz, correction=");
        Serial.print(lo_correction);
        Serial.println(" (0.01Hz units)");
    } else {
        Serial.println("[CALIB] No saved calibration, using defaults.");
    }
}

void calib_save() {
    Si5351Cal cal;
    cal.magic = CALIB_MAGIC;
    cal.lo_freq_hz = lo_freq_hz;
    cal.correction = lo_correction;
    EEPROM.put(EEPROM_CAL_ADDR, cal);
    EEPROM.commit();
    Serial.println("[CALIB] Saved to flash.");
}

void calib_reset() {
    lo_freq_hz = LO_FREQ_DEFAULT;
    lo_correction = 0;
    // Erase magic so next boot uses defaults
    Si5351Cal blank = {0, 0, 0};
    EEPROM.put(EEPROM_CAL_ADDR, blank);
    EEPROM.commit();
    Serial.println("[CALIB] Reset to defaults and cleared flash.");
}

alignas(4) volatile uint8_t global_sp_fifo[SP_FIFO_SIZE] __attribute__((section(".uninitialized")));

alignas(BPSKModulator) static uint8_t modulator_memory[sizeof(BPSKModulator)];
static BPSKModulator *modulator = nullptr;
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
static bool pin_test_mode = false;

static unsigned long current_frame_period_ms() {
    if (modulator == nullptr)
        return 100;
    uint32_t rate = modulator->get_symbolrate();
    if (rate == 0)
        rate = 1;

    uint32_t output_symbols_per_frame;
    if (modulator->get_convolutional_encoding()) {
        double code_rate = 1.0;
        switch (modulator->get_conv_rate()) {
        case 0:
            code_rate = 1.0 / 2.0;
            break;
        case 1:
            code_rate = 2.0 / 3.0;
            break;
        case 2:
            code_rate = 3.0 / 4.0;
            break;
        case 3:
            code_rate = 5.0 / 6.0;
            break;
        case 4:
            code_rate = 7.0 / 8.0;
            break;
        }
        output_symbols_per_frame = (uint32_t)((modulator->get_frame_size() * 8.0) / code_rate);
    } else {
        output_symbols_per_frame = modulator->get_frame_size() * 8U;
    }

    uint32_t period_ms = (output_symbols_per_frame * 1000U + rate - 1U) / rate;
    if (period_ms < 1U)
        period_ms = 1U;
    return (unsigned long)(period_ms + 2U);
}

/*
Command summary:
    r <rate> - Set symbol rate (1-100000 Hz)
    s - Start modulation
    p - Stop modulation
    i - Print current rate
    c - Toggle CCSDS convolutional encoding
    n - Toggle CCSDS randomizer
    y - Toggle Reed-Solomon (255,223) I=4 encoding
    L - Toggle CCSDS LDPC (8176, 7154) Rate 7/8 encoding
    l <count> - Set Reed-Solomon interleaver depth (1, 2, 4, 5, 8)
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

void apply_si5351_lo() {
    // Apply the currently active lo_freq_hz + correction to the hardware
    si5351.set_correction(lo_correction, SI5351_PLL_INPUT_XO);
    si5351.set_freq(lo_freq_hz * SI5351_FREQ_MULT, SI5351_CLK0);
    Serial.print("Si5351 LO: ");
    Serial.print((uint32_t)(lo_freq_hz / 1000000ULL));
    Serial.print(".");
    // Print fractional Hz with 6 digits
    uint32_t frac = (uint32_t)(lo_freq_hz % 1000000ULL);
    if (frac < 100000)
        Serial.print("0");
    if (frac < 10000)
        Serial.print("0");
    if (frac < 1000)
        Serial.print("0");
    if (frac < 100)
        Serial.print("0");
    if (frac < 10)
        Serial.print("0");
    Serial.print(frac);
    Serial.print(" MHz  correction=");
    Serial.print(lo_correction);
    Serial.println(" (0.01 Hz units)");
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
    si5351.output_enable(SI5351_CLK0, 1);
    apply_si5351_lo();
}

[[noreturn]] void reboot_microcontroller() {
    Serial.println("Rebooting microcontroller");
    Serial.flush();
    delay(50);
    watchdog_reboot(0, 0, 0);
    while (true) {}
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
    Serial.println("  N - Toggle CCSDS randomizer polynomial (8-bit vs 17-bit)");
    Serial.println("  q - Print upload/FIFO status");
    Serial.println("  l <count> - Set Reed-Solomon interleaver depth (1, 2, 4, 5, 8)");
    Serial.println("  t <data> - Transmit message (ASCII text)");
    Serial.println("  y - Toggle Reed-Solomon (255,223) I=4 encoding");
    Serial.println("  L - Toggle CCSDS LDPC (8160, 7136) Rate 7/8 encoding");
    Serial.println("  e - Toggle Frame Error Control Field (FECF)");
    Serial.println("  u - Binary upload mode (raw Space Packets, timeout 5s exits)");
    Serial.println("  x - Reinitialize Si5351 LO from saved calibration");
    Serial.println("  f <hz> - Set Si5351 LO frequency in Hz (e.g. 144500000)");
    Serial.println("  v/V - Nudge LO frequency -100/+100 Hz (fine tune)");
    Serial.println("  C <ppb*10> - Set Si5351 crystal correction (0.01 Hz units, e.g. -150)");
    Serial.println("  X - Save current Si5351 calibration to flash");
    Serial.println("  D - Reset Si5351 calibration to factory defaults");
    Serial.println("  o - Queue OID (Operational Idle Data) frame (VCID=0x3F)");
    Serial.println("  Z - Hex dump the last generated baseband frame");
    Serial.println("  I <vcid> - Queue idle data MPDU frame with specified VCID (0-63)");
    Serial.println("  O - Output OID pattern (first 10 bytes) for verification");
    Serial.println("  m - Restart whole microcontroller");
    Serial.println("  T - CPU Pin Test Mode (Toggle GP0-8 at 100 Hz)");
    Serial.println("  W - Toggle RRC/RC pulse shaping filter");
    Serial.println("  R - Configure RRC/RC filter (alpha, min_dac_rate, span, type)");
    Serial.println("  h - Print this help menu");
}

void setup() {
    // Fix coil whine & boot crashes: Force DC/DC into PWM mode BEFORE increasing power draw!
    pinMode(PIN_SMPS_MODE, OUTPUT);
    digitalWrite(PIN_SMPS_MODE, HIGH); // Use hardware-agnostic macro (GPIO 23 on Pico 1, mapped internally on Pico 2)
    delay(10);

    vreg_disable_voltage_limit();        // Unlock higher voltages for the RP2350
    vreg_set_voltage(VREG_VOLTAGE_1_30); // Manually set internal core voltage to 1.30V to stabilize 400 MHz
    delay(100);                          // Allow internal LDO to charge up fully
    set_sys_clock_khz(400000, true);     // Safely apply 400 MHz overclock
    delay(100);                          // Allow hardware peripherals (Si5351) a fraction of a second to cleanly power up

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    Serial.begin(921600);
    rs_init(); // Initialize Reed-Solomon tables BEFORE the modulator generates its OID frame!

    modulator = new (modulator_memory) BPSKModulator(MODULATING_PIN, global_sp_fifo, 500000);
    if (modulator == nullptr) {
        Serial.println("Modulator allocation failed");
        return;
    }
    calib_load();
    init_si5351_i2c();
    init_si5351_lo();

    Serial.println("BPSK Modulator initialized");
    Serial.println("  Mode: PIO-based (hardware)");
    Serial.print("Modulation GPIO: ");
    Serial.println(MODULATING_PIN);
    print_help();

    frame_start = millis();
}

void process_input(char mode, const char *data) {
    Serial.println("");

    if (mode == 'r') {
        uint32_t rate = atol(data);
        if (modulator != nullptr)
            modulator->set_symbolrate(rate);
        Serial.print("Symbol rate set to: ");
        Serial.println(rate);
    } else if (mode == 'k') {
        uint8_t rate = atoi(data);
        if (modulator != nullptr)
            modulator->set_conv_rate(rate);
        Serial.print("Convolutional rate set to: ");
        switch (modulator->get_conv_rate()) {
        case 0:
            Serial.println("1/2");
            break;
        case 1:
            Serial.println("2/3");
            break;
        case 2:
            Serial.println("3/4");
            break;
        case 3:
            Serial.println("5/6");
            break;
        case 4:
            Serial.println("7/8");
            break;
        }
    } else if (mode == 'l') {
        uint8_t interleave = atoi(data);
        if (interleave == 1 || interleave == 2 || interleave == 4 || interleave == 5 || interleave == 8) {
            if (modulator != nullptr)
                modulator->set_rs_interleave(interleave);
            Serial.print("RS interleave set to: ");
            Serial.println(interleave);
        } else {
            Serial.println("ERROR: RS interleave must be 1, 2, 4, 5, or 8");
        }
    } else if (mode == 't') {
        uint16_t len = strlen(data);
        if (len > MSG_BUFFER_SIZE)
            len = MSG_BUFFER_SIZE;

        if (len > 0) {
            uint8_t msg[MSG_BUFFER_SIZE];
            for (uint16_t i = 0; i < len; i++) {
                msg[i] = (uint8_t)data[i];
            }
            if (modulator != nullptr)
                modulator->queue_message(msg, len);
            Serial.print("Message queued: ");
            Serial.print(len);
            Serial.println(" bytes");
        }
    } else if (mode == 'I') {
        uint8_t vcid = atoi(data);
        if (vcid > 63) {
            Serial.println("ERROR: VCID must be 0-63");
        } else {
            if (modulator != nullptr)
                modulator->queue_idle_data_frame(vcid);
            Serial.print("Idle data frame queued with VCID=");
            Serial.println(vcid);
        }
    } else if (mode == 'R') {
        float alpha = 0.35f;
        uint32_t min_dac_rate = 20000000;
        int span = 8;
        int type = 1;

        if (sscanf(data, "%f %lu %d %d", &alpha, &min_dac_rate, &span, &type) >= 1) {
            if (alpha < 0.05f || alpha > 0.95f) {
                Serial.println("ERROR: alpha must be between 0.05 and 0.95");
                return;
            }
            if (min_dac_rate < 10000 || min_dac_rate > 100000000) {
                Serial.println("ERROR: min_dac_rate must be between 10000 and 100000000");
                return;
            }
            if (span < 2 || span > 12) {
                Serial.println("ERROR: filter span must be between 2 and 12 symbols");
                return;
            }
            if (type != 0 && type != 1) {
                Serial.println("ERROR: type must be 0 (Raised Cosine) or 1 (Root Raised Cosine)");
                return;
            }
            if (modulator != nullptr) {
                modulator->update_rrc_config(true, alpha, (uint8_t)span, min_dac_rate, type == 1);
                Serial.println("RRC/RC filter configured and enabled.");
            }
        } else {
            Serial.println("ERROR: Invalid format. Expected space-separated parameters.");
        }
    } else if (mode == 'f') {
        uint64_t hz = (uint64_t)strtoul(data, nullptr, 10);
        if (hz < 1000000ULL || hz > 200000000ULL) {
            Serial.println("ERROR: Frequency must be 1-200 MHz");
            return;
        }
        lo_freq_hz = hz;
        apply_si5351_lo();
    } else if (mode == 'C') {
        int32_t corr = (int32_t)strtol(data, nullptr, 10);
        lo_correction = corr;
        apply_si5351_lo();
        Serial.print("Correction set to: ");
        Serial.print(corr);
        Serial.println(" (0.01 Hz units)");
    }
}

// setup1() and loop1() natively execute on Core 1 in Arduino-Pico
void setup1() {
    // Wait for modulator to be dynamically initialized by Core 0
    while (modulator == nullptr) {
        delay(10);
    }
}

void __not_in_flash_func(loop1)() {
    if (modulator == nullptr)
        return;

    // Yield completely if modulator is not running or settings are actively being changed
    __dmb(); // Ensure we see the latest config_lock/running states from Core 0
    if (!modulator->is_running() || modulator->config_lock) {
        modulator->core1_processing = false;
        __dmb();
        delayMicroseconds(50); // Prevent Core 1 from starving the system bus
        return;
    }

    modulator->core1_processing = true;
    __dmb();

    // 1. Keep the DMA queue permanently full of dynamically encoded LDPC frames.
    // If we have user data, pack it. If not, dynamically generate an OID frame (VCID 63)
    // so the frame counter increments, avalanching the parity to prevent static false-syncs!
    if (modulator->can_queue_binary_frame()) {
        uint32_t t0 = time_us_32();
        if (modulator->has_data_to_send()) {
            modulator->generate_and_queue_mpdu();
        } else {
            modulator->generate_and_queue_idle_frame(0x3F); // Dynamically encode VCID 63
        }
        modulator->perf_mpdu_us = time_us_32() - t0;
    }

    // 2. Drain ONE baseband frame into the DMA chunk queue
    uint32_t t1 = time_us_32();
    if (modulator->process_baseband_to_dma()) {
        modulator->perf_bb_us = time_us_32() - t1;
    } else {
        modulator->core1_processing = false;
        __dmb();
        delayMicroseconds(50); // Prevent Core 1 from starving the system bus
        return;
    }

    modulator->core1_processing = false;
    __dmb();
}

void loop() {
    if (modulator == nullptr) {
        return;
    }

    // CPU Pin Test Mode
    static unsigned long last_test_toggle = 0;
    static bool test_pins_state = false;
    if (pin_test_mode) {
        unsigned long now = millis();
        if (now - last_test_toggle >= 5) { // 5ms high, 5ms low = 100Hz
            test_pins_state = !test_pins_state;
            // Set data pins GP1-8 (DAC Parallel Data) first
            for (int p = 1; p <= 8; p++) {
                gpio_put(p, test_pins_state);
            }
            // Setup time delay
            delayMicroseconds(1);
            // Pulse GP0 (BCLK) low, then high to clock the data into the DAC
            gpio_put(0, 0);
            delayMicroseconds(1);
            gpio_put(0, 1);
            last_test_toggle = now;
        }
    }

    // LED heartbeat: toggle every 500ms for 1s total blink cycle
    unsigned long now = millis();
    if (now - last_blink >= 500) {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state ? HIGH : LOW);
        last_blink = now;
    }

    // Telemetry Heartbeat during binary streaming
    static unsigned long last_telemetry = 0;

    if (!pin_test_mode && binary_upload_mode && (now - last_telemetry >= 100)) { // 10Hz to provide low-latency flow control feedback

        float temp_c = analogReadTemp(); // Arduino-Pico core natively abstracts RP2040 vs RP2350 hw sensors

        uint32_t symbols_per_frame = modulator->get_frame_size() * 8;
        if (modulator->get_convolutional_encoding()) {
            switch (modulator->get_conv_rate()) {
            case 0:
                symbols_per_frame = modulator->get_frame_size() * 16;
                break;
            case 1:
                symbols_per_frame = (modulator->get_frame_size() * 8 * 3) / 2;
                break;
            case 2:
                symbols_per_frame = (modulator->get_frame_size() * 8 * 4) / 3;
                break;
            case 3:
                symbols_per_frame = (modulator->get_frame_size() * 8 * 6) / 5;
                break;
            case 4:
                symbols_per_frame = (modulator->get_frame_size() * 8 * 8) / 7;
                break;
            }
        }
        uint32_t frame_tx_time_us = (uint32_t)((symbols_per_frame * 1000000ULL) / modulator->get_symbolrate());
        uint32_t cpu_time_us = modulator->perf_mpdu_us + modulator->perf_bb_us;
        float headroom_pct = (1.0f - ((float)cpu_time_us / (float)frame_tx_time_us)) * 100.0f;

        Serial.print("[MCU] FIFO: ");
        Serial.print(modulator->get_sp_fifo_count());
        Serial.print("/");
        Serial.print(SP_FIFO_SIZE);
        Serial.print(" | TX Frames: ");
        Serial.print(modulator->get_pending_frames_count());
        Serial.print(" | Headroom: ");
        Serial.print(headroom_pct, 1);
        Serial.print("% | Underflows: ");
        Serial.print(modulator->get_dma_underflows());
        Serial.print(" | Temp: ");
        Serial.print(temp_c, 1);
        Serial.println("C");
        last_telemetry = now;
    }

    // Safety Timeout: Escape Binary Mode after 15 seconds of silence.
    if (binary_upload_mode && !Serial.available()) {
        if (now - last_binary_rx_ms > 15000) {
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
        if (!Serial.available())
            return;

        char c = Serial.read();

        if (c == 'u') {
            waiting_for_input = false;
            input_idx = 0;
            binary_upload_mode = true;

            modulator->clear_sp_fifo(); // Safely reset parsing state WITHOUT violently aborting the DMA!

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
                Serial.write(c); // Echo back
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
            case 'l': {
                Serial.println("Enter RS interleave depth (1, 2, 4, 5, 8):");
                waiting_for_input = true;
                input_mode = 'l';
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
            case 'N': {
                uint8_t current_poly = modulator->get_randomizer_poly();
                uint8_t next_poly = (current_poly == 8) ? 17 : 8;
                modulator->set_randomizer_poly(next_poly);
                Serial.print("CCSDS randomizer polynomial: ");
                Serial.print(modulator->get_randomizer_poly());
                Serial.println("-bit");
                break;
            }
            case 'y': {
                bool current = modulator->get_reed_solomon_enabled();
                modulator->set_reed_solomon_enabled(!current);
                Serial.print("Reed-Solomon (255,223) I=4: ");
                Serial.println(modulator->get_reed_solomon_enabled() ? "ON" : "OFF");
                break;
            }
            case 'L': {
                bool current = modulator->get_ldpc_enabled();
                modulator->set_ldpc_enabled(!current);
                Serial.print("CCSDS LDPC (8160, 7136) Rate 7/8: ");
                Serial.println(modulator->get_ldpc_enabled() ? "ON" : "OFF");
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
                float temp_c = analogReadTemp();
                Serial.println("Modulator status:");
                Serial.print("  Core Temp: ");
                Serial.print(temp_c, 1);
                Serial.println(" C");
                Serial.print("  Symbol Rate: ");
                Serial.print(modulator->get_symbolrate());
                Serial.println(" Hz");
                Serial.print("  Frame Size: ");
                Serial.print(modulator->get_frame_size());
                Serial.println(" bytes");
                Serial.print("  RS (255,223): ");
                Serial.println(modulator->get_reed_solomon_enabled() ? "ON" : "OFF");
                Serial.print("  RS Interleave: ");
                Serial.println(modulator->get_rs_interleave());
                Serial.print("  LDPC (8160,7136): ");
                Serial.println(modulator->get_ldpc_enabled() ? "ON" : "OFF");
                Serial.print("  FECF (CRC-16): ");
                Serial.println(modulator->get_fecf_enabled() ? "ON" : "OFF");
                Serial.print("  Randomizer: ");
                if (modulator->get_randomizer_enabled()) {
                    Serial.print("ON (Poly: ");
                    Serial.print(modulator->get_randomizer_poly());
                    Serial.println("-bit)");
                } else {
                    Serial.println("OFF");
                }
                Serial.print("  Convolutional: ");
                Serial.print(modulator->get_convolutional_encoding() ? "ON" : "OFF");
                if (modulator->get_convolutional_encoding()) {
                    Serial.print(" (Rate ");
                    switch (modulator->get_conv_rate()) {
                    case 0:
                        Serial.print("1/2");
                        break;
                    case 1:
                        Serial.print("2/3");
                        break;
                    case 2:
                        Serial.print("3/4");
                        break;
                    case 3:
                        Serial.print("5/6");
                        break;
                    case 4:
                        Serial.print("7/8");
                        break;
                    }
                    Serial.print(")");
                }
                Serial.println();
                Serial.print("  RRC Filter: ");
                if (modulator->get_rrc_enabled()) {
                    Serial.print("ON (Type: ");
                    Serial.print(modulator->get_use_rrc() ? "RRC" : "RC");
                    Serial.print(", alpha=");
                    Serial.print(modulator->get_rrc_alpha());
                    Serial.print(", L=");
                    Serial.print(modulator->get_rrc_L());
                    Serial.print(", span=");
                    Serial.print(modulator->get_rrc_filter_span());
                    Serial.print(", Fsample=");
                    Serial.print((float)(modulator->get_symbolrate() * modulator->get_rrc_L()) / 1000000.0f, 3);
                    Serial.println(" MHz)");
                } else {
                    Serial.println("OFF");
                }
                Serial.print("  Expected Payload: ");
                uint16_t ep;
                if (modulator->get_ldpc_enabled()) {
                    ep = 892 - (modulator->get_fecf_enabled() ? 2 : 0) - 8;
                } else {
                    ep = (modulator->get_reed_solomon_enabled() ? modulator->get_rs_input_size() : modulator->get_rs_output_size()) - (modulator->get_fecf_enabled() ? 2 : 0) - 8;
                }
                Serial.print(ep);
                Serial.println(" bytes");
                Serial.println("Space Packet FIFO status:");
                Serial.print("  FIFO depth: ");
                Serial.print(modulator->get_sp_fifo_count());
                Serial.print("/");
                Serial.print(SP_FIFO_SIZE);
                Serial.println(" bytes");
                Serial.print("  TX Frames pending: ");
                Serial.println(modulator->get_pending_frames_count());
                Serial.print("  DMA Chunks pending: ");
                Serial.println(modulator->get_dma_chunks_count());
                Serial.print("  Active User Chunks: ");
                Serial.println(modulator->get_active_user_chunks());
                Serial.print("  DMA Underflows: ");
                Serial.println(modulator->get_dma_underflows());
                modulator->print_debug_info();
                Serial.print("  upload active: ");
                Serial.println(binary_upload_mode ? "YES" : "NO");

                Serial.println("Performance Profiling:");
                uint32_t symbols_per_frame = modulator->get_frame_size() * 8;
                if (modulator->get_convolutional_encoding()) {
                    switch (modulator->get_conv_rate()) {
                    case 0:
                        symbols_per_frame = modulator->get_frame_size() * 16;
                        break;
                    case 1:
                        symbols_per_frame = (modulator->get_frame_size() * 8 * 3) / 2;
                        break;
                    case 2:
                        symbols_per_frame = (modulator->get_frame_size() * 8 * 4) / 3;
                        break;
                    case 3:
                        symbols_per_frame = (modulator->get_frame_size() * 8 * 6) / 5;
                        break;
                    case 4:
                        symbols_per_frame = (modulator->get_frame_size() * 8 * 8) / 7;
                        break;
                    }
                }
                uint32_t frame_tx_time_us = (uint32_t)((symbols_per_frame * 1000000ULL) / modulator->get_symbolrate());
                uint32_t cpu_time_us = modulator->perf_mpdu_us + modulator->perf_bb_us;
                float headroom_pct = (1.0f - ((float)cpu_time_us / (float)frame_tx_time_us)) * 100.0f;

                Serial.print("  MPDU Generation: ");
                Serial.print(modulator->perf_mpdu_us);
                Serial.println(" us");
                Serial.print("  Baseband + DMA:  ");
                Serial.print(modulator->perf_bb_us);
                Serial.println(" us");
                Serial.print("  Total CPU Time:  ");
                Serial.print(cpu_time_us);
                Serial.print(" us / ");
                Serial.print(frame_tx_time_us);
                Serial.println(" us available");
                Serial.print("  Core 1 Headroom: ");
                Serial.print(headroom_pct, 1);
                Serial.println("%");
                if (headroom_pct < 0)
                    Serial.println("  [!] WARNING: CPU Too Slow! Pipeline Starvation Occurring!");
                break;
            }
            case 'x': {
                Serial.println("Reinitializing Si5351 LO from saved calibration");
                calib_load(); // re-read from flash in case it changed
                init_si5351_lo();
                break;
            }
            case 'f': {
                Serial.println("Enter LO frequency in Hz (e.g. 144500000):");
                waiting_for_input = true;
                input_mode = 'f';
                input_idx = 0;
                break;
            }
            case 'v': {
                lo_freq_hz = (lo_freq_hz > 100) ? lo_freq_hz - 100 : lo_freq_hz;
                Serial.print("LO nudged -100 Hz -> ");
                Serial.print((uint32_t)lo_freq_hz);
                Serial.println(" Hz");
                apply_si5351_lo();
                break;
            }
            case 'V': {
                lo_freq_hz += 100;
                Serial.print("LO nudged +100 Hz -> ");
                Serial.print((uint32_t)lo_freq_hz);
                Serial.println(" Hz");
                apply_si5351_lo();
                break;
            }
            case 'C': {
                Serial.println("Enter correction (0.01 Hz units, e.g. -150 for -1.5 Hz):");
                waiting_for_input = true;
                input_mode = 'C';
                input_idx = 0;
                break;
            }
            case 'X': {
                calib_save();
                break;
            }
            case 'D': {
                calib_reset();
                apply_si5351_lo();
                break;
            }
            case 'm': {
                reboot_microcontroller();
                break;
            }
            case 'O': {
                // Output first 10 bytes of OID frame for verification
                Serial.println("OID pattern (first 10 bytes):");
                uint8_t *oid_frame = modulator->get_oid_frame();
                for (int i = 0; i < 10; i++) {
                    if (oid_frame[i] < 0x10)
                        Serial.print("0");
                    Serial.print(oid_frame[i], HEX);
                    if (i < 9)
                        Serial.print(" ");
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
            case 'Z': {
                Serial.println("Dumping last generated baseband frame (Hex):");
                uint8_t *tx_frame = modulator->frame;
                uint16_t len = modulator->get_frame_size();
                for (uint16_t i = 0; i < len; i++) {
                    if (tx_frame[i] < 0x10)
                        Serial.print("0");
                    Serial.print(tx_frame[i], HEX);
                    Serial.print(" ");
                    if ((i + 1) % 32 == 0)
                        Serial.println();
                }
                Serial.println();
                break;
            }
            case 'I': {
                Serial.println("Enter VCID (0-63, default 0):");
                waiting_for_input = true;
                input_mode = 'I';
                input_idx = 0;
                break;
            }
            case 'T': {
                pin_test_mode = !pin_test_mode;
                if (pin_test_mode) {
                    if (modulator != nullptr) {
                        modulator->lock_config();
                        modulator->stop();
                        modulator->unlock_config();
                    }
                    // Configure all 9 pins as normal SIO GPIO outputs
                    for (int p = 0; p <= 8; p++) {
                        gpio_init(p);
                        gpio_set_dir(p, GPIO_OUT);
                    }
                    Serial.println("CPU Pin Test Mode: ACTIVE (toggling GP0-GP8 at 100 Hz)");
                } else {
                    // Restore pins to PIO function
                    for (int p = 0; p <= 8; p++) {
                        pio_gpio_init(pio0, p);
                    }
                    Serial.println("CPU Pin Test Mode: INACTIVE (pins restored to PIO)");
                }
                break;
            }
            case 'W': {
                if (modulator != nullptr) {
                    bool current = modulator->get_rrc_enabled();
                    modulator->update_rrc_config(!current, modulator->get_rrc_alpha(), modulator->get_rrc_filter_span(), modulator->get_rrc_min_dac_rate(), modulator->get_use_rrc());
                    Serial.print("RRC/RC filter toggled: ");
                    Serial.println(modulator->get_rrc_enabled() ? "ON" : "OFF");
                }
                break;
            }
            case 'R': {
                Serial.println("Enter alpha min_dac_rate span type (e.g. 0.35 20000000 8 1):");
                waiting_for_input = true;
                input_mode = 'R';
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

// Global instance definition for the interrupt handler
BPSKModulator *g_modulator_instance = nullptr;

// Interrupt handler executed in RAM (not in Flash)
void __not_in_flash_func(bpsk_dma_isr)() {
    if (g_modulator_instance != nullptr) {
        g_modulator_instance->handle_dma_irq();
    }
}