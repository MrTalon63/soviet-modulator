#ifndef BPSK_MODULATOR_H
#define BPSK_MODULATOR_H

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/pio_instructions.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>

#define FRAME_SIZE 1024
#define CCSDS_ASM_SIZE 4
#define CCSDS_ASM 0x1ACFFC1D
#define MSG_BUFFER_SIZE 512

// PIO program for BPSK modulator
// Pull one word per byte period, then shift 8 bits to the output pin.
static const uint16_t bpsk_modulator_program[] = {
    pio_encode_pull(false, false), // 0: pull noblock
    pio_encode_set(pio_x, 7),      // 1: set x, 7
    pio_encode_out(pio_pins, 1),   // 2: out pins, 1
    pio_encode_jmp_x_dec(2),       // 3: jmp x--, 2
};

static const pio_program_t bpsk_modulator_program_default = {
    .instructions = bpsk_modulator_program,
    .length = 4,
    .origin = -1,
};

class BPSKModulator {
private:
    PIO pio;
    uint sm;
    uint pin;
    uint32_t symbolrate_hz;
    uint pio_offset;
    uint16_t tx_index;
    uint16_t msg_len;
    bool msg_pending;
    bool running;
    bool restore_filler_after_message;

public:
    uint8_t frame[FRAME_SIZE];
    uint8_t msg_buffer[MSG_BUFFER_SIZE];
    BPSKModulator(uint pin, uint32_t initial_symbolrate = 1000) 
        : pio(pio0), sm(0), pin(pin), symbolrate_hz(initial_symbolrate), pio_offset(0), tx_index(0), msg_len(0), msg_pending(false), running(false), restore_filler_after_message(false) {
        init_frame();
        init_pio(pin);
    }

    void init_frame() {
        // CCSDS ASM (4 bytes)
        frame[0] = (CCSDS_ASM >> 24) & 0xFF;
        frame[1] = (CCSDS_ASM >> 16) & 0xFF;
        frame[2] = (CCSDS_ASM >> 8) & 0xFF;
        frame[3] = CCSDS_ASM & 0xFF;

        // Rest: repeated 0x55 (01010101 pattern)
        for (uint16_t i = CCSDS_ASM_SIZE; i < FRAME_SIZE; i++) {
            frame[i] = 0x55;
        }
    }

    void init_frame_with_message(const uint8_t* msg, uint16_t len) {
        // CCSDS ASM (4 bytes)
        frame[0] = (CCSDS_ASM >> 24) & 0xFF;
        frame[1] = (CCSDS_ASM >> 16) & 0xFF;
        frame[2] = (CCSDS_ASM >> 8) & 0xFF;
        frame[3] = CCSDS_ASM & 0xFF;

        // Copy message
        uint16_t pos = CCSDS_ASM_SIZE;
        for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++) {
            frame[pos] = msg[i];
        }

        // Pad rest with 0x55 (10101010)
        for (uint16_t i = pos; i < FRAME_SIZE; i++) {
            frame[i] = 0x55;
        }
    }

    void init_frame_message_only(const uint8_t* msg, uint16_t len) {
        // Message at beginning
        uint16_t pos = 0;
        for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++) {
            frame[pos] = msg[i];
        }

        // Pad rest with 0x55 (10101010 pattern)
        for (uint16_t i = pos; i < FRAME_SIZE; i++) {
            frame[i] = 0x55;
        }
    }

    void init_pio(uint pin) {
        // Load PIO program
        pio_offset = pio_add_program(pio, &bpsk_modulator_program_default);

        // Claim an unused SM so this works even when SM0 is already taken.
        sm = pio_claim_unused_sm(pio, true);

        // Initialize state machine config
        pio_sm_config c = pio_get_default_sm_config();
        
        // Set output pin (shift out to this pin)
        sm_config_set_out_pins(&c, pin, 1);

        // Limit execution to this instruction sequence.
        sm_config_set_wrap(&c, pio_offset, pio_offset + bpsk_modulator_program_default.length - 1);
        
        // Right shift; pulls are handled explicitly in the PIO program.
        sm_config_set_out_shift(&c, true, false, 32);

        // Hand GPIO over to selected PIO instance.
        pio_gpio_init(pio, pin);

        // Initialize and start SM
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_init(pio, sm, pio_offset, &c);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        
        set_symbolrate(symbolrate_hz);
        pio_sm_set_enabled(pio, sm, true);
        running = true;

        // Fill TX FIFO with initial data
        refill_fifo();
    }

    void refill_fifo() {
        // Keep FIFO topped up and preserve position in the frame.
        while (!pio_sm_is_tx_fifo_full(pio, sm)) {
            pio_sm_put(pio, sm, frame[tx_index]);
            tx_index++;
            if (tx_index >= FRAME_SIZE) {
                tx_index = 0;
                if (restore_filler_after_message) {
                    // Message frame has been fully queued once; resume default filler frame.
                    init_frame();
                    restore_filler_after_message = false;
                }
            }
        }
    }

    void queue_message(const uint8_t* msg, uint16_t len) {
        if (len > MSG_BUFFER_SIZE) len = MSG_BUFFER_SIZE;
        memcpy(msg_buffer, msg, len);
        msg_len = len;
        msg_pending = true;
    }

    bool has_pending_message() const {
        return msg_pending;
    }

    bool tx_fifo_empty() const {
        return pio_sm_is_tx_fifo_empty(pio, sm);
    }

    bool tx_fifo_full() const {
        return pio_sm_is_tx_fifo_full(pio, sm);
    }

    uint get_sm() const {
        return sm;
    }

    void inject_message(bool filler_enabled) {
        if (!msg_pending || msg_len == 0) return;

        if (filler_enabled) {
            init_frame_with_message(msg_buffer, msg_len);
            restore_filler_after_message = true;
        } else {
            init_frame_message_only(msg_buffer, msg_len);
            restore_filler_after_message = false;
        }

        msg_pending = false;
        tx_index = 0;
        refill_fifo();
    }

    void service() {
        if (!running) return;
        refill_fifo();
    }

    void set_symbolrate(uint32_t hz) {
        if (hz < 1 || hz > 100000) return;

        // Use runtime sys clock (125 MHz on RP2040, 150 MHz on RP2350 by default).
        float clkdiv = (float)clock_get_hz(clk_sys) / (hz * 1.0f);

        // RP2 PIO divider integer part is 16-bit (plus 8-bit fractional), so clamp.
        if (clkdiv < 1.0f) {
            clkdiv = 1.0f;
        } else if (clkdiv > 65535.996f) {
            clkdiv = 65535.996f;
        }

        // Keep the reported symbol rate aligned with the actual programmed divider.
        symbolrate_hz = (uint32_t)((float)clock_get_hz(clk_sys) / (clkdiv * 1.0f));

        pio_sm_set_clkdiv(pio, sm, clkdiv);
    }

    uint32_t get_symbolrate() const {
        return symbolrate_hz;
    }

    void start() {
        // Reclaim pin for PIO in case it was temporarily switched to SIO for debug.
        pio_gpio_init(pio, pin);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        pio_sm_set_enabled(pio, sm, true);
        running = true;
        refill_fifo();
    }

    void stop() {
        pio_sm_set_enabled(pio, sm, false);
        running = false;
    }

    void release_pin_to_sio() {
        stop();
        gpio_set_function(pin, GPIO_FUNC_SIO);
        gpio_set_dir(pin, GPIO_OUT);
    }
};

#endif // BPSK_MODULATOR_H


