#ifndef BPSK_MODULATOR_H
#define BPSK_MODULATOR_H

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <hardware/dma.h>

#define USE_PIO_MODULATION 1
#define FRAME_SIZE 256
#define MSG_BUFFER_SIZE 512
static constexpr uint16_t TX_STREAM_SIZE = FRAME_SIZE * 2;
static constexpr uint8_t CONV_G1 = 0x79; // 1111001b, 171 octal
static constexpr uint8_t CONV_G2 = 0x5B; // 1011011b, 133 octal
// CCSDS legacy 8-bit randomizer table used by SatDump.
static const uint8_t RANDOMIZER8_TABLE[255] = {
    0xff, 0x48, 0x0e, 0xc0, 0x9a, 0x0d, 0x70, 0xbc,
    0x8e, 0x2c, 0x93, 0xad, 0xa7, 0xb7, 0x46, 0xce,
    0x5a, 0x97, 0x7d, 0xcc, 0x32, 0xa2, 0xbf, 0x3e,
    0x0a, 0x10, 0xf1, 0x88, 0x94, 0xcd, 0xea, 0xb1,
    0xfe, 0x90, 0x1d, 0x81, 0x34, 0x1a, 0xe1, 0x79,
    0x1c, 0x59, 0x27, 0x5b, 0x4f, 0x6e, 0x8d, 0x9c,
    0xb5, 0x2e, 0xfb, 0x98, 0x65, 0x45, 0x7e, 0x7c,
    0x14, 0x21, 0xe3, 0x11, 0x29, 0x9b, 0xd5, 0x63,
    0xfd, 0x20, 0x3b, 0x02, 0x68, 0x35, 0xc2, 0xf2,
    0x38, 0xb2, 0x4e, 0xb6, 0x9e, 0xdd, 0x1b, 0x39,
    0x6a, 0x5d, 0xf7, 0x30, 0xca, 0x8a, 0xfc, 0xf8,
    0x28, 0x43, 0xc6, 0x22, 0x53, 0x37, 0xaa, 0xc7,
    0xfa, 0x40, 0x76, 0x04, 0xd0, 0x6b, 0x85, 0xe4,
    0x71, 0x64, 0x9d, 0x6d, 0x3d, 0xba, 0x36, 0x72,
    0xd4, 0xbb, 0xee, 0x61, 0x95, 0x15, 0xf9, 0xf0,
    0x50, 0x87, 0x8c, 0x44, 0xa6, 0x6f, 0x55, 0x8f,
    0xf4, 0x80, 0xec, 0x09, 0xa0, 0xd7, 0x0b, 0xc8,
    0xe2, 0xc9, 0x3a, 0xda, 0x7b, 0x74, 0x6c, 0xe5,
    0xa9, 0x77, 0xdc, 0xc3, 0x2a, 0x2b, 0xf3, 0xe0,
    0xa1, 0x0f, 0x18, 0x89, 0x4c, 0xde, 0xab, 0x1f,
    0xe9, 0x01, 0xd8, 0x13, 0x41, 0xae, 0x17, 0x91,
    0xc5, 0x92, 0x75, 0xb4, 0xf6, 0xe8, 0xd9, 0xcb,
    0x52, 0xef, 0xb9, 0x86, 0x54, 0x57, 0xe7, 0xc1,
    0x42, 0x1e, 0x31, 0x12, 0x99, 0xbd, 0x56, 0x3f,
    0xd2, 0x03, 0xb0, 0x26, 0x83, 0x5c, 0x2f, 0x23,
    0x8b, 0x24, 0xeb, 0x69, 0xed, 0xd1, 0xb3, 0x96,
    0xa5, 0xdf, 0x73, 0x0c, 0xa8, 0xaf, 0xcf, 0x82,
    0x84, 0x3c, 0x62, 0x25, 0x33, 0x7a, 0xac, 0x7f,
    0xa4, 0x07, 0x60, 0x4d, 0x06, 0xb8, 0x5e, 0x47,
    0x16, 0x49, 0xd6, 0xd3, 0xdb, 0xa3, 0x67, 0x2d,
    0x4b, 0xbe, 0xe6, 0x19, 0x51, 0x5f, 0x9f, 0x05,
    0x08, 0x78, 0xc4, 0x4a, 0x66, 0xf5, 0x58,
};

const int CADU_ASM_SIZE = 4;

constexpr uint8_t bpsk_parity8(uint8_t value) {
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

struct BPSKConvLutGen {
    uint8_t lut[128];
    constexpr BPSKConvLutGen() : lut{} {
        for (int s = 0; s < 128; ++s) {
            lut[s] = (bpsk_parity8((uint8_t)(s & CONV_G1)) << 1) | bpsk_parity8((uint8_t)(s & CONV_G2));
        }
    }
};
static constexpr BPSKConvLutGen CCSDS_CONV_LUT;

static const uint16_t bpsk_modulator_program[] = {
    0x6001,
};

static const pio_program_t bpsk_modulator_program_default = {
    .instructions = bpsk_modulator_program,
    .length = 1,
    .origin = -1,
};

class BPSKModulator;
extern BPSKModulator* g_modulator_instance;

#if USE_PIO_MODULATION
extern void bpsk_dma_isr();
#endif

#if !USE_PIO_MODULATION
extern bool bpsk_timer_callback(repeating_timer_t *rt);
#endif

class BPSKModulator
{
private:
#if USE_PIO_MODULATION
    PIO pio;
    uint sm;
    uint pio_offset;
    int dma_chan_0;
    int dma_chan_1;
    dma_channel_config c0;
    dma_channel_config c1;
    uint32_t dma_buf[2][128];
    uint32_t tx_stream_len_words;
#else
    repeating_timer_t timer;
    bool timer_active;
    uint8_t bit_index;
    uint8_t tx_stream[TX_STREAM_SIZE];
    uint16_t tx_stream_len;
    uint16_t tx_index;
#endif
    
    uint pin;
    uint32_t symbolrate_hz;
    uint16_t msg_len;
    bool msg_pending;
    bool running;
    volatile bool restore_filler_after_message;
    bool invert_output;
    bool convolution_enabled;
    bool randomizer_enabled;
    uint8_t pending_frame[FRAME_SIZE];
    volatile bool pending_frame_valid;
    uint8_t conv_shift_reg;

public:
    uint8_t frame[FRAME_SIZE];
    uint8_t msg_buffer[MSG_BUFFER_SIZE];
    
    BPSKModulator(uint pin, uint32_t rate = 1200) : pin(pin), symbolrate_hz(rate), 
          msg_len(0), msg_pending(false), running(false), restore_filler_after_message(false), invert_output(false),
                    convolution_enabled(false), randomizer_enabled(true), pending_frame_valid(false), conv_shift_reg(0) {
        g_modulator_instance = this;
#if !USE_PIO_MODULATION
        tx_stream_len = 0;
        tx_index = 0;
        bit_index = 0;
        timer_active = false;
#else
        tx_stream_len_words = 0;
#endif
        init_frame();
#if USE_PIO_MODULATION
        init_pio(pin);
#else
        init_gpio(pin);
#endif
    }

    ~BPSKModulator() {
        stop();
#if !USE_PIO_MODULATION
        if (timer_active) cancel_repeating_timer(&timer);
        g_modulator_instance = nullptr;
#endif
    }

#if !USE_PIO_MODULATION
    void build_tx_stream_from_frame(const uint8_t *src_frame, uint8_t *out_stream, uint16_t *out_len) {
        if (!convolution_enabled) {
            memcpy(out_stream, src_frame, FRAME_SIZE);
            *out_len = FRAME_SIZE;
            return;
        }

        uint16_t out_index = 0;
        for (uint16_t i = 0; i < FRAME_SIZE; i++) {
            uint8_t current_byte = src_frame[i];
            uint16_t out_word = 0;

            for (int bit = 7; bit >= 0; bit--) {
                uint8_t input_bit = (current_byte >> bit) & 0x01;
                // CCSDS convention: newest input bit is at register bit 6.
                conv_shift_reg = (uint8_t)(((conv_shift_reg >> 1) | (input_bit << 6)) & 0x7F);
                out_word = (out_word << 2) | CCSDS_CONV_LUT.lut[conv_shift_reg];
            }

            out_stream[out_index++] = (out_word >> 8) & 0xFF;
            out_stream[out_index++] = out_word & 0xFF;
        }

        *out_len = out_index;
    }

    void build_tx_stream() {
        build_tx_stream_from_frame(frame, tx_stream, &tx_stream_len);
    }

    void prepare_tx_stream() {
        build_tx_stream();
        tx_index = 0;
        bit_index = 0;
    }
#else
    void prepare_tx_stream() {
        // DMA prepares chunks synchronously via ISR. Empty wrapper preserves compatibility.
    }
#endif

    void randomize_frame_data(uint16_t start_index) {
        for (uint16_t i = start_index; i < FRAME_SIZE; i++) {
            frame[i] ^= RANDOMIZER8_TABLE[(i - start_index) % 255];
        }
    }

    void randomize_buffer_data(uint8_t *buffer, uint16_t start_index) {
        for (uint16_t i = start_index; i < FRAME_SIZE; i++) {
            buffer[i] ^= RANDOMIZER8_TABLE[(i - start_index) % 255];
        }
    }

    void init_frame_pattern_55() {
        memset(frame, 0x55, FRAME_SIZE);
        frame[0] = 0x1A;
        frame[1] = 0xCF;
        frame[2] = 0xFC;
        frame[3] = 0x1D;
        if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
        prepare_tx_stream();
    }

    void init_frame_with_message(const uint8_t *msg, uint16_t len) {
        frame[0] = 0x1A;  frame[1] = 0xCF;  frame[2] = 0xFC;  frame[3] = 0x1D;
        uint16_t pos = CADU_ASM_SIZE;
        for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++)
            frame[pos] = msg[i];
        for (uint16_t i = pos; i < FRAME_SIZE; i++)
            frame[i] = 0x55;
        if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
        prepare_tx_stream();
    }

    void init_frame_binary(const uint8_t *data, uint16_t len, bool prepacked_asm) {
        memset(frame, 0x55, FRAME_SIZE);
        if (prepacked_asm) {
            for (uint16_t i = 0; i < len && i < FRAME_SIZE; i++)
                frame[i] = data[i];
        } else {
            frame[0] = 0x1A;  frame[1] = 0xCF;  frame[2] = 0xFC;  frame[3] = 0x1D;
            uint16_t pos = CADU_ASM_SIZE;
            for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++)
                frame[pos] = data[i];
        }

        if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
        prepare_tx_stream();
    }

    bool queue_binary_frame(const uint8_t *data, uint16_t len, bool prepacked_asm) {
        if (pending_frame_valid) return false;

        memset(pending_frame, 0x55, FRAME_SIZE);

        if (prepacked_asm) {
            for (uint16_t i = 0; i < len && i < FRAME_SIZE; i++) {
                pending_frame[i] = data[i];
            }
        } else {
            pending_frame[0] = 0x1A;
            pending_frame[1] = 0xCF;
            pending_frame[2] = 0xFC;
            pending_frame[3] = 0x1D;
            uint16_t pos = CADU_ASM_SIZE;
            for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++) {
                pending_frame[pos] = data[i];
            }
        }

        if (randomizer_enabled && !prepacked_asm) {
            randomize_buffer_data(pending_frame, CADU_ASM_SIZE);
        }
        restore_filler_after_message = true;
        pending_frame_valid = true;
        // Uploaded payload frame should be sent once, then fall back to filler.
        return true;
    }

    bool can_queue_binary_frame() const { return !pending_frame_valid; }

    void init_frame_message_only(const uint8_t *msg, uint16_t len) {
        uint16_t pos = 0;
        for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++)
            frame[pos] = msg[i];
        for (uint16_t i = pos; i < FRAME_SIZE; i++)
            frame[i] = 0x55;
        if (randomizer_enabled) randomize_frame_data(0);
        prepare_tx_stream();
    }

    void queue_message(const uint8_t *msg, uint16_t len) {
        if (len > MSG_BUFFER_SIZE) len = MSG_BUFFER_SIZE;
        memcpy(msg_buffer, msg, len);
        msg_len = len;
        msg_pending = true;
    }

    bool has_pending_message() const { return msg_pending; }

    void clear_frame() {
        memset(frame, 0, FRAME_SIZE);
        conv_shift_reg = 0;
        prepare_tx_stream();
        restart_transmission();
    }

    void set_convolutional_encoding(bool enabled) {
        convolution_enabled = enabled;
        conv_shift_reg = 0;
        restore_filler_after_message = true;
    }

    bool get_convolutional_encoding() const { return convolution_enabled; }

    void set_randomizer_enabled(bool enabled) {
        randomizer_enabled = enabled;
        restore_filler_after_message = true;
    }

    bool get_randomizer_enabled() const { return randomizer_enabled; }

    void restart_transmission() {
#if USE_PIO_MODULATION
        if (running) {
            stop();
            start();
        }
#endif
    }

    void inject_message(bool filler_enabled) {
        if (!msg_pending || msg_len == 0) return;
        if (pending_frame_valid) return;

        if (filler_enabled) {
            pending_frame[0] = 0x1A;  pending_frame[1] = 0xCF;  pending_frame[2] = 0xFC;  pending_frame[3] = 0x1D;
            uint16_t pos = CADU_ASM_SIZE;
            for (uint16_t i = 0; i < msg_len && pos < FRAME_SIZE; i++, pos++) pending_frame[pos] = msg_buffer[i];
            for (uint16_t i = pos; i < FRAME_SIZE; i++) pending_frame[i] = 0x55;
            if (randomizer_enabled) randomize_buffer_data(pending_frame, CADU_ASM_SIZE);
            restore_filler_after_message = true;
        } else {
            uint16_t pos = 0;
            for (uint16_t i = 0; i < msg_len && pos < FRAME_SIZE; i++, pos++) pending_frame[pos] = msg_buffer[i];
            for (uint16_t i = pos; i < FRAME_SIZE; i++) pending_frame[i] = 0x55;
            if (randomizer_enabled) randomize_buffer_data(pending_frame, 0);
            restore_filler_after_message = false;
        }
        pending_frame_valid = true;
        msg_pending = false;
    }

    void set_symbolrate(uint32_t hz) {
        if (hz < 1 || hz > 10000000) return;
        symbolrate_hz = hz;
#if USE_PIO_MODULATION
        float clkdiv = (float)clock_get_hz(clk_sys) / (float)hz;
        if (clkdiv < 1.0f) clkdiv = 1.0f;
        else if (clkdiv > 65535.996f) clkdiv = 65535.996f;
        pio_sm_set_clkdiv(pio, sm, clkdiv);
#else
        if (running && timer_active) {
            cancel_repeating_timer(&timer);
            timer_active = false;
            start_timer();
        }
#endif
    }

    uint32_t get_symbolrate() const { return symbolrate_hz; }
    void set_invert_output(bool invert) { invert_output = invert; }
    bool get_invert_output() const { return invert_output; }

    void init_frame() {
        init_frame_pattern_55();
    }

#if USE_PIO_MODULATION
    void fill_dma_buffer(uint32_t *out_buf) {
        if (pending_frame_valid) {
            memcpy(frame, pending_frame, FRAME_SIZE);
            pending_frame_valid = false;
        } else if (restore_filler_after_message) {
            memset(frame, 0x55, FRAME_SIZE);
            frame[0] = 0x1A;
            frame[1] = 0xCF;
            frame[2] = 0xFC;
            frame[3] = 0x1D;
            if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
            restore_filler_after_message = false;
        }

        uint32_t words_written = 0;
        if (!convolution_enabled) {
            for (uint16_t i = 0; i < FRAME_SIZE; i += 4) {
                uint32_t word = ((uint32_t)frame[i] << 24) | ((uint32_t)frame[i+1] << 16) | ((uint32_t)frame[i+2] << 8) | (uint32_t)frame[i+3];
                if (invert_output) word ^= 0xFFFFFFFF;
                out_buf[words_written++] = word;
            }
        } else {
            for (uint16_t i = 0; i < FRAME_SIZE; i += 2) {
                uint32_t word = 0;
                for (int b = 0; b < 2; b++) {
                    uint8_t current_byte = frame[i + b];
                    uint16_t out_word = 0;
                    for (int bit = 7; bit >= 0; bit--) {
                        uint8_t input_bit = (current_byte >> bit) & 0x01;
                        conv_shift_reg = (uint8_t)(((conv_shift_reg >> 1) | (input_bit << 6)) & 0x7F);
                        out_word = (out_word << 2) | CCSDS_CONV_LUT.lut[conv_shift_reg];
                    }
                    word = (word << 16) | out_word;
                }
                if (invert_output) word ^= 0xFFFFFFFF;
                out_buf[words_written++] = word;
            }
        }
        tx_stream_len_words = words_written;
    }

    void handle_dma_irq() {
        if (dma_channel_get_irq0_status(dma_chan_0)) {
            dma_channel_acknowledge_irq0(dma_chan_0);
            fill_dma_buffer(dma_buf[0]);
            dma_channel_set_trans_count(dma_chan_0, tx_stream_len_words, false);
            dma_channel_set_read_addr(dma_chan_0, dma_buf[0], false);
        }
        if (dma_channel_get_irq0_status(dma_chan_1)) {
            dma_channel_acknowledge_irq0(dma_chan_1);
            fill_dma_buffer(dma_buf[1]);
            dma_channel_set_trans_count(dma_chan_1, tx_stream_len_words, false);
            dma_channel_set_read_addr(dma_chan_1, dma_buf[1], false);
        }
    }

    void init_pio(uint pin) {
        pio = pio0;
        pio_offset = pio_add_program(pio, &bpsk_modulator_program_default);
        sm = pio_claim_unused_sm(pio, true);
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_out_pins(&c, pin, 1);
        sm_config_set_wrap(&c, pio_offset, pio_offset);
        sm_config_set_out_shift(&c, false, true, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        pio_gpio_init(pio, pin);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_init(pio, sm, pio_offset, &c);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        set_symbolrate(symbolrate_hz);

        dma_chan_0 = dma_claim_unused_channel(true);
        dma_chan_1 = dma_claim_unused_channel(true);

        c0 = dma_channel_get_default_config(dma_chan_0);
        channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
        channel_config_set_read_increment(&c0, true);
        channel_config_set_write_increment(&c0, false);
        channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));
        channel_config_set_chain_to(&c0, dma_chan_1);
        dma_channel_set_config(dma_chan_0, &c0, false);

        c1 = dma_channel_get_default_config(dma_chan_1);
        channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
        channel_config_set_read_increment(&c1, true);
        channel_config_set_write_increment(&c1, false);
        channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));
        channel_config_set_chain_to(&c1, dma_chan_0);
        dma_channel_set_config(dma_chan_1, &c1, false);

        dma_channel_set_irq0_enabled(dma_chan_0, true);
        dma_channel_set_irq0_enabled(dma_chan_1, true);

        irq_set_exclusive_handler(DMA_IRQ_0, bpsk_dma_isr);
        irq_set_enabled(DMA_IRQ_0, true);
    }

    bool tx_fifo_empty() const { return pio_sm_is_tx_fifo_empty(pio, sm); }
    bool tx_fifo_full() const { return pio_sm_is_tx_fifo_full(pio, sm); }
    uint get_sm() const { return sm; }

    void start() {
        if (running) return;
        pio_gpio_init(pio, pin);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);

        fill_dma_buffer(dma_buf[0]);
        fill_dma_buffer(dma_buf[1]);

        dma_channel_configure(dma_chan_0, &c0, &pio->txf[sm], dma_buf[0], tx_stream_len_words, false);
        dma_channel_configure(dma_chan_1, &c1, &pio->txf[sm], dma_buf[1], tx_stream_len_words, false);

        pio_sm_set_enabled(pio, sm, true);
        dma_channel_start(dma_chan_0);
        running = true;
    }

    void stop() {
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(dma_chan_0);
        dma_channel_abort(dma_chan_1);
        running = false;
    }

#else
    void init_gpio(uint pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    void output_next_bit() {
        if (!running) return;

        if (tx_index >= tx_stream_len) {
            tx_index = 0;
            if (pending_frame_valid) {
                memcpy(frame, pending_frame, FRAME_SIZE);
                pending_frame_valid = false;
            } else if (restore_filler_after_message) {
                memset(frame, 0x55, FRAME_SIZE);
                frame[0] = 0x1A;
                frame[1] = 0xCF;
                frame[2] = 0xFC;
                frame[3] = 0x1D;
                if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
                restore_filler_after_message = false;
            }
            build_tx_stream();
        }

        uint8_t current_byte = tx_stream[tx_index];
        uint8_t bit = (current_byte >> (7 - bit_index)) & 0x01;
        if (invert_output) bit ^= 0x01;
        gpio_put(pin, bit);
        bit_index++;
        if (bit_index >= 8) {
            bit_index = 0;
            tx_index++;
        }
    }

    void start_timer() {
        if (timer_active) return;
        int64_t period_us = (int64_t)(1000000.0 / symbolrate_hz);
        if (period_us < 1) period_us = 1;
        if (add_repeating_timer_us(period_us, bpsk_timer_callback, nullptr, &timer))
            timer_active = true;
    }

    void start() {
        if (running) return;
        running = true;
        tx_index = 0;
        bit_index = 0;
        start_timer();
    }

    void stop() {
        running = false;
        if (timer_active) {
            cancel_repeating_timer(&timer);
            timer_active = false;
        }
        gpio_put(pin, 0);
    }
#endif
};

BPSKModulator* g_modulator_instance = nullptr;

#if USE_PIO_MODULATION
void bpsk_dma_isr() {
    if (g_modulator_instance != nullptr) {
        g_modulator_instance->handle_dma_irq();
    }
}
#endif

#if !USE_PIO_MODULATION
bool bpsk_timer_callback(repeating_timer_t *rt) {
    if (g_modulator_instance != nullptr) {
        g_modulator_instance->output_next_bit();
    }
    return true;
}
#endif

#endif