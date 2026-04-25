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
#include "pico/util/queue.h"

#include "rs.h"

#define MAX_FRAME_SIZE 4084
#define MAX_MSG_BUFFER_SIZE 4084
#define MAX_FEC_INPUT_SIZE 3568

static constexpr uint8_t CONV_G1 = 0x79; // 1111001b, 171 octal
static constexpr uint8_t CONV_G2 = 0x5B; // 1011011b, 133 octal
// CCSDS legacy 8-bit randomizer table used by SatDump.
static constexpr uint8_t RANDOMIZER8_TABLE[255] = {
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

struct BPSKRandomizerFastLut {
    uint8_t seq[MAX_FRAME_SIZE];
    constexpr BPSKRandomizerFastLut() : seq{} {
        for (int i = 0; i < MAX_FRAME_SIZE; i++) {
            seq[i] = RANDOMIZER8_TABLE[i % 255];
        }
    }
};
static BPSKRandomizerFastLut CCSDS_RANDOMIZER_FAST_LUT;

constexpr uint8_t bpsk_parity8(uint8_t value) {
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

struct BPSKConvFastLut {
    uint16_t byte_out[256];
    uint16_t state_out[64];
    uint8_t next_state[256];

    constexpr BPSKConvFastLut() : byte_out{}, state_out{}, next_state{} {
        for (int b = 0; b < 256; b++) {
            uint16_t out_word = 0;
            uint8_t c = 0;
            for (int bit = 7; bit >= 0; bit--) {
                uint8_t input_bit = (b >> bit) & 0x01;
                c = (uint8_t)(((c >> 1) | (input_bit << 6)) & 0x7F);
                out_word = (out_word << 2) | ((bpsk_parity8(c & CONV_G1) << 1) | bpsk_parity8(c & CONV_G2));
            }
            byte_out[b] = out_word;
            next_state[b] = c >> 1; // Top 6 bits perfectly capture the state for the next byte
        }

        for (int s = 0; s < 64; s++) {
            uint16_t out_word = 0;
            uint8_t c = s << 1; 
            for (int bit = 7; bit >= 0; bit--) {
                c = (uint8_t)((c >> 1) & 0x7F);
                out_word = (out_word << 2) | ((bpsk_parity8(c & CONV_G1) << 1) | bpsk_parity8(c & CONV_G2));
            }
            state_out[s] = out_word;
        }
    }
};
static BPSKConvFastLut CCSDS_CONV_FAST_LUT;

static uint8_t PUNC_PERIOD[] = { 1, 2, 3, 5, 7 };
static uint8_t PUNC_C1[] = { 0b1, 0b01, 0b101, 0b10101, 0b1010001 };
static uint8_t PUNC_C2[] = { 0b1, 0b11, 0b011, 0b01011, 0b0101111 };

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

extern void bpsk_dma_isr();

class BPSKModulator
{
private:
    PIO pio;
    uint sm;
    uint pio_offset;
    int dma_chan_0;
    int dma_chan_1;
    dma_channel_config c0;
    dma_channel_config c1;
    
    static constexpr int DMA_BUFFER_TOTAL_WORDS = 4096;
    uint32_t dma_buf[2][DMA_BUFFER_TOTAL_WORDS];
    
    uint pin;
    uint32_t symbolrate_hz;
    uint16_t msg_len;
    bool msg_pending;
    volatile bool running;
    volatile bool restore_filler_after_message;
    bool convolution_enabled;
    bool randomizer_enabled;
    bool reed_solomon_enabled;
    bool dual_basis_enabled;
    uint8_t rs_interleave;
    uint16_t current_frame_size;
    uint16_t current_payload_size;

    alignas(4) uint8_t fec_input_buffer[MAX_FEC_INPUT_SIZE];
    uint16_t fec_input_buffer_len;

    // Deep enough to absorb math delays, small enough to prevent Out-Of-Memory crashes
    static constexpr int PENDING_FRAME_QUEUE_SIZE = 8;
    queue_t pending_frame_queue;

    struct alignas(4) PendingFrame {
        uint8_t data[MAX_FRAME_SIZE];
        uint16_t length;
    };

    struct alignas(4) DmaChunk {
        uint32_t words[2048]; // Max 2048 words (8192 bytes) securely handles Convolutional 1/2 expanding a 4084 byte frame
        uint32_t length; // 32-bit perfectly aligns struct for ARM LDM/STM memory copies
    };

    alignas(4) PendingFrame current_tx_pf;
    alignas(4) PendingFrame staging_pf;
    alignas(4) DmaChunk dma_generation_chunk;
    alignas(4) DmaChunk dma_irq_chunk;
    alignas(4) uint8_t rs_cadu_buffer[MAX_FRAME_SIZE];

    uint8_t conv_shift_reg;
    alignas(4) uint32_t tx_accum_data;
    uint8_t tx_accum_bits;
    uint8_t punc_phase;
    uint8_t conv_rate;
    alignas(4) uint8_t filler_frame[MAX_FRAME_SIZE];

    static constexpr int DMA_CHUNK_QUEUE_SIZE = 8;
    queue_t dma_chunk_queue;

    void flush_dma_chunks() {
        static DmaChunk dummy;
        while (queue_try_remove(&dma_chunk_queue, &dummy));
    }

    void flush_pending_frames() {
        static PendingFrame dummy;
        while (queue_try_remove(&pending_frame_queue, &dummy));
    }

    uint8_t get_dma_chunks_count() const {
        return queue_get_level(const_cast<queue_t*>(&dma_chunk_queue));
    }

    void clear_baseband_state() {
        conv_shift_reg = 0;
        tx_accum_data = 0;
        tx_accum_bits = 0;
        punc_phase = 0;
        fec_input_buffer_len = 0;
        flush_pending_frames();
        flush_dma_chunks();
    }

public:
    alignas(4) uint8_t frame[MAX_FRAME_SIZE];
    alignas(4) uint8_t msg_buffer[MAX_MSG_BUFFER_SIZE];
    volatile bool config_lock;
    
    void lock_config() {
        config_lock = true;
        __dmb(); // Force memory controller to instantly broadcast lock state to Core 1
        delay(2); // Wait 2ms to ensure Core 1 cleanly exits its baseband processing loop
    }

    void unlock_config() {
        __dmb();
        config_lock = false;
        __dmb();
    }
    
    BPSKModulator(uint pin, uint32_t rate = 1200) : pin(pin), symbolrate_hz(rate), 
          msg_len(0), msg_pending(false), running(false), restore_filler_after_message(false),
                    convolution_enabled(true), randomizer_enabled(true), reed_solomon_enabled(true), dual_basis_enabled(false),
                    rs_interleave(4), current_frame_size(CADU_ASM_SIZE + (255 * 4)), current_payload_size(223 * 4),
                    fec_input_buffer_len(0),
                    conv_shift_reg(0), tx_accum_data(0), tx_accum_bits(0), punc_phase(0), conv_rate(0),
                    config_lock(false) {
        g_modulator_instance = this;
        queue_init(&pending_frame_queue, sizeof(PendingFrame), PENDING_FRAME_QUEUE_SIZE);
        queue_init(&dma_chunk_queue, sizeof(DmaChunk), DMA_CHUNK_QUEUE_SIZE);
        build_filler_frame();
        init_frame();
        init_pio(pin);
    }

    ~BPSKModulator() {
        lock_config();
        stop();
        unlock_config();
        queue_free(&pending_frame_queue);
        queue_free(&dma_chunk_queue);
    }

    void prepare_tx_stream() {
        // DMA prepares chunks synchronously via ISR. Empty wrapper preserves compatibility.
    }

    void randomize_frame_data(uint16_t start_index) {
        for (uint16_t i = start_index; i < current_frame_size; i++) {
            frame[i] ^= CCSDS_RANDOMIZER_FAST_LUT.seq[i - start_index];
        }
    }

    void randomize_buffer_data(uint8_t *buffer, uint16_t start_index, uint16_t len) {
        for (uint16_t i = start_index; i < len; i++) {
            buffer[i] ^= CCSDS_RANDOMIZER_FAST_LUT.seq[i - start_index];
        }
    }

    void build_filler_frame() {
        if (reed_solomon_enabled) {
            static const int RS_K = 223;
            static const int RS_N = 255;
            int RS_ENCODED_SIZE = rs_interleave * RS_N;

            filler_frame[0] = 0x1A;
            filler_frame[1] = 0xCF;
            filler_frame[2] = 0xFC;
            filler_frame[3] = 0x1D;

            memset(filler_frame + CADU_ASM_SIZE, 0x55, RS_K * rs_interleave);

            for (int i = 0; i < rs_interleave; i++) {
                uint8_t parity[RS_N - RS_K];
                rs_encode(filler_frame + CADU_ASM_SIZE, parity, i, rs_interleave);

                for (int c = 0; c < (RS_N - RS_K); c++) {
                    filler_frame[CADU_ASM_SIZE + RS_K * rs_interleave + c * rs_interleave + i] = parity[c];
                }
            }

            if (dual_basis_enabled) {
                rs_apply_dual_basis(filler_frame + CADU_ASM_SIZE, RS_ENCODED_SIZE);
            }

            if (randomizer_enabled) {
                for (int i = 0; i < RS_ENCODED_SIZE; i++) {
                    filler_frame[CADU_ASM_SIZE + i] ^= CCSDS_RANDOMIZER_FAST_LUT.seq[i];
                }
            }
        } else {
            memset(filler_frame, 0x55, current_frame_size);
            filler_frame[0] = 0x1A;
            filler_frame[1] = 0xCF;
            filler_frame[2] = 0xFC;
            filler_frame[3] = 0x1D;
            if (randomizer_enabled) {
                randomize_buffer_data(filler_frame, CADU_ASM_SIZE, current_frame_size);
            }
        }
    }

    void init_frame_with_message(const uint8_t *msg, uint16_t len) {
        frame[0] = 0x1A;  frame[1] = 0xCF;  frame[2] = 0xFC;  frame[3] = 0x1D;
        uint16_t pos = CADU_ASM_SIZE;
        for (uint16_t i = 0; i < len && pos < current_frame_size; i++, pos++)
            frame[pos] = msg[i];
        for (uint16_t i = pos; i < current_frame_size; i++)
            frame[i] = 0x55;
        if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
        prepare_tx_stream();
    }

    void init_frame_binary(const uint8_t *data, uint16_t len, bool prepacked_asm) {
        memset(frame, 0x55, current_frame_size);
        if (prepacked_asm) {
            for (uint16_t i = 0; i < len && i < current_frame_size; i++)
                frame[i] = data[i];
        } else {
            frame[0] = 0x1A;  frame[1] = 0xCF;  frame[2] = 0xFC;  frame[3] = 0x1D;
            uint16_t pos = CADU_ASM_SIZE;
            for (uint16_t i = 0; i < len && pos < current_frame_size; i++, pos++)
                frame[pos] = data[i];
        }

        if (randomizer_enabled) randomize_frame_data(CADU_ASM_SIZE);
        prepare_tx_stream();
    }

    uint8_t get_pending_frames_count() const {
        return queue_get_level(const_cast<queue_t*>(&pending_frame_queue));
    }

    bool push_pending_frame(const uint8_t* frame_data, uint16_t len) {
        memcpy(staging_pf.data, frame_data, len);
        staging_pf.length = len;
        return queue_try_add(&pending_frame_queue, &staging_pf);
    }

    bool pop_pending_frame(PendingFrame* pf) {
        return queue_try_remove(&pending_frame_queue, pf);
    }

    void process_rs_buffer() {
        static const int RS_K = 223;
        static const int RS_N = 255;
        int RS_ENCODED_SIZE = rs_interleave * RS_N;
        int CADU_SIZE = CADU_ASM_SIZE + RS_ENCODED_SIZE;

        rs_cadu_buffer[0] = 0x1A;
        rs_cadu_buffer[1] = 0xCF;
        rs_cadu_buffer[2] = 0xFC;
        rs_cadu_buffer[3] = 0x1D;

        memcpy(rs_cadu_buffer + CADU_ASM_SIZE, fec_input_buffer, RS_K * rs_interleave);

        for (int i = 0; i < rs_interleave; i++) {
            uint8_t parity[RS_N - RS_K];
            rs_encode(fec_input_buffer, parity, i, rs_interleave);

            for (int c = 0; c < (RS_N - RS_K); c++) {
                rs_cadu_buffer[CADU_ASM_SIZE + RS_K * rs_interleave + c * rs_interleave + i] = parity[c];
            }
        }

        if (dual_basis_enabled) {
            // Apply CCSDS dual basis conversion to the encoded payload (not ASM)
            rs_apply_dual_basis(rs_cadu_buffer + CADU_ASM_SIZE, RS_ENCODED_SIZE);
        }

        if (randomizer_enabled) {
            for (int i = 0; i < RS_ENCODED_SIZE; i++) {
                rs_cadu_buffer[CADU_ASM_SIZE + i] ^= CCSDS_RANDOMIZER_FAST_LUT.seq[i];
            }
        }

        push_pending_frame(rs_cadu_buffer, CADU_SIZE);
        
        restore_filler_after_message = true;
    }

    bool queue_binary_frame(const uint8_t *data, uint16_t len, bool prepacked_asm) {
        if (!can_queue_binary_frame()) return false;

        if (!reed_solomon_enabled) {
            init_frame_binary(data, len, prepacked_asm);
            return push_pending_frame(frame, current_frame_size);
        }

        uint16_t data_copied = 0;
        while(data_copied < len) {
            uint16_t to_copy = len - data_copied;
            uint16_t space_left = current_payload_size - fec_input_buffer_len;
            if (to_copy > space_left) to_copy = space_left;
            
            memcpy(fec_input_buffer + fec_input_buffer_len, data + data_copied, to_copy);
            fec_input_buffer_len += to_copy;
            data_copied += to_copy;

            if (fec_input_buffer_len == current_payload_size) {
                if (reed_solomon_enabled) process_rs_buffer();
                fec_input_buffer_len = 0;
            }
        }
        return true;
    }

    void clear_fec_buffer() {
        lock_config();
        fec_input_buffer_len = 0;
        unlock_config();
    }

    bool can_queue_binary_frame() const {
        uint8_t count = get_pending_frames_count();
        if (reed_solomon_enabled) {
            return (count <= (PENDING_FRAME_QUEUE_SIZE - 2));
        }
        return (count < PENDING_FRAME_QUEUE_SIZE - 1);
    }

    void init_frame_message_only(const uint8_t *msg, uint16_t len) {
        uint16_t pos = 0;
        for (uint16_t i = 0; i < len && pos < current_frame_size; i++, pos++)
            frame[pos] = msg[i];
        for (uint16_t i = pos; i < current_frame_size; i++)
            frame[i] = 0x55;
        if (randomizer_enabled) randomize_frame_data(0);
        prepare_tx_stream();
    }

    void queue_message(const uint8_t *msg, uint16_t len) {
        if (len > MAX_MSG_BUFFER_SIZE) len = MAX_MSG_BUFFER_SIZE;
        memcpy(msg_buffer, msg, len);
        msg_len = len;
        msg_pending = true;
    }

    bool has_pending_message() const { return msg_pending; }

    void clear_frame() {
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        memset(frame, 0, current_frame_size);
        clear_baseband_state();
        prepare_tx_stream();
        
        if (was_running) start();
        unlock_config();
    }

    void set_convolutional_encoding(bool enabled) {
        if (convolution_enabled == enabled) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        convolution_enabled = enabled;
        clear_baseband_state();
        restore_filler_after_message = true;
        
        if (was_running) start();
        unlock_config();
    }

    bool get_convolutional_encoding() const { return convolution_enabled; }

    void set_conv_rate(uint8_t rate) {
        if (rate > 4) rate = 4;
        if (conv_rate == rate) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        conv_rate = rate;
        clear_baseband_state();
        restore_filler_after_message = true;
        
        if (was_running) start();
        unlock_config();
    }
    
    uint8_t get_conv_rate() const { return conv_rate; }

    void set_reed_solomon_enabled(bool enabled) {
        if (reed_solomon_enabled == enabled) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        reed_solomon_enabled = enabled;
        if (enabled) {
            current_frame_size = CADU_ASM_SIZE + (255 * rs_interleave);
            current_payload_size = 223 * rs_interleave;
        } else {
            dual_basis_enabled = false;
            current_frame_size = 1024;
            current_payload_size = 1020;
        }
        clear_baseband_state();
        build_filler_frame();
        
        if (was_running) start();
        unlock_config();
    }

    bool get_reed_solomon_enabled() const { return reed_solomon_enabled; }

    void set_rs_interleave(uint8_t depth) {
        if (depth != 1 && depth != 2 && depth != 4 && depth != 8 && depth != 16) return;
        if (rs_interleave == depth) return;
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        rs_interleave = depth;
        if (reed_solomon_enabled) {
            current_frame_size = CADU_ASM_SIZE + (255 * depth);
            current_payload_size = 223 * depth;
        }
        
        clear_baseband_state();
        build_filler_frame();
        
        if (was_running) start();
        unlock_config();
    }

    uint8_t get_rs_interleave() const { return rs_interleave; }
    uint16_t get_frame_size() const { return current_frame_size; }
    uint16_t get_payload_size() const { return current_payload_size; }

    void flush_fec_buffer() {
        if (fec_input_buffer_len > 0) {
            memset(fec_input_buffer + fec_input_buffer_len, 0x55, current_payload_size - fec_input_buffer_len);
            if (reed_solomon_enabled) process_rs_buffer();
            fec_input_buffer_len = 0;
        }
    }

    void set_dual_basis_enabled(bool enabled) { 
        if (dual_basis_enabled == enabled) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        dual_basis_enabled = enabled; 
        if (enabled) {
            reed_solomon_enabled = true;
        }
        clear_baseband_state();
        build_filler_frame();
        
        if (was_running) start();
        unlock_config();
    }
    bool get_dual_basis_enabled() const { return dual_basis_enabled; }

    void set_randomizer_enabled(bool enabled) {
        if (randomizer_enabled == enabled) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        randomizer_enabled = enabled;
        restore_filler_after_message = true;
        clear_baseband_state();
        build_filler_frame();
        
        if (was_running) start();
        unlock_config();
    }

    bool get_randomizer_enabled() const { return randomizer_enabled; }

    void restart_transmission() {
        lock_config();
        if (running) {
            stop();
            start();
        }
        unlock_config();
    }

    void inject_message(bool filler_enabled) {
        if (!msg_pending || msg_len == 0) return;
        if (!can_queue_binary_frame()) return;

        if (!reed_solomon_enabled) {
            if (filler_enabled) {
                init_frame_with_message(msg_buffer, msg_len);
            } else {
                init_frame_message_only(msg_buffer, msg_len);
            }
            push_pending_frame(frame, current_frame_size);
        } else {
            // Buffer for FEC encoding
            uint16_t data_copied = 0;
            while(data_copied < msg_len) {
                uint16_t to_copy = msg_len - data_copied;
                uint16_t space_left = current_payload_size - fec_input_buffer_len;
                if (to_copy > space_left) to_copy = space_left;

                memcpy(fec_input_buffer + fec_input_buffer_len, msg_buffer + data_copied, to_copy);
                fec_input_buffer_len += to_copy;
                data_copied += to_copy;

                if (fec_input_buffer_len == current_payload_size) {
                    if (reed_solomon_enabled) process_rs_buffer();
                    fec_input_buffer_len = 0;
                }
            }
            
            // Pad the rest of the FEC block with 0x55 so the text message is actually transmitted
            flush_fec_buffer();
        }

        msg_pending = false;
    }

    void set_symbolrate(uint32_t hz) {
        if (hz < 1 || hz > 10000000) return;
        if (symbolrate_hz == hz) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        symbolrate_hz = hz;
        float clkdiv = (float)clock_get_hz(clk_sys) / (float)hz;
        if (clkdiv < 1.0f) clkdiv = 1.0f;
        else if (clkdiv > 65535.996f) clkdiv = 65535.996f;
        pio_sm_set_clkdiv(pio, sm, clkdiv);
        unlock_config();
    }

    uint32_t get_symbolrate() const { return symbolrate_hz; }

    void reset_baseband_queues() {
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        clear_baseband_state();
        
        if (was_running) start();
        unlock_config();
    }

    void init_frame() {
        memcpy(frame, filler_frame, current_frame_size);
        prepare_tx_stream();
    }

    bool process_baseband_to_dma() {
        if (queue_is_full(&dma_chunk_queue)) return false;

        if (!pop_pending_frame(&current_tx_pf)) {
            // Keep a massive 28-chunk DMA buffer to guarantee the radio never loses Viterbi lock
            if (get_dma_chunks_count() > 28) {
                return false; // Queue is healthy, don't spam filler frames. Yield to let feed_modulator run.
            }
            // Always copy to the local buffer to ensure consistent memory access patterns
            memcpy(current_tx_pf.data, filler_frame, current_frame_size);
            current_tx_pf.length = current_frame_size;
        }

        uint32_t words_written = 0;
        uint16_t len = current_tx_pf.length;
        uint16_t words = len / 4;
        uint8_t* pf_data = (uint8_t*)current_tx_pf.data;

        if (!convolution_enabled) {
            uint32_t* frame_words = (uint32_t*)current_tx_pf.data;
            for (uint16_t i = 0; i < words; i++) {
                dma_generation_chunk.words[words_written++] = __builtin_bswap32(frame_words[i]);
            }
            // Safely pack any leftover bytes into the final 32-bit word
            if (len % 4 != 0) {
                uint32_t word = 0;
                int shift = 24;
                for (uint16_t i = words * 4; i < len; i++) {
                    word |= (uint32_t)pf_data[i] << shift;
                    shift -= 8;
                }
                dma_generation_chunk.words[words_written++] = word;
            }
        } else {
            if (conv_rate == 0 && tx_accum_bits == 0) {
                uint8_t shift_reg = conv_shift_reg; // Cache state locally to prevent RAM ping-pong
                uint32_t* frame_ptr32 = (uint32_t*)current_tx_pf.data;
                
                for (uint16_t i = 0; i < words; i++) {
                    uint32_t quad_byte = frame_ptr32[i]; // Read 4 bytes at once
                    
                    uint8_t cb0 = quad_byte & 0xFF;
                    uint16_t out0 = CCSDS_CONV_FAST_LUT.byte_out[cb0] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                    shift_reg = CCSDS_CONV_FAST_LUT.next_state[cb0];

                    uint8_t cb1 = (quad_byte >> 8) & 0xFF;
                    uint16_t out1 = CCSDS_CONV_FAST_LUT.byte_out[cb1] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                    shift_reg = CCSDS_CONV_FAST_LUT.next_state[cb1];
                    dma_generation_chunk.words[words_written++] = ((uint32_t)out0 << 16) | out1;

                    uint8_t cb2 = (quad_byte >> 16) & 0xFF;
                    uint16_t out2 = CCSDS_CONV_FAST_LUT.byte_out[cb2] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                    shift_reg = CCSDS_CONV_FAST_LUT.next_state[cb2];

                    uint8_t cb3 = (quad_byte >> 24) & 0xFF;
                    uint16_t out3 = CCSDS_CONV_FAST_LUT.byte_out[cb3] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                    shift_reg = CCSDS_CONV_FAST_LUT.next_state[cb3];
                    dma_generation_chunk.words[words_written++] = ((uint32_t)out2 << 16) | out3;
                }

                // Automatically handle 1-3 byte remainders (e.g., resulting from I=1 interleaved frames)
                for (uint16_t i = words * 4; i < len; i++) {
                    uint8_t cb = pf_data[i];
                    uint16_t out = CCSDS_CONV_FAST_LUT.byte_out[cb] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                    shift_reg = CCSDS_CONV_FAST_LUT.next_state[cb];
                    
                    tx_accum_data = (tx_accum_data << 16) | out;
                    tx_accum_bits += 16;
                    if (tx_accum_bits == 32) {
                        dma_generation_chunk.words[words_written++] = tx_accum_data;
                        tx_accum_bits = 0;
                    }
                }
                conv_shift_reg = shift_reg;
            } else {
                uint32_t accum_data = tx_accum_data;
                uint8_t accum_bits = tx_accum_bits;
                uint8_t p_phase = punc_phase;
                uint8_t p_period = PUNC_PERIOD[conv_rate];
                uint8_t c1_mask = PUNC_C1[conv_rate];
                uint8_t c2_mask = PUNC_C2[conv_rate];
                uint8_t shift_reg = conv_shift_reg; // Cache state locally

                uint32_t* frame_ptr32 = (uint32_t*)current_tx_pf.data;

                for (uint16_t i = 0; i < words; i++) {
                    uint32_t quad_byte = frame_ptr32[i]; // Read 4 bytes at once
                    
                    for (int q = 0; q < 4; q++) {
                        uint8_t current_byte = (quad_byte >> (q * 8)) & 0xFF;
                        uint16_t out_word = CCSDS_CONV_FAST_LUT.byte_out[current_byte] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                        shift_reg = CCSDS_CONV_FAST_LUT.next_state[current_byte];

                        for (int b = 14; b >= 0; b -= 2) { // Unroll arithmetic
                            uint8_t c1 = (out_word >> (b + 1)) & 1;
                            uint8_t c2 = (out_word >> b) & 1;

                            if ((c1_mask >> p_phase) & 1) {
                                accum_data = (accum_data << 1) | c1;
                                accum_bits++;
                                if (accum_bits == 32) {
                                    dma_generation_chunk.words[words_written++] = accum_data;
                                    accum_bits = 0;
                                }
                            }
                            if ((c2_mask >> p_phase) & 1) {
                                accum_data = (accum_data << 1) | c2;
                                accum_bits++;
                                if (accum_bits == 32) {
                                    dma_generation_chunk.words[words_written++] = accum_data;
                                    accum_bits = 0;
                                }
                            }
                            p_phase++;
                            if (p_phase == p_period) p_phase = 0;
                        }
                    }
                }

                // Remainder logic for generalized puncturing
                for (uint16_t i = words * 4; i < len; i++) {
                    uint8_t current_byte = pf_data[i];
                    uint16_t out_word = CCSDS_CONV_FAST_LUT.byte_out[current_byte] ^ CCSDS_CONV_FAST_LUT.state_out[shift_reg];
                    shift_reg = CCSDS_CONV_FAST_LUT.next_state[current_byte];

                    for (int b = 14; b >= 0; b -= 2) {
                        uint8_t c1 = (out_word >> (b + 1)) & 1;
                        uint8_t c2 = (out_word >> b) & 1;

                        if ((c1_mask >> p_phase) & 1) {
                            accum_data = (accum_data << 1) | c1;
                            accum_bits++;
                            if (accum_bits == 32) {
                                dma_generation_chunk.words[words_written++] = accum_data;
                                accum_bits = 0;
                            }
                        }
                        if ((c2_mask >> p_phase) & 1) {
                            accum_data = (accum_data << 1) | c2;
                            accum_bits++;
                            if (accum_bits == 32) {
                                dma_generation_chunk.words[words_written++] = accum_data;
                                accum_bits = 0;
                            }
                        }
                        p_phase++;
                        if (p_phase == p_period) p_phase = 0;
                    }
                }
                tx_accum_data = accum_data;
                tx_accum_bits = accum_bits;
                punc_phase = p_phase;
                conv_shift_reg = shift_reg;
            }
        }
        dma_generation_chunk.length = words_written;
        queue_try_add(&dma_chunk_queue, &dma_generation_chunk);
        return true;
    }

    uint32_t fill_dma_buffer(uint32_t *out_buf) {
        uint32_t total_words = 0;
        while (total_words < DMA_BUFFER_TOTAL_WORDS) {
            if (queue_try_peek(&dma_chunk_queue, &dma_irq_chunk)) {
                if (total_words + dma_irq_chunk.length > DMA_BUFFER_TOTAL_WORDS) {
                    break; // Doesn't fit cleanly in this DMA buffer boundary, wait for next IRQ
                }
                queue_try_remove(&dma_chunk_queue, &dma_irq_chunk);
                memcpy(out_buf + total_words, dma_irq_chunk.words, dma_irq_chunk.length * 4);
                total_words += dma_irq_chunk.length;
            } else {
                break; // Queue is empty, stop batching
            }
        }
        
        if (total_words == 0) {
            // Emergency underflow! Just send idle pattern
            uint32_t dummy = 0x55555555;
            for (int i = 0; i < 256; i++) out_buf[i] = dummy;
            return 256;
        }
        
        return total_words;
    }

    void handle_dma_irq() {
        if (!running) {
            // Safely sink phantom interrupts from aborted transfers to prevent memory corruption
            if (dma_channel_get_irq0_status(dma_chan_0)) dma_channel_acknowledge_irq0(dma_chan_0);
            if (dma_channel_get_irq0_status(dma_chan_1)) dma_channel_acknowledge_irq0(dma_chan_1);
            return;
        }
        
        if (dma_channel_get_irq0_status(dma_chan_0)) {
            dma_channel_acknowledge_irq0(dma_chan_0);
            dma_channel_start(dma_chan_1); // Instantly hot-swap to the pre-filled channel to maintain RF carrier!
            
            uint32_t len = fill_dma_buffer(dma_buf[0]);
            dma_channel_set_trans_count(dma_chan_0, len, false);
            dma_channel_set_read_addr(dma_chan_0, dma_buf[0], false);
        } 
        else if (dma_channel_get_irq0_status(dma_chan_1)) {
            dma_channel_acknowledge_irq0(dma_chan_1);
            dma_channel_start(dma_chan_0); // Instantly hot-swap to the pre-filled channel to maintain RF carrier!
            
            uint32_t len = fill_dma_buffer(dma_buf[1]);
            dma_channel_set_trans_count(dma_chan_1, len, false);
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
        dma_channel_set_config(dma_chan_0, &c0, false);

        c1 = dma_channel_get_default_config(dma_chan_1);
        channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
        channel_config_set_read_increment(&c1, true);
        channel_config_set_write_increment(&c1, false);
        channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));
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

        // Pre-fill DMA queue with 2 chunks directly on Core 0 to prevent cross-core deadlocks
        while (get_dma_chunks_count() < 2) {
            process_baseband_to_dma();
        }

        uint32_t len0 = fill_dma_buffer(dma_buf[0]);
        uint32_t len1 = fill_dma_buffer(dma_buf[1]);

        // c0 and c1 natively contain the correct chain_to configurations from init_pio
        dma_channel_configure(dma_chan_0, &c0, &pio->txf[sm], dma_buf[0], len0, false);
        dma_channel_configure(dma_chan_1, &c1, &pio->txf[sm], dma_buf[1], len1, false);

        dma_channel_set_irq0_enabled(dma_chan_0, true);
        dma_channel_set_irq0_enabled(dma_chan_1, true);

        running = true; // MUST be true before starting DMA so the ISR doesn't reject the very first interrupt
        dma_channel_start(dma_chan_0);
        pio_sm_set_enabled(pio, sm, true);
    }

    void stop() {
        running = false; // Instantly prevent the ISR from processing new chunks
        
        // Disable IRQs so the ISR doesn't re-arm the channels during abort
        dma_channel_set_irq0_enabled(dma_chan_0, false);
        dma_channel_set_irq0_enabled(dma_chan_1, false);

        dma_channel_abort(dma_chan_0);
        dma_channel_abort(dma_chan_1);
        pio_sm_set_enabled(pio, sm, false);
        // Clear pending IRQs to prevent phantom Restarts mid-flush
        dma_channel_acknowledge_irq0(dma_chan_0);
        dma_channel_acknowledge_irq0(dma_chan_1);
    }
};

BPSKModulator* g_modulator_instance = nullptr;

void bpsk_dma_isr() {
    if (g_modulator_instance != nullptr) {
        g_modulator_instance->handle_dma_irq();
    }
}

#endif