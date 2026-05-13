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
#define FRAME_SIZE 1024
#define MSG_BUFFER_SIZE 512
static constexpr uint16_t TX_STREAM_SIZE = FRAME_SIZE * 2;
static constexpr uint8_t CONV_G1 = 0x79; // 1111001b, 171 octal
static constexpr uint8_t CONV_G2 = 0x5B; // 1011011b, 133 octal

// CCSDS AOS Level 0 Transfer Frame (VCDU) constants - CCSDS 732.0-B-5
static constexpr uint16_t ASM_SIZE = 4;  // 4-byte Attached Synchronization Marker (not RS encoded)
static constexpr uint16_t VCDU_HEADER_SIZE = 6;  // 6-byte VCDU header (inside RS block)
static constexpr uint16_t MPDU_HEADER_SIZE = 2;  // 2-byte MPDU header (inside RS block)
static constexpr uint16_t RS_PAYLOAD_SIZE = 884;  // 884 bytes payload (inside RS block)
static constexpr uint16_t RS_INPUT_SIZE = VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + RS_PAYLOAD_SIZE;  // 892 bytes total input to RS
static constexpr uint16_t RS_OUTPUT_SIZE = 1020;  // RS(255,223) I=4 output size
static constexpr uint16_t FRAME_DATA_SIZE = ASM_SIZE + RS_OUTPUT_SIZE;  // 1024 total frame size
static constexpr uint16_t VCDU_OFFSET = ASM_SIZE;  // VCDU starts at byte 4 (inside RS block)
static constexpr uint16_t MPDU_OFFSET = VCDU_OFFSET + VCDU_HEADER_SIZE;  // MPDU at byte 10 (inside RS block)
static constexpr uint16_t PAYLOAD_OFFSET = MPDU_OFFSET + MPDU_HEADER_SIZE;  // Payload at byte 12 (inside RS block)

static constexpr uint8_t VCDU_TRANSFER_FRAME_VERSION = 0x01;  // AOS version (2 bits)
static constexpr uint16_t VCDU_SPACECRAFT_ID = 0x000;  // All zeros (10 bits)
static constexpr uint8_t VCDU_REPLAY_FLAG = 0;  // Always 0 (1 bit)
static constexpr uint8_t VCDU_CYCLE_USE_FLAG = 1;  // Cycle use flag (1 bit)
static constexpr uint8_t VCDU_FRAME_COUNT_CYCLE = 1;  // Frame count cycle (4 bits)
static constexpr uint8_t VCDU_DEFAULT_VCID = 0x00;  // Default VCID = 0 (6 bits)

// CCSDS Multiplexing Protocol Data Unit (MPDU) Header - CCSDS 732.0-B-5
// MPDU Sequence Control Flags (2 bits)
static constexpr uint8_t MPDU_SEQ_CONTINUATION = 0x00;  // 00: Continuation of packet
static constexpr uint8_t MPDU_SEQ_END = 0x01;          // 01: End of packet
static constexpr uint8_t MPDU_SEQ_START = 0x02;        // 10: Start of packet
static constexpr uint8_t MPDU_SEQ_UNSEGMENTED = 0x03;  // 11: Unsegmented packet (complete packet)
static constexpr uint16_t MPDU_NO_START_PACKET = 0xFFFF; // FHP = all ones: No packet start in this frame
static constexpr uint16_t MPDU_IDLE_DATA = 0xFFFE;       // FHP = all ones minus one: Only Idle Data in this frame
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
static constexpr uint32_t CADU_ASM = 0x1ACFFC1D;  // CCSDS standard ASM: 0x1A 0xCF 0xFC 0x1D

struct alignas(4) BPSKRandomizerFastLut {
    uint8_t seq[FRAME_SIZE];
    constexpr BPSKRandomizerFastLut() : seq{} {
        for (int i = 0; i < FRAME_SIZE; i++) {
            seq[i] = RANDOMIZER8_TABLE[i % 255];
        }
    }
};
// Removed constexpr to force the table into RAM to prevent Flash XIP cache misses
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
// Removed constexpr to force the table into RAM to prevent Flash XIP cache misses
static BPSKConvFastLut CCSDS_CONV_FAST_LUT;

// CCSDS OID (Operational Idle Data) Frame - 40-bit Galois LFSR per CCSDS 732.0-B-5 Appendix D
// OID frames contain VCDU header with VCID=0x3F, MPDU header, and idle data pattern
// Frame structure: ASM (4) + VCDU (6) + MPDU (2) + OID pattern (1012) = 1024 bytes
struct OIDFrameRandomizer {
    // Precompute 256 bytes of OID pattern (repeats every 256 bytes)
    // Initial seed (Galois form): 0x00000001FFFFFD (40 bits)
    // Polynomial: x^40 + x^37 + x^33 + x^31 + x^23 + x^17 + x^16 + x^15 + x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^2 + 1
    uint8_t pattern[256];
    
    OIDFrameRandomizer() {
        // 40-bit Galois LFSR state
        uint64_t state = 0x00000001FFFFFDULL & 0xFFFFFFFFFFULL;  // 40-bit seed
        
        // Generate 256 bytes of OID pattern
        for (int i = 0; i < 256; i++) {
            uint8_t byte_out = 0;
            // Extract 8 output bits
            for (int bit = 0; bit < 8; bit++) {
                // Galois LFSR: feedback from bit 0
                uint8_t feedback = (uint8_t)(state & 1);
                byte_out = (byte_out << 1) | feedback;
                
                // Shift and apply feedback to tap positions (40-bit polynomial)
                // Taps at: 40, 37, 33, 31, 23, 17, 16, 15, 11, 10, 7, 6, 5, 4, 2, 1
                if (feedback) {
                    state ^= 0x00000001B8000000ULL & 0xFFFFFFFFFFULL;  // XOR with feedback polynomial
                }
                state >>= 1;
            }
            pattern[i] = byte_out;
        }
    }
};
// Static instance - constructor runs at runtime to initialize pattern
static OIDFrameRandomizer OID_FRAME_PATTERN;

struct Crc16CcittFastLut {
    uint16_t table[256];
    constexpr Crc16CcittFastLut() : table{} {
        for (int i = 0; i < 256; i++) {
            uint16_t crc = (uint16_t)(i << 8);
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc = (crc << 1);
            }
            table[i] = crc;
        }
    }
};
static Crc16CcittFastLut CRC16_FAST_LUT;

__attribute__((always_inline)) static inline uint16_t calculate_fecf(const uint8_t *data, uint16_t length) {
    // CCSDS 732.0-B-5 Section 4.1.6.2: CRC-16-CCITT, Initial=0xFFFF, No final XOR
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ CRC16_FAST_LUT.table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

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
    
    static constexpr int DMA_BUFFER_MULTIPLIER = 2;
    uint32_t dma_buf[2][(FRAME_SIZE / 2) * DMA_BUFFER_MULTIPLIER];
    
    uint pin;
    uint32_t symbolrate_hz;
    uint16_t msg_len;
    bool msg_pending;
    volatile bool running;
    bool convolution_enabled;
    bool randomizer_enabled;
    bool reed_solomon_enabled;
    bool dual_basis_enabled;
    bool fecf_enabled;

    static constexpr uint32_t SP_FIFO_SIZE = 131072; // 128 KB, MUST be power of 2
    volatile uint8_t sp_fifo[SP_FIFO_SIZE];
    volatile uint32_t sp_fifo_head = 0;
    volatile uint32_t sp_fifo_tail = 0;

    uint32_t current_sp_size;
    uint32_t current_sp_offset;
    bool is_filler;
    uint8_t filler_header[6];
    uint16_t filler_seq;

    // VCDU (Virtual Channel Data Unit) state for AOS Level 0 Transfer Frames
    uint32_t vcdu_frame_count;  // Master frame counter (20 bits), reset on transmitter restart
    uint32_t vcdu_vcid_counters[64];  // Per-VCID counter (24 bits each) for 64 possible VCIDs

    // Deep enough to absorb math delays, small enough to prevent Out-Of-Memory crashes
    static constexpr int PENDING_FRAME_QUEUE_SIZE = 32;
    queue_t pending_frame_queue;

    struct alignas(4) DmaChunk {
        uint32_t words[FRAME_SIZE / 2];
        uint32_t length; // 32-bit perfectly aligns struct for ARM LDM/STM memory copies
        uint32_t is_user_data; // Tag to differentiate payload chunks from auto-generated OID chunks
    };

    alignas(4) uint8_t current_tx_frame[FRAME_SIZE];
    alignas(4) DmaChunk dma_irq_chunk;
    alignas(4) uint8_t rs_cadu_buffer[FRAME_SIZE];

    uint8_t conv_shift_reg;
    alignas(4) uint32_t tx_accum_data;
    uint8_t tx_accum_bits;
    uint8_t punc_phase;
    uint8_t conv_rate;
    alignas(4) uint8_t oid_frame[FRAME_SIZE];

    static constexpr int DMA_CHUNK_QUEUE_SIZE = 32;
    queue_t dma_chunk_queue;

    // Heap-allocated buffers to prevent Core 1 Stack Overflows (4KB stack limit)
    alignas(4) uint8_t mpdu_payload_buf[1012];
    alignas(4) uint8_t mpdu_temp_rs_buf[FRAME_SIZE];
    alignas(4) uint8_t bb_temp_rs_buf[FRAME_SIZE];
    alignas(4) DmaChunk bb_generation_chunk;

    volatile uint32_t tx_user_frames_generated = 0;
    volatile uint32_t tx_user_chunks_dma_cleared = 0;

    void flush_dma_chunks() {
        static DmaChunk dummy;
        while (queue_try_remove(&dma_chunk_queue, &dummy));
    }

    void flush_pending_frames() {
        static uint8_t dummy[FRAME_SIZE];
        while (queue_try_remove(&pending_frame_queue, dummy));
    }

    void clear_baseband_state() {
        conv_shift_reg = 0;
        tx_accum_data = 0;
        tx_accum_bits = 0;
        punc_phase = 0;
        
        sp_fifo_head = 0;
        sp_fifo_tail = 0;
        current_sp_size = 0;
        current_sp_offset = 0;
        is_filler = false;
        filler_seq = 0;
        tx_user_frames_generated = 0;
        tx_user_chunks_dma_cleared = 0;

        flush_pending_frames();
        flush_dma_chunks();
    }

public:
    alignas(4) uint8_t frame[FRAME_SIZE];
    volatile bool config_lock;
    
    void lock_config() {
        config_lock = true;
        __dmb(); // Force memory controller to instantly broadcast lock state to Core 1
        delay(2); // Wait 2ms to ensure Core 1 cleanly exits its baseband processing loop
    }

    uint8_t get_dma_chunks_count() const {
        return queue_get_level(const_cast<queue_t*>(&dma_chunk_queue));
    }

    void unlock_config() {
        __dmb();
        config_lock = false;
        __dmb();
    }
    
    BPSKModulator(uint pin, uint32_t rate = 1200) : pin(pin), symbolrate_hz(rate), 
          msg_len(0), msg_pending(false), running(false),
                    convolution_enabled(true), randomizer_enabled(true), reed_solomon_enabled(true), dual_basis_enabled(false),
                    fecf_enabled(true), conv_shift_reg(0), tx_accum_data(0), tx_accum_bits(0), punc_phase(0), conv_rate(0),
                    vcdu_frame_count(0),
                    config_lock(false), current_sp_size(0), current_sp_offset(0), is_filler(false), filler_seq(0) {
        g_modulator_instance = this;
        queue_init(&pending_frame_queue, FRAME_SIZE, PENDING_FRAME_QUEUE_SIZE);
        queue_init(&dma_chunk_queue, sizeof(DmaChunk), DMA_CHUNK_QUEUE_SIZE);
        memset(vcdu_vcid_counters, 0, sizeof(vcdu_vcid_counters));
        build_oid_frame();
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

    uint32_t get_sp_fifo_count() const {
        uint32_t head = sp_fifo_head;
        __dmb(); // Prevent speculative reads of the FIFO buffer before the head pointer is evaluated
        return (head - sp_fifo_tail) & (SP_FIFO_SIZE - 1);
    }

    uint32_t get_sp_fifo_free() const {
        return SP_FIFO_SIZE - 1 - get_sp_fifo_count();
    }

    bool push_sp_byte(uint8_t b) {
        if (get_sp_fifo_free() == 0) return false;
        sp_fifo[sp_fifo_head] = b;
        __dmb(); // Force byte to physical RAM before head increments to prevent Core 1 from reading garbage
        sp_fifo_head = (sp_fifo_head + 1) & (SP_FIFO_SIZE - 1);
        return true;
    }

    uint8_t pop_sp_byte() {
        if (get_sp_fifo_count() == 0) return 0;
        uint8_t b = sp_fifo[sp_fifo_tail];
        __dmb(); // Ensure read completes before advancing tail
        sp_fifo_tail = (sp_fifo_tail + 1) & (SP_FIFO_SIZE - 1);
        return b;
    }

    uint8_t peek_sp_byte(uint32_t offset) const {
        return sp_fifo[(sp_fifo_tail + offset) & (SP_FIFO_SIZE - 1)];
    }

    void clear_sp_fifo() {
        lock_config();
        sp_fifo_head = 0;
        sp_fifo_tail = 0;
        current_sp_size = 0;
        current_sp_offset = 0;
        is_filler = false;
        unlock_config();
    }

    void prepare_tx_stream() {
        // DMA prepares chunks synchronously via ISR. Empty wrapper preserves compatibility.
    }

    void randomize_frame_data(uint16_t start_index) {
        for (uint16_t i = start_index; i < FRAME_SIZE; i++) {
            frame[i] ^= CCSDS_RANDOMIZER_FAST_LUT.seq[i - start_index];
        }
    }


    __attribute__((section(".time_critical.build_vcdu_header"))) void build_vcdu_header(uint8_t *frame, uint16_t frame_offset, uint8_t vcid, uint32_t frame_count) {
        // Build VCDU (Virtual Channel Data Unit) Transfer Frame Header (6 bytes) per CCSDS 732.0-B-5
        // frame_offset: offset into frame buffer (typically ASM_SIZE = 4 for RS blocks)
        
        uint8_t *header = frame + frame_offset;
        
        // Byte 0: TFVN (2 bits) = 01 | SCID LSB (6 bits, bits 7-2 of 10-bit SCID)
        header[0] = (VCDU_TRANSFER_FRAME_VERSION << 6) | ((VCDU_SPACECRAFT_ID >> 2) & 0x3F);
        
        // Byte 1: SCID LSB (2 bits, bits 1-0 of 10-bit SCID) | VCID (6 bits)
        header[1] = ((VCDU_SPACECRAFT_ID & 0x03) << 6) | (vcid & 0x3F);
        
        // Bytes 2-4: Frame Count (24 bits, 3 octets)
        header[2] = (frame_count >> 16) & 0xFF;  // Bits 23-16
        header[3] = (frame_count >> 8) & 0xFF;   // Bits 15-8
        header[4] = frame_count & 0xFF;          // Bits 7-0
        
        // Byte 5: Replay Flag (1 bit) | Cycle Use Flag (1 bit) | SCID MSB (2 bits, bits 9-8) | Frame Count Cycle (4 bits)
        header[5] = (VCDU_REPLAY_FLAG << 7) | (VCDU_CYCLE_USE_FLAG << 6) | 
                   ((VCDU_SPACECRAFT_ID >> 8) & 0x03) << 4 | (VCDU_FRAME_COUNT_CYCLE & 0x0F);
    }

    void reset_vcdu_counters() {
        vcdu_frame_count = 0;
        memset(vcdu_vcid_counters, 0, sizeof(vcdu_vcid_counters));
    }

    __attribute__((section(".time_critical.build_mpdu_header"))) void build_mpdu_header(uint8_t *frame, uint16_t frame_offset, uint16_t first_header_ptr) {
        // Build MPDU (Multiplexing Protocol Data Unit) Header (2 bytes) per CCSDS 732.0-B-5
        // Bits 0-15: First Header Pointer (16-bit value, position of first packet start in MPDU Packet Zone)
        // frame_offset: offset into frame buffer
        
        uint8_t *header = frame + frame_offset;
        
        // MPDU Byte 0: FHP bits 8-15 (upper byte)
        header[0] = (first_header_ptr >> 8) & 0xFF;
        
        // MPDU Byte 1: FHP bits 0-7 (lower byte)
        header[1] = first_header_ptr & 0xFF;
    }



    void randomize_buffer_data(uint8_t *buffer, uint16_t start_index) {
        for (uint16_t i = start_index; i < FRAME_SIZE; i++) {
            buffer[i] ^= CCSDS_RANDOMIZER_FAST_LUT.seq[i - start_index];
        }
    }

    void build_oid_frame() {
        // Build OID (Operational Idle Data) frame per CCSDS 732.0-B-5
        // OID frames have VCID set to 0x3F (all ones) to indicate idle data
        // Frame structure: ASM + VCDU (VCID=0x3F) + MPDU + OID pattern
        
        // ASM at bytes 0-3
        oid_frame[0] = 0x1A;
        oid_frame[1] = 0xCF;
        oid_frame[2] = 0xFC;
        oid_frame[3] = 0x1D;
        
        // VCDU header at bytes 4-9 with VCID=0x3F (OID indicator)
        // Frame count and other VCDU fields set to default values
        build_vcdu_header(oid_frame, VCDU_OFFSET, 0x3F, 0);
        
        // For OID frames (VCID 63), there is NO MPDU header!
        // OID pattern starts immediately after the VCDU header.
        uint16_t length_to_crc = (reed_solomon_enabled ? RS_INPUT_SIZE : (FRAME_SIZE - ASM_SIZE)) - (fecf_enabled ? 2 : 0);
        uint16_t oid_payload_size = length_to_crc - VCDU_HEADER_SIZE;
        for (uint16_t i = 0; i < oid_payload_size; i++) {
            oid_frame[ASM_SIZE + VCDU_HEADER_SIZE + i] = OID_FRAME_PATTERN.pattern[i & 0xFF];
        }

        if (fecf_enabled) {
            uint16_t crc = calculate_fecf(oid_frame + ASM_SIZE, length_to_crc);
            oid_frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
            oid_frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
        }
    }

    void init_idle_data_frame(uint8_t vcid = VCDU_DEFAULT_VCID) {
        // Build an MPDU Idle Data frame per CCSDS 732.0-B-5 Section 4.1.4.1
        // These frames contain VCDU/MPDU headers with idle data in the Packet Zone
        // Used to maintain synchronous transmission when no real packets are available
        
        // ASM at bytes 0-3
        frame[0] = 0x1A;
        frame[1] = 0xCF;
        frame[2] = 0xFC;
        frame[3] = 0x1D;
        
        // VCDU header at bytes 4-9 with specified VCID
        build_vcdu_header(frame, VCDU_OFFSET, vcid, vcdu_vcid_counters[vcid]);
        vcdu_vcid_counters[vcid]++;  // Increment per-VCID counter
        vcdu_frame_count++;  // Increment master frame counter
        
        uint16_t length_to_crc = (reed_solomon_enabled ? RS_INPUT_SIZE : (FRAME_SIZE - ASM_SIZE)) - (fecf_enabled ? 2 : 0);
        
        if (vcid == 0x3F) {
            // OID Frame: No MPDU header
            uint16_t payload_size = length_to_crc - VCDU_HEADER_SIZE;
            for (uint16_t i = 0; i < payload_size; i++) {
                frame[VCDU_OFFSET + VCDU_HEADER_SIZE + i] = OID_FRAME_PATTERN.pattern[i & 0xFF];
            }
        } else {
            // MPDU Idle Frame
            build_mpdu_header(frame, MPDU_OFFSET, MPDU_IDLE_DATA);
            uint16_t payload_size = length_to_crc - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;
            for (uint16_t i = 0; i < payload_size; i++) {
                frame[PAYLOAD_OFFSET + i] = OID_FRAME_PATTERN.pattern[i & 0xFF];
            }
        }
        
        if (fecf_enabled) {
            uint16_t crc = calculate_fecf(frame + ASM_SIZE, length_to_crc);
            frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
            frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
        }
        
        // Apply RS encoding if enabled
        if (reed_solomon_enabled) {
            // RS block: 892 input bytes (VCDU + MPDU + payload) → 1020 output bytes
            // Copy data to temporary RS buffer for encoding
            for (int i = 0; i < RS_INPUT_SIZE; i++) {
                rs_cadu_buffer[ASM_SIZE + i] = frame[ASM_SIZE + i];
            }
            
            // RS encode with I=4 interleaving
            static const int I = 4;
            static const int RS_K = 223;
            static const int RS_N = 255;
            for (int i = 0; i < I; i++) {
                uint8_t msg[RS_K];
                uint8_t parity[RS_N - RS_K];
                for (int c = 0; c < RS_K; c++) {
                    msg[c] = rs_cadu_buffer[ASM_SIZE + c * I + i];
                }
                rs_encode(msg, parity);
                for (int c = 0; c < RS_K; c++) {
                    rs_cadu_buffer[ASM_SIZE + c * I + i] = msg[c];
                }
                for (int c = 0; c < (RS_N - RS_K); c++) {
                    rs_cadu_buffer[ASM_SIZE + RS_K * I + c * I + i] = parity[c];
                }
            }
            
            // Copy RS output back to frame buffer
            memcpy(frame + ASM_SIZE, rs_cadu_buffer + ASM_SIZE, RS_OUTPUT_SIZE);
            
            if (dual_basis_enabled) {
                rs_apply_dual_basis(frame + ASM_SIZE, RS_OUTPUT_SIZE);
            }
            
            if (randomizer_enabled) {
                uint32_t *frame_words = (uint32_t*)(frame + ASM_SIZE);
                uint32_t *lut_words = (uint32_t*)(CCSDS_RANDOMIZER_FAST_LUT.seq);
                for (uint16_t i = 0; i < RS_OUTPUT_SIZE / 4; i++) {
                    frame_words[i] ^= lut_words[i];
                }
            }
        } else {
            // Non-RS mode: just copy the frame with randomization if enabled
            if (randomizer_enabled) {
                uint32_t *frame_words = (uint32_t*)(frame + ASM_SIZE);
                uint32_t *lut_words = (uint32_t*)(CCSDS_RANDOMIZER_FAST_LUT.seq);
                for (uint16_t i = 0; i < (FRAME_SIZE - ASM_SIZE) / 4; i++) {
                    frame_words[i] ^= lut_words[i];
                }
            }
        }
    }

    uint8_t get_pending_frames_count() const {
        return queue_get_level(const_cast<queue_t*>(&pending_frame_queue));
    }

    bool push_pending_frame(const uint8_t* frame_data) {
        if (queue_try_add(&pending_frame_queue, frame_data)) {
            tx_user_frames_generated++; // Track all user-provided data frames
            return true;
        }
        return false;
    }

    bool pop_pending_frame(uint8_t* frame_data) {
        return queue_try_remove(&pending_frame_queue, frame_data);
    }

    uint8_t* get_oid_frame() {
        return oid_frame;
    }

    uint32_t get_active_user_chunks() const {
        return tx_user_frames_generated - tx_user_chunks_dma_cleared;
    }

    bool queue_oid_frame() {
        return push_pending_frame(oid_frame);
    }

    bool queue_idle_data_frame(uint8_t vcid = VCDU_DEFAULT_VCID) {
        // Build an idle data MPDU frame with specified VCID and queue it
        init_idle_data_frame(vcid);
        return push_pending_frame(frame);
    }

    bool can_queue_binary_frame() const {
        uint8_t count = get_pending_frames_count();
        if (reed_solomon_enabled) {
            return (count <= (PENDING_FRAME_QUEUE_SIZE - 2));
        }
        return (count < PENDING_FRAME_QUEUE_SIZE - 1);
    }

    __attribute__((section(".time_critical.has_data_to_send"))) bool has_data_to_send() const {
        if (current_sp_offset < current_sp_size) return true; // Actively mid-packet, must finish
        if (get_sp_fifo_free() == 0) return true; // FIFO is 100% full (emergency flush to unblock USB)
        
        uint32_t fifo_count = get_sp_fifo_count();
        if (fifo_count >= 6) {
            // Check if the head of the FIFO contains RF garbage that needs to be dropped
            if ((peek_sp_byte(0) & 0xF8) != 0x00 || (peek_sp_byte(2) & 0xC0) != 0xC0 || peek_sp_byte(4) >= 0x40) {
                return true; 
            }
            
            // Valid header. Ensure we have buffered the ENTIRE packet before authorizing generation
            uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
            if (fifo_count >= (uint32_t)(pdl + 7)) return true;
        }
        return false; // Not enough data, safe to bypass frame generator and send pre-calculated OID Idle Frames
    }

    __attribute__((section(".time_critical.generate_and_queue_mpdu"))) bool generate_and_queue_mpdu(uint8_t vcid = VCDU_DEFAULT_VCID) {
        if (!can_queue_binary_frame()) return false;

        uint16_t fhp = 0xFFFF;
        uint16_t length_to_crc = (reed_solomon_enabled ? RS_INPUT_SIZE : (FRAME_SIZE - ASM_SIZE)) - (fecf_enabled ? 2 : 0);
        uint16_t payload_size = length_to_crc - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;

        for (int i = 0; i < payload_size; i++) {
            if (current_sp_offset >= current_sp_size) {
                if (fhp == 0xFFFF) fhp = i; // First Header Pointer points to the start of this packet

                bool have_user_packet = false;
                while (get_sp_fifo_count() >= 6 && 
                      ((peek_sp_byte(0) & 0xF8) != 0x00 || 
                       (peek_sp_byte(2) & 0xC0) != 0xC0 ||
                       peek_sp_byte(4) >= 0x40)) {
                    pop_sp_byte(); // Drop byte to resynchronize space packet stream
                }
                if (get_sp_fifo_count() >= 6) {
                    uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
                    uint32_t total_len = pdl + 7;
                    if (get_sp_fifo_count() >= total_len) {
                        have_user_packet = true;
                        current_sp_size = total_len;
                        current_sp_offset = 0;
                        is_filler = false;
                    }
                }

                if (!have_user_packet) {
                    uint16_t filler_size = (payload_size - i >= 7) ? (payload_size - i) : 7;
                    current_sp_size = filler_size;
                    current_sp_offset = 0;
                    is_filler = true;
                    filler_header[0] = 0x07;
                    filler_header[1] = 0xFF;
                    filler_header[2] = 0xC0 | ((filler_seq >> 8) & 0x3F);
                    filler_header[3] = filler_seq & 0xFF;
                    filler_seq = (filler_seq + 1) & 0x3FFF;
                    uint16_t pdl = filler_size - 7;
                    filler_header[4] = (pdl >> 8) & 0xFF;
                    filler_header[5] = pdl & 0xFF;
                }
            }

            if (is_filler) {
                if (current_sp_offset < 6) {
                    mpdu_payload_buf[i] = filler_header[current_sp_offset];
                } else {
                    mpdu_payload_buf[i] = 0x00; // Idle payload (all zeros per CCSDS recommendation)
                }
            } else {
                mpdu_payload_buf[i] = pop_sp_byte();
            }
            current_sp_offset++;
        }

        // Now assemble the frame
        frame[0] = 0x1A;
        frame[1] = 0xCF;
        frame[2] = 0xFC;
        frame[3] = 0x1D;
        
        build_vcdu_header(frame, ASM_SIZE, vcid, vcdu_vcid_counters[vcid]);
        vcdu_vcid_counters[vcid]++;
        vcdu_frame_count++;

        build_mpdu_header(frame, ASM_SIZE + VCDU_HEADER_SIZE, fhp);
        memcpy(frame + PAYLOAD_OFFSET, mpdu_payload_buf, payload_size);

        if (fecf_enabled) {
            uint16_t crc = calculate_fecf(frame + ASM_SIZE, length_to_crc);
            frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
            frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
        }

        if (reed_solomon_enabled) {
            static const int I = 4;
            static const int RS_K = 223;
            static const int RS_N = 255;
            
            // Temporary buffer for RS encoding
            memcpy(mpdu_temp_rs_buf, frame, FRAME_SIZE);
            
            for (int i = 0; i < I; i++) {
                uint8_t msg[RS_K];
                uint8_t parity[RS_N - RS_K];
                for (int c = 0; c < RS_K; c++) {
                    msg[c] = mpdu_temp_rs_buf[ASM_SIZE + c * I + i];
                }
                rs_encode(msg, parity);
                for (int c = 0; c < RS_K; c++) {
                    mpdu_temp_rs_buf[ASM_SIZE + c * I + i] = msg[c];
                }
                for (int c = 0; c < (RS_N - RS_K); c++) {
                    mpdu_temp_rs_buf[ASM_SIZE + RS_K * I + c * I + i] = parity[c];
                }
            }
            
            if (dual_basis_enabled) {
                rs_apply_dual_basis(mpdu_temp_rs_buf + ASM_SIZE, 1020);
            }
            
            memcpy(frame, mpdu_temp_rs_buf, FRAME_SIZE);
        }
        
        if (randomizer_enabled) {
            uint32_t *frame_words = (uint32_t*)(frame + ASM_SIZE);
            uint32_t *lut_words = (uint32_t*)(CCSDS_RANDOMIZER_FAST_LUT.seq);
            for (uint16_t i = 0; i < (FRAME_SIZE - ASM_SIZE) / 4; i++) {
                frame_words[i] ^= lut_words[i];
            }
        }
        
        return push_pending_frame(frame);
    }

    void queue_message(const uint8_t *msg, uint16_t len) {
        lock_config();
        uint32_t total_len = len + 6;
        if (get_sp_fifo_free() >= total_len) {
            static uint16_t msg_seq = 0;
            push_sp_byte(0x00); // Version 0, Type 0, Sec 0, APID 1
            push_sp_byte(0x01);
            push_sp_byte(0xC0 | ((msg_seq >> 8) & 0x3F));
            push_sp_byte(msg_seq & 0xFF);
            msg_seq = (msg_seq + 1) & 0x3FFF;
            uint16_t pdl = len - 1;
            push_sp_byte((pdl >> 8) & 0xFF);
            push_sp_byte(pdl & 0xFF);
            for (uint16_t i = 0; i < len; i++) {
                push_sp_byte(msg[i]);
            }
        }
        unlock_config();
    }

    void clear_frame() {
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        memset(frame, 0, FRAME_SIZE);
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
        if (!enabled) {
            dual_basis_enabled = false;
        }
        clear_baseband_state();
        build_oid_frame();
        
        if (was_running) start();
        unlock_config();
    }

    bool get_reed_solomon_enabled() const { return reed_solomon_enabled; }

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
        build_oid_frame();
        
        if (was_running) start();
        unlock_config();
    }
    bool get_dual_basis_enabled() const { return dual_basis_enabled; }

    void set_fecf_enabled(bool enabled) {
        if (fecf_enabled == enabled) return;
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        fecf_enabled = enabled;
        clear_baseband_state();
        build_oid_frame();
        
        if (was_running) start();
        unlock_config();
    }
    bool get_fecf_enabled() const { return fecf_enabled; }

    void set_randomizer_enabled(bool enabled) {
        if (randomizer_enabled == enabled) return; // Do not interrupt running stream if unchanged!
        
        lock_config();
        bool was_running = running;
        if (was_running) stop();
        
        randomizer_enabled = enabled;
        clear_baseband_state();
        build_oid_frame();
        
        if (was_running) start();
        unlock_config();
    }

    bool get_randomizer_enabled() const { return randomizer_enabled; }

    void restart_transmission() {
        lock_config();
        reset_vcdu_counters();  // Reset frame counter and VCID counters on transmitter restart
        build_oid_frame();   // Rebuild OID frame with reset counters
        if (running) {
            stop();
            start();
        }
        unlock_config();
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
        memcpy(frame, oid_frame, FRAME_SIZE);
        prepare_tx_stream();
    }

    __attribute__((section(".time_critical.process_baseband_to_dma"))) bool process_baseband_to_dma() {
        if (queue_is_full(&dma_chunk_queue)) return false;

        bool is_tracked_frame = true;
        if (!pop_pending_frame(current_tx_frame)) {
            is_tracked_frame = false; // Mark this as an auto-generated Idle chunk
            // Always generate a fresh OID frame when no user data is available (CCSDS compliant idle data)
            // This ensures continuous transmission to prevent Viterbi lock loss
            memcpy(current_tx_frame, oid_frame, FRAME_SIZE);
            
            // Update VCDU header for OID frame inside RS block (bytes 4-9)
            build_vcdu_header(current_tx_frame, VCDU_OFFSET, 0x3F, vcdu_vcid_counters[0x3F]);
            vcdu_vcid_counters[0x3F]++;  // Increment OID VCID counter
            vcdu_frame_count++;  // Increment master frame counter
            
            // Note: NO MPDU header update here! VCID 63 does not have one!
            
            if (fecf_enabled) {
                uint16_t length_to_crc = (reed_solomon_enabled ? RS_INPUT_SIZE : (FRAME_SIZE - ASM_SIZE)) - 2;
                uint16_t crc = calculate_fecf(current_tx_frame + ASM_SIZE, length_to_crc);
                current_tx_frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
                current_tx_frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
            }
            
            // Apply RS encoding and randomization to OID frame if enabled
            if (reed_solomon_enabled) {
                static const int I = 4;
                static const int RS_K = 223;
                static const int RS_N = 255;
                
                // Temporary buffer for RS encoding (1024 bytes total)
                memcpy(bb_temp_rs_buf, current_tx_frame, FRAME_SIZE);

                
                // RS encode the frame (interleaved encoding, 4 codewords)
                for (int i = 0; i < I; i++) {
                    uint8_t msg[RS_K];
                    uint8_t parity[RS_N - RS_K];
                    for (int c = 0; c < RS_K; c++) {
                        msg[c] = bb_temp_rs_buf[ASM_SIZE + c * I + i];
                    }
                    rs_encode(msg, parity);
                    for (int c = 0; c < RS_K; c++) {
                        bb_temp_rs_buf[ASM_SIZE + c * I + i] = msg[c];
                    }
                    for (int c = 0; c < (RS_N - RS_K); c++) {
                        bb_temp_rs_buf[ASM_SIZE + RS_K * I + c * I + i] = parity[c];
                    }
                }
                
                if (dual_basis_enabled) {
                    rs_apply_dual_basis(bb_temp_rs_buf + ASM_SIZE, 1020);
                }
                
                memcpy(current_tx_frame, bb_temp_rs_buf, FRAME_SIZE);
            }
            
            // Apply randomization if enabled (skips ASM, starts at byte 4)
            if (randomizer_enabled) {
                uint32_t *frame_words = (uint32_t*)(current_tx_frame + ASM_SIZE);
                uint32_t *lut_words = (uint32_t*)(CCSDS_RANDOMIZER_FAST_LUT.seq);
                for (uint16_t i = 0; i < (FRAME_SIZE - ASM_SIZE) / 4; i++) {
                    frame_words[i] ^= lut_words[i];
                }
            }
        }

        uint32_t words_written = 0;

        if (!convolution_enabled) {
            uint32_t* frame_words = (uint32_t*)current_tx_frame;
            for (uint16_t i = 0; i < FRAME_SIZE / 4; i++) {
                uint32_t word = __builtin_bswap32(frame_words[i]);
            bb_generation_chunk.words[words_written++] = word;
            }
        } else {
            if (conv_rate == 0 && tx_accum_bits == 0) {
                for (uint16_t i = 0; i < FRAME_SIZE; i += 2) {
                    uint32_t word = 0;
                    for (int b = 0; b < 2; b++) {
                        uint8_t current_byte = current_tx_frame[i + b];
                        
                        uint16_t out_word = CCSDS_CONV_FAST_LUT.byte_out[current_byte] ^ CCSDS_CONV_FAST_LUT.state_out[conv_shift_reg];
                        conv_shift_reg = CCSDS_CONV_FAST_LUT.next_state[current_byte];
                        
                        word = (word << 16) | out_word;
                    }
                    bb_generation_chunk.words[words_written++] = word;
                }
            } else {
                uint32_t accum_data = tx_accum_data;
                uint8_t accum_bits = tx_accum_bits;
                uint8_t p_phase = punc_phase;
                uint8_t p_period = PUNC_PERIOD[conv_rate];
                uint8_t c1_mask = PUNC_C1[conv_rate];
                uint8_t c2_mask = PUNC_C2[conv_rate];

                for (uint16_t i = 0; i < FRAME_SIZE; i++) {
                    uint8_t current_byte = current_tx_frame[i];
                    uint16_t out_word = CCSDS_CONV_FAST_LUT.byte_out[current_byte] ^ CCSDS_CONV_FAST_LUT.state_out[conv_shift_reg];
                    conv_shift_reg = CCSDS_CONV_FAST_LUT.next_state[current_byte];

                    for (int b = 7; b >= 0; b--) {
                        uint8_t c1 = (out_word >> (b * 2 + 1)) & 1;
                        uint8_t c2 = (out_word >> (b * 2)) & 1;

                        if ((c1_mask >> p_phase) & 1) {
                            accum_data = (accum_data << 1) | c1;
                            accum_bits++;
                            if (accum_bits == 32) {
                            bb_generation_chunk.words[words_written++] = accum_data;
                                accum_bits = 0;
                                accum_data = 0;
                            }
                        }
                        if ((c2_mask >> p_phase) & 1) {
                            accum_data = (accum_data << 1) | c2;
                            accum_bits++;
                            if (accum_bits == 32) {
                            bb_generation_chunk.words[words_written++] = accum_data;
                                accum_bits = 0;
                                accum_data = 0;
                            }
                        }
                        p_phase++;
                        if (p_phase >= p_period) p_phase = 0;
                    }
                }
                tx_accum_data = accum_data;
                tx_accum_bits = accum_bits;
                punc_phase = p_phase;
            }
        }
    bb_generation_chunk.length = words_written;
    bb_generation_chunk.is_user_data = is_tracked_frame ? 1 : 0;
    queue_try_add(&dma_chunk_queue, &bb_generation_chunk);
        return true;
    }

    __attribute__((section(".time_critical.fill_dma_buffer"))) uint32_t fill_dma_buffer(uint32_t *out_buf) {
        uint32_t total_words = 0;
        // Batch pull up to DMA_BUFFER_MULTIPLIER chunks to give Core 0 massive breathing room
        while (total_words + (FRAME_SIZE / 2) <= (FRAME_SIZE / 2) * DMA_BUFFER_MULTIPLIER) {
            if (queue_try_remove(&dma_chunk_queue, &dma_irq_chunk)) {
                if (dma_irq_chunk.is_user_data) {
                    tx_user_chunks_dma_cleared++; // Acknowledge user data successfully hit the RF transmission stage
                }
                if (dma_irq_chunk.length > 0) {
                    memcpy(out_buf + total_words, dma_irq_chunk.words, dma_irq_chunk.length * 4);
                    total_words += dma_irq_chunk.length;
                }
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

    __attribute__((section(".time_critical.handle_dma_irq"))) void handle_dma_irq() {
        if (!running) {
            // Safely sink phantom interrupts from aborted transfers to prevent memory corruption
            if (dma_channel_get_irq0_status(dma_chan_0)) dma_channel_acknowledge_irq0(dma_chan_0);
            if (dma_channel_get_irq0_status(dma_chan_1)) dma_channel_acknowledge_irq0(dma_chan_1);
            return;
        }
        
        if (dma_channel_get_irq0_status(dma_chan_0)) {
            dma_channel_acknowledge_irq0(dma_chan_0);
            dma_channel_start(dma_chan_1); // Software chain
            
            uint32_t len = fill_dma_buffer(dma_buf[0]);
            dma_channel_set_trans_count(dma_chan_0, len, false);
            dma_channel_set_read_addr(dma_chan_0, dma_buf[0], false);
        } 
        else if (dma_channel_get_irq0_status(dma_chan_1)) {
            dma_channel_acknowledge_irq0(dma_chan_1);
            dma_channel_start(dma_chan_0); // Software chain
            
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

        flush_dma_chunks();
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_set_enabled(pio, sm, false);

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
        __dmb(); // Ensure running state is visible to the ISR before it can be triggered
        
        dma_channel_start(dma_chan_0);
        pio_sm_set_enabled(pio, sm, true);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
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

void __not_in_flash_func(bpsk_dma_isr)() {
    if (g_modulator_instance != nullptr) {
        g_modulator_instance->handle_dma_irq();
    }
}

#endif