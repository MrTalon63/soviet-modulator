#pragma once

#include <cstdint>

// CCSDS legacy 8-bit randomizer table used by SatDump.
static constexpr uint8_t RANDOMIZER8_TABLE[255] = {
    0xff, 0x48, 0x0e, 0xc0, 0x9a, 0x0d, 0x70, 0xbc, 0x8e, 0x2c, 0x93, 0xad, 0xa7, 0xb7, 0x46, 0xce, 0x5a, 0x97, 0x7d, 0xcc, 0x32, 0xa2, 0xbf, 0x3e, 0x0a, 0x10, 0xf1, 0x88, 0x94, 0xcd, 0xea, 0xb1,
    0xfe, 0x90, 0x1d, 0x81, 0x34, 0x1a, 0xe1, 0x79, 0x1c, 0x59, 0x27, 0x5b, 0x4f, 0x6e, 0x8d, 0x9c, 0xb5, 0x2e, 0xfb, 0x98, 0x65, 0x45, 0x7e, 0x7c, 0x14, 0x21, 0xe3, 0x11, 0x29, 0x9b, 0xd5, 0x63,
    0xfd, 0x20, 0x3b, 0x02, 0x68, 0x35, 0xc2, 0xf2, 0x38, 0xb2, 0x4e, 0xb6, 0x9e, 0xdd, 0x1b, 0x39, 0x6a, 0x5d, 0xf7, 0x30, 0xca, 0x8a, 0xfc, 0xf8, 0x28, 0x43, 0xc6, 0x22, 0x53, 0x37, 0xaa, 0xc7,
    0xfa, 0x40, 0x76, 0x04, 0xd0, 0x6b, 0x85, 0xe4, 0x71, 0x64, 0x9d, 0x6d, 0x3d, 0xba, 0x36, 0x72, 0xd4, 0xbb, 0xee, 0x61, 0x95, 0x15, 0xf9, 0xf0, 0x50, 0x87, 0x8c, 0x44, 0xa6, 0x6f, 0x55, 0x8f,
    0xf4, 0x80, 0xec, 0x09, 0xa0, 0xd7, 0x0b, 0xc8, 0xe2, 0xc9, 0x3a, 0xda, 0x7b, 0x74, 0x6c, 0xe5, 0xa9, 0x77, 0xdc, 0xc3, 0x2a, 0x2b, 0xf3, 0xe0, 0xa1, 0x0f, 0x18, 0x89, 0x4c, 0xde, 0xab, 0x1f,
    0xe9, 0x01, 0xd8, 0x13, 0x41, 0xae, 0x17, 0x91, 0xc5, 0x92, 0x75, 0xb4, 0xf6, 0xe8, 0xd9, 0xcb, 0x52, 0xef, 0xb9, 0x86, 0x54, 0x57, 0xe7, 0xc1, 0x42, 0x1e, 0x31, 0x12, 0x99, 0xbd, 0x56, 0x3f,
    0xd2, 0x03, 0xb0, 0x26, 0x83, 0x5c, 0x2f, 0x23, 0x8b, 0x24, 0xeb, 0x69, 0xed, 0xd1, 0xb3, 0x96, 0xa5, 0xdf, 0x73, 0x0c, 0xa8, 0xaf, 0xcf, 0x82, 0x84, 0x3c, 0x62, 0x25, 0x33, 0x7a, 0xac, 0x7f,
    0xa4, 0x07, 0x60, 0x4d, 0x06, 0xb8, 0x5e, 0x47, 0x16, 0x49, 0xd6, 0xd3, 0xdb, 0xa3, 0x67, 0x2d, 0x4b, 0xbe, 0xe6, 0x19, 0x51, 0x5f, 0x9f, 0x05, 0x08, 0x78, 0xc4, 0x4a, 0x66, 0xf5, 0x58,
};

struct alignas(4) BPSKRandomizer8FastLut {
    uint8_t seq[MAX_FRAME_SIZE];
    constexpr BPSKRandomizer8FastLut() : seq{} {
        for (int i = 0; i < MAX_FRAME_SIZE; i++) {
            seq[i] = RANDOMIZER8_TABLE[i % 255];
        }
    }
};

static constexpr BPSKRandomizer8FastLut CCSDS_RANDOMIZER8_FAST_LUT;

struct alignas(4) BPSKRandomizer17FastLut {
    uint8_t seq[MAX_FRAME_SIZE];
    constexpr BPSKRandomizer17FastLut() : seq{} {
        uint32_t state = 0x38E3; // Initial state (seed) for S1..S17 (MSB..LSB binary '00011100011100011')
        for (int i = 0; i < MAX_FRAME_SIZE; i++) {
            uint8_t byte_val = 0;
            for (int bit = 0; bit < 8; bit++) {
                uint8_t out_bit = (state >> 16) & 1;
                uint8_t feedback = ((state >> 16) ^ (state >> 2)) & 1;
                state = ((state << 1) | feedback) & 0x1FFFF;
                byte_val = (byte_val << 1) | out_bit;
            }
            seq[i] = byte_val;
        }
    }
};

static constexpr BPSKRandomizer17FastLut CCSDS_RANDOMIZER17_FAST_LUT;

// CCSDS OID (Operational Idle Data) Frame - 40-bit Galois LFSR per CCSDS 732.0-B-5 Appendix D
// OID frames contain a VCDU header with VCID=0x3F followed immediately by the OID idle data pattern.
// Note: OID frames do NOT contain an MPDU header.
// Frame structure: ASM (4) + VCDU (6) + OID pattern (variable length based on interleave/coding configuration).
struct OIDFrameRandomizer {
    // Precompute MAX_FRAME_SIZE bytes of OID pattern
    // Initial seed (Galois form): 0x00000001FFFFFD (40 bits)
    // Polynomial: x^40 + x^37 + x^33 + x^31 + x^23 + x^17 + x^16 + x^15 + x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^2 + 1
    uint8_t pattern[MAX_FRAME_SIZE];

    constexpr OIDFrameRandomizer() : pattern{} {
        // 40-bit Galois LFSR state
        uint64_t state = 0x00000001FFFFFDULL & 0xFFFFFFFFFFULL; // 40-bit seed

        // Generate MAX_FRAME_SIZE bytes of OID pattern
        for (int i = 0; i < MAX_FRAME_SIZE; i++) {
            uint8_t byte_out = 0;
            // Extract 8 output bits
            for (int bit = 0; bit < 8; bit++) {
                // Galois LFSR: feedback from bit 0
                uint8_t feedback = (uint8_t)(state & 1);
                byte_out = (byte_out << 1) | feedback;

                // Shift and apply feedback to tap positions (40-bit polynomial)
                // Taps at: 40, 37, 33, 31, 23, 17, 16, 15, 11, 10, 7, 6, 5, 4, 2, 1
                if (feedback) {
                    state ^= 0x00000001B8000000ULL & 0xFFFFFFFFFFULL; // XOR with feedback polynomial
                }
                state >>= 1;
            }
            pattern[i] = byte_out;
        }
    }
};
// Static instance - precomputed at compile time and stored in Flash/ROM
static constexpr OIDFrameRandomizer OID_FRAME_PATTERN;

struct Crc16CcittFastLut {
    uint16_t table[256];
    constexpr Crc16CcittFastLut() : table{} {
        for (int i = 0; i < 256; i++) {
            uint16_t crc = (uint16_t)(i << 8);
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc = (crc << 1);
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
