#include "rs.h"
#include <cstring>

// A compact Reed-Solomon (255, 223) implementation for CCSDS.
// Based on a public domain implementation.

static uint8_t gf_exp[512]; // Expanded to avoid modulo 255 in math
static uint8_t gf_log[256];
static uint8_t rs_generator_poly[33];
static int rs_generator_poly_log[33];

// Precomputed feedback multiplier LUT forced to 32-bit words for extreme register optimization
static uint32_t rs_feedback_lut32[256][8];

static uint8_t dual_basis_lut[256] = {
    0x00, 0x7b, 0xaf, 0xd4, 0x99, 0xe2, 0x36, 0x4d, 0xfa, 0x81, 0x55, 0x2e, 0x63, 0x18, 0xcc, 0xb7,
    0x86, 0xfd, 0x29, 0x52, 0x1f, 0x64, 0xb0, 0xcb, 0x7c, 0x07, 0xd3, 0xa8, 0xe5, 0x9e, 0x4a, 0x31,
    0xec, 0x97, 0x43, 0x38, 0x75, 0x0e, 0xda, 0xa1, 0x16, 0x6d, 0xb9, 0xc2, 0x8f, 0xf4, 0x20, 0x5b,
    0x6a, 0x11, 0xc5, 0xbe, 0xf3, 0x88, 0x5c, 0x27, 0x90, 0xeb, 0x3f, 0x44, 0x09, 0x72, 0xa6, 0xdd,
    0xef, 0x94, 0x40, 0x3b, 0x76, 0x0d, 0xd9, 0xa2, 0x15, 0x6e, 0xba, 0xc1, 0x8c, 0xf7, 0x23, 0x58,
    0x69, 0x12, 0xc6, 0xbd, 0xf0, 0x8b, 0x5f, 0x24, 0x93, 0xe8, 0x3c, 0x47, 0x0a, 0x71, 0xa5, 0xde,
    0x03, 0x78, 0xac, 0xd7, 0x9a, 0xe1, 0x35, 0x4e, 0xf9, 0x82, 0x56, 0x2d, 0x60, 0x1b, 0xcf, 0xb4,
    0x85, 0xfe, 0x2a, 0x51, 0x1c, 0x67, 0xb3, 0xc8, 0x7f, 0x04, 0xd0, 0xab, 0xe6, 0x9d, 0x49, 0x32,
    0x8d, 0xf6, 0x22, 0x59, 0x14, 0x6f, 0xbb, 0xc0, 0x77, 0x0c, 0xd8, 0xa3, 0xee, 0x95, 0x41, 0x3a,
    0x0b, 0x70, 0xa4, 0xdf, 0x92, 0xe9, 0x3d, 0x46, 0xf1, 0x8a, 0x5e, 0x25, 0x68, 0x13, 0xc7, 0xbc,
    0x61, 0x1a, 0xce, 0xb5, 0xf8, 0x83, 0x57, 0x2c, 0x9b, 0xe0, 0x34, 0x4f, 0x02, 0x79, 0xad, 0xd6,
    0xe7, 0x9c, 0x48, 0x33, 0x7e, 0x05, 0xd1, 0xaa, 0x1d, 0x66, 0xb2, 0xc9, 0x84, 0xff, 0x2b, 0x50,
    0x62, 0x19, 0xcd, 0xb6, 0xfb, 0x80, 0x54, 0x2f, 0x98, 0xe3, 0x37, 0x4c, 0x01, 0x7a, 0xae, 0xd5,
    0xe4, 0x9f, 0x4b, 0x30, 0x7d, 0x06, 0xd2, 0xa9, 0x1e, 0x65, 0xb1, 0xca, 0x87, 0xfc, 0x28, 0x53,
    0x8e, 0xf5, 0x21, 0x5a, 0x17, 0x6c, 0xb8, 0xc3, 0x74, 0x0f, 0xdb, 0xa0, 0xed, 0x96, 0x42, 0x39,
    0x08, 0x73, 0xa7, 0xdc, 0x91, 0xea, 0x3e, 0x45, 0xf2, 0x89, 0x5d, 0x26, 0x6b, 0x10, 0xc4, 0xbf
};

static uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) return 0;
    return gf_exp[(int)gf_log[a] + (int)gf_log[b]];
}

// cppcheck-suppress unusedFunction
void rs_init() {
    // Generate GF(2^8) log and anti-log tables
    // CCSDS Field Polynomial: F(x) = x^8 + x^7 + x^2 + x + 1 -> 0x187
    gf_exp[0] = 1;
    gf_log[0] = 0;
    gf_log[1] = 0;
    uint16_t x = 1;
    for (int i = 1; i < 255; i++) {
        x <<= 1;
        if (x & 0x100) {
            x ^= 0x187;
        }
        gf_exp[i] = x;
        gf_log[x] = i;
    }
    gf_exp[255] = gf_exp[0];

    // Fill the rest of the expanded table to avoid bounds checks during addition
    for (int i = 256; i < 512; i++) {
        gf_exp[i] = gf_exp[i - 255];
    }

    // Generate CCSDS RS generator polynomial
    // g(x) = (x - a^112)(x - a^113)...(x - a^143)
    // Note: roots are a^(11*112), a^(11*113), ...
    rs_generator_poly[0] = 1;
    for (int i = 1; i <= 32; i++) {
        rs_generator_poly[i] = 0;
    }

    // Match libcorrect's exact '(112 + i) * 11' gap arithmetic 
    // which perfectly maps to the standard CCSDS 11 * j roots
    for (int i = 0; i < 32; i++) {
        uint8_t root = gf_exp[((112 + i) * 11) % 255];
        for (int j = i + 1; j > 0; j--) {
            rs_generator_poly[j] = rs_generator_poly[j - 1] ^ gf_mul(rs_generator_poly[j], root);
        }
        rs_generator_poly[0] = gf_mul(rs_generator_poly[0], root);
    }

    // Precompute logs of the generator polynomial
    for (int i = 0; i <= 32; i++) {
        rs_generator_poly_log[i] = (rs_generator_poly[i] == 0) ? -1 : gf_log[rs_generator_poly[i]];
    }

    // Precompute the entire 8 KB feedback multiplication table to eliminate branches and math in the encoder
    for (int fb = 0; fb < 256; fb++) {
        uint8_t temp_row[32];
        if (fb == 0) {
            for (int j = 0; j < 32; j++) temp_row[j] = 0;
        } else {
            int log_fb = gf_log[fb];
            for (int j = 0; j < 32; j++) {
                int poly_log = rs_generator_poly_log[31 - j];
                temp_row[j] = (poly_log != -1) ? gf_exp[poly_log + log_fb] : 0;
            }
        }
        memcpy(rs_feedback_lut32[fb], temp_row, 32);
    }
}

void rs_encode(const uint8_t* msg_base, uint8_t* parity, int offset, int stride) {
    const int K = 223;

    // Keep the entire 32-byte parity buffer strictly inside the CPU's high-speed hardware registers
    uint32_t w0 = 0, w1 = 0, w2 = 0, w3 = 0, w4 = 0, w5 = 0, w6 = 0, w7 = 0;
    const uint8_t* msg_ptr = msg_base + offset;

    for (int i = 0; i < K; i++) {
        // LSB of w0 is parity[0] in little-endian
        uint8_t feedback = (*msg_ptr) ^ (uint8_t)(w0 & 0xFF);
        msg_ptr += stride; // Instantly jump to the next interleaved byte
        const uint32_t* lut_row = rs_feedback_lut32[feedback];

        // Shift the 32-byte register array down by 1 byte via little-endian bitwise math
        uint32_t next_w0 = (w0 >> 8) | (w1 << 24);
        uint32_t next_w1 = (w1 >> 8) | (w2 << 24);
        uint32_t next_w2 = (w2 >> 8) | (w3 << 24);
        uint32_t next_w3 = (w3 >> 8) | (w4 << 24);
        uint32_t next_w4 = (w4 >> 8) | (w5 << 24);
        uint32_t next_w5 = (w5 >> 8) | (w6 << 24);
        uint32_t next_w6 = (w6 >> 8) | (w7 << 24);
        uint32_t next_w7 = (w7 >> 8);

        // XOR with the precomputed Galois field multiplication row
        w0 = next_w0 ^ lut_row[0];
        w1 = next_w1 ^ lut_row[1];
        w2 = next_w2 ^ lut_row[2];
        w3 = next_w3 ^ lut_row[3];
        w4 = next_w4 ^ lut_row[4];
        w5 = next_w5 ^ lut_row[5];
        w6 = next_w6 ^ lut_row[6];
        w7 = next_w7 ^ lut_row[7];
    }

    // Safely dump the CPU registers back to the output RAM buffer using an aligned block copy
    uint32_t final_w[8] = {w0, w1, w2, w3, w4, w5, w6, w7};
    memcpy(parity, final_w, 32);
}

void rs_apply_dual_basis(uint8_t* data, int len) {
    int len32 = len / 4;
    
    for (int i = 0; i < len32; i++) {
        uint32_t word;
        memcpy(&word, &data[i * 4], 4); // Protects against unaligned traps and strict-aliasing faults
        word = (dual_basis_lut[word & 0xFF]) |
               (dual_basis_lut[(word >> 8) & 0xFF] << 8) |
               (dual_basis_lut[(word >> 16) & 0xFF] << 16) |
               (dual_basis_lut[(word >> 24) & 0xFF] << 24);
        memcpy(&data[i * 4], &word, 4);
    }
    
    for (int i = len32 * 4; i < len; i++) {
        data[i] = dual_basis_lut[data[i]];
    }
}