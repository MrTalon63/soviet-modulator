#ifndef VITERBI_H
#define VITERBI_H

#include <stdint.h>

static constexpr uint8_t CONV_G1 = 0x79; // 1111001b, 171 octal
static constexpr uint8_t CONV_G2 = 0x5B; // 1011011b, 133 octal

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

  constexpr BPSKConvFastLut();
};

extern BPSKConvFastLut CCSDS_CONV_FAST_LUT;

static constexpr uint8_t PUNC_PERIOD[] = {1, 2, 3, 5, 7};
static constexpr uint8_t PUNC_C1[] = {0b1, 0b01, 0b101, 0b10101, 0b1010001};
static constexpr uint8_t PUNC_C2[] = {0b1, 0b11, 0b011, 0b01011, 0b0101111};

struct alignas(4) BPSKPunctureFastLut {
  // table[rate][phase][byte] = (nexmt_phase << 16) | (count << 8) | bits
  uint32_t table[5][7][256];

  constexpr BPSKPunctureFastLut();
};

extern BPSKPunctureFastLut CCSDS_PUNC_FAST_LUT;

#endif // VITERBI_H
