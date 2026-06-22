#include "viterbi.h"

constexpr BPSKConvFastLut::BPSKConvFastLut()
    : byte_out{}, state_out{}, next_state{} {
  for (int b = 0; b < 256; b++) {
    uint16_t out_word = 0;
    uint8_t c = 0;
    for (int bit = 7; bit >= 0; bit--) {
      uint8_t input_bit = (b >> bit) & 0x01;
      c = (uint8_t)(((c >> 1) | (input_bit << 6)) & 0x7F);
      out_word = (out_word << 2) | ((bpsk_parity8(c & CONV_G1) << 1) |
                                    (bpsk_parity8(c & CONV_G2)));
    }
    byte_out[b] = out_word;
    next_state[b] =
        c >> 1; // Top 6 bits perfectly capture the state for the next byte
  }

  for (int s = 0; s < 64; s++) {
    uint16_t out_word = 0;
    uint8_t c = s << 1;
    for (int bit = 7; bit >= 0; bit--) {
      c = (uint8_t)((c >> 1) & 0x7F);
      out_word = (out_word << 2) |
                 ((bpsk_parity8(c & CONV_G1) << 1) | bpsk_parity8(c & CONV_G2));
    }
    state_out[s] = out_word;
  }
}

// Removed constexpr to force the table into RAM to prevent Flash XIP cache
// misses
BPSKConvFastLut CCSDS_CONV_FAST_LUT;

constexpr BPSKPunctureFastLut::BPSKPunctureFastLut() : table{} {
  for (int rate = 1; rate <= 4; rate++) {
    uint8_t p_period = PUNC_PERIOD[rate];
    uint8_t c1_mask = PUNC_C1[rate];
    uint8_t c2_mask = PUNC_C2[rate];

    for (int phase = 0; phase < p_period; phase++) {
      for (int val = 0; val < 256; val++) {
        uint8_t accum = 0;
        uint8_t count = 0;
        uint8_t current_phase = phase;

        for (int s = 3; s >= 0; s--) {
          uint8_t c1 = (val >> (s * 2 + 1)) & 1;
          uint8_t c2 = (val >> (s * 2)) & 1;
          if ((c1_mask >> current_phase) & 1) {
            accum = (accum << 1) | c1;
            count++;
          }
          if ((c2_mask >> current_phase) & 1) {
            accum = (accum << 1) | c2;
            count++;
          }

          current_phase++;
          if (current_phase >= p_period)
            current_phase = 0;
        }
        table[rate][phase][val] = accum | (count << 8) | (current_phase << 16);
      }
    }
  }
}

BPSKPunctureFastLut CCSDS_PUNC_FAST_LUT;
