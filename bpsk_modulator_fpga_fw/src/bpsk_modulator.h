#ifndef BPSK_MODULATOR_H
#define BPSK_MODULATOR_H

#include "ldpc.h"
#include "pico/critical_section.h"
#include <Arduino.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/spi.h>
#include <hardware/timer.h>

#define MAX_FRAME_SIZE 2044
#define MSG_BUFFER_SIZE 512
#define SP_FIFO_SIZE                                                           \
  131072 // 128 KB (Supports Max 64KB Space Packets + USB Jitter buffer)

#include "bpsk_pio.h"
#include "ccsds.h"
#include "locked_queue.h"
#include "rand.h"
#include "rs.h"
#include "viterbi.h"

class BPSKModulator;
extern BPSKModulator *g_modulator_instance;

extern void bpsk_dma_isr();

class BPSKModulator {
private:
  PIO pio;
  uint sm;
  uint pio_offset_fpga = 0xFFFFFFFF;
  int dma_chan_0;
  int dma_chan_1;
  dma_channel_config c0;
  dma_channel_config c1;

  uint pin;
  uint32_t symbolrate_hz;
  uint16_t msg_len;
  bool msg_pending;
  volatile bool running;
  uint8_t rs_interleave;
  bool convolution_enabled;
  bool randomizer_enabled;
  uint8_t randomizer_poly;
  bool reed_solomon_enabled;
  bool fecf_enabled;
  bool ldpc_enabled;

  volatile uint8_t *sp_fifo;
  volatile uint32_t sp_fifo_head = 0;
  volatile uint32_t sp_fifo_tail = 0;

  uint32_t current_sp_size;
  uint32_t current_sp_offset;
  bool is_filler;
  uint8_t filler_header[6];
  uint16_t filler_seq;

  // VCDU (Virtual Channel Data Unit) state for AOS Level 0 Transfer Frames
  uint32_t vcdu_frame_count;       // Master frame counter (20 bits), reset on
                                   // transmitter restart
  uint32_t vcdu_vcid_counters[64]; // Per-VCID counter (24 bits each) for 64
                                   // possible VCIDs

  uint32_t tx_chunk_words_written = 0;
  uint32_t rrc_chunk_bytes_written = 0;

  struct alignas(4) PendingFrame {
    uint8_t data[MAX_FRAME_SIZE];
  };

  // Deep enough to absorb math delays, small enough to prevent Out-Of-Memory
  // crashes
  static constexpr int PENDING_FRAME_QUEUE_SIZE = 16;
  LockedQueue<PendingFrame, PENDING_FRAME_QUEUE_SIZE>
      pending_frame_queue; // Lock protects Core 0 Command injections

  static constexpr uint16_t MAX_DMA_WORDS = 4096;

  struct alignas(4) DmaChunk {
    volatile uint32_t words[MAX_DMA_WORDS];
    uint32_t
        length; // 32-bit perfectly aligns struct for ARM LDM/STM memory copies
    uint32_t is_user_data; // Tag to differentiate payload chunks from
                           // auto-generated OID chunks
  };

  alignas(4) uint8_t current_tx_frame[MAX_FRAME_SIZE];

  uint8_t conv_shift_reg;
  alignas(4) uint32_t tx_accum_data;
  uint8_t tx_accum_bits;
  uint8_t punc_phase;
  uint8_t conv_rate;
  alignas(4) uint8_t oid_frame[MAX_FRAME_SIZE];

  static constexpr int DMA_RING_SIZE = 8;
  DmaChunk dma_ring[DMA_RING_SIZE];
  volatile uint16_t dma_ring_head;
  volatile uint16_t dma_ring_tail;
  volatile uint32_t underflow_buf[MAX_DMA_WORDS];
  volatile uint32_t dma_underflows;

  // Heap-allocated buffers to prevent Core 1 Stack Overflows (4KB stack limit)
  alignas(4) uint8_t mpdu_payload_buf[MAX_FRAME_SIZE];
  alignas(4) uint8_t ldpc_codeword_buf[1020];

  // RRC/RC pulse shaping filter configuration
  bool rrc_enabled = true;
  float rrc_alpha = 0.35f;
  uint8_t rrc_filter_span = 8;          // N_sym (default 8 symbols)
  uint32_t rrc_min_dac_rate = 20000000; // default 20 MHz
  uint32_t rrc_fixed_L = 0;             // 0 = automatic based on min_dac_rate
  uint32_t rrc_L = 8;                   // Calculated oversampling factor L
  bool use_rrc = true; // true = RRC, false = RC (Raised Cosine)
  uint8_t *rrc_lut =
      nullptr; // Precomputed state LUT: size = (1 << rrc_filter_span) * rrc_L
  uint16_t rrc_history = 0; // N_sym-bit shifting history of symbols
  bool cic_bypass = false;  // false = RRC active (true = bypass RRC)

  __attribute__((always_inline)) static inline double
  get_filter_coeff(uint32_t m, uint32_t filter_len, double L_d, double alpha,
                   bool use_rrc) {
    double x = ((double)m - (double)(filter_len - 1) / 2.0) / L_d;
    double h_val = 0.0;
    double pi = 3.14159265358979323846;
    if (use_rrc) {
      if (x == 0.0) {
        h_val = 1.0 - alpha + (4.0 * alpha / pi);
      } else if (fabs(fabs(4.0 * alpha * x) - 1.0) < 1e-9) {
        h_val =
            (alpha / sqrt(2.0)) * ((1.0 + 2.0 / pi) * sin(pi / (4.0 * alpha)) +
                                   (1.0 - 2.0 / pi) * cos(pi / (4.0 * alpha)));
      } else {
        double num = sin(pi * x * (1.0 - alpha)) +
                     4.0 * alpha * x * cos(pi * x * (1.0 + alpha));
        double den = pi * x * (1.0 - (4.0 * alpha * x) * (4.0 * alpha * x));
        h_val = num / den;
      }
    } else {
      if (x == 0.0) {
        h_val = 1.0;
      } else if (fabs(fabs(2.0 * alpha * x) - 1.0) < 1e-9) {
        h_val = (alpha / 2.0) * sin(pi / (2.0 * alpha));
      } else {
        double num = sin(pi * x) * cos(pi * alpha * x);
        double den = pi * x * (1.0 - (2.0 * alpha * x) * (2.0 * alpha * x));
        h_val = num / den;
      }
    }
    // No extra window: truncated RRC (rectangular window) gives better ISI than
    // Hamming because Hamming reduces the tails that carry the Nyquist zero
    // crossings.
    return h_val;
  }

  void compute_rrc_lut() {
    if (rrc_lut != nullptr) {
      free(rrc_lut);
      rrc_lut = nullptr;
    }

    uint32_t rate = symbolrate_hz;
    if (rate == 0)
      rate = 1;

    if (rrc_fixed_L > 0) {
      rrc_L = rrc_fixed_L;
    } else {
      rrc_L = (rrc_min_dac_rate + rate - 1) / rate;
    }

    uint32_t num_states = 1 << rrc_filter_span;
    uint32_t max_L = 131072 / num_states;
    if (max_L > 4096)
      max_L = 4096;
    if (max_L < 4)
      max_L = 4;

    rrc_L = (rrc_L + 3) & ~3;

    if (rrc_L < 4)
      rrc_L = 4;
    if (rrc_L > max_L)
      rrc_L = max_L & ~3;

    uint32_t lut_size = num_states * rrc_L;

    rrc_lut = (uint8_t *)malloc(lut_size);
    if (rrc_lut == nullptr) {
      Serial.println("ERROR: Failed to allocate memory for RRC LUT");
      return;
    }

    uint32_t filter_len = rrc_filter_span * rrc_L + 1;
    double alpha = rrc_alpha;
    double L_d = (double)rrc_L;

    double y_max = 0.0;
    for (uint32_t i = 0; i < rrc_L; i++) {
      double sum = 0.0;
      for (uint32_t j = 0; j < rrc_filter_span; j++) {
        sum += fabs(
            get_filter_coeff(j * rrc_L + i, filter_len, L_d, alpha, use_rrc));
      }
      if (sum > y_max) {
        y_max = sum;
      }
    }

    double scale = (y_max > 1e-9) ? (127.5 / y_max) : 127.5;

    for (uint32_t i = 0; i < rrc_L; i++) {
      double coeff[16];
      for (uint32_t j = 0; j < rrc_filter_span; j++) {
        coeff[j] =
            get_filter_coeff(j * rrc_L + i, filter_len, L_d, alpha, use_rrc);
      }

      for (uint32_t state = 0; state < num_states; state++) {
        double val = 0.0;
        for (uint32_t j = 0; j < rrc_filter_span; j++) {
          int bit = (state >> j) & 1;
          double symbol = bit ? 1.0 : -1.0;
          val += symbol * coeff[j];
        }

        int dac_val = (int)round(128.0 + scale * val);
        if (dac_val < 0)
          dac_val = 0;
        if (dac_val > 255)
          dac_val = 255;

        uint8_t reversed_val = (uint8_t)dac_val;
        reversed_val =
            ((reversed_val & 0xF0) >> 4) | ((reversed_val & 0x0F) << 4);
        reversed_val =
            ((reversed_val & 0xCC) >> 2) | ((reversed_val & 0x33) << 2);
        reversed_val =
            ((reversed_val & 0xAA) >> 1) | ((reversed_val & 0x55) << 1);

        rrc_lut[state * rrc_L + i] = reversed_val;
      }
    }

    Serial.print("RRC/RC LUT precomputed: Alpha=");
    Serial.print(rrc_alpha);
    Serial.print(", Span=");
    Serial.print(rrc_filter_span);
    Serial.print(" symbols, Oversampling L=");
    Serial.print(rrc_L);
    Serial.print(", States=");
    Serial.print(num_states);
    Serial.print(", Size=");
    Serial.print(lut_size);
    Serial.println(" bytes");
  }

  volatile uint32_t tx_user_frames_generated = 0;
  volatile uint32_t tx_user_chunks_dma_cleared = 0;

  void flush_pending_frames() { pending_frame_queue.clear(); }

  void clear_baseband_state() {
    conv_shift_reg = 0;
    tx_accum_data = 0;
    tx_accum_bits = 0;
    punc_phase = 0;

    sp_fifo_tail = 0;
    sp_fifo_head = 0;

    current_sp_size = 0;
    current_sp_offset = 0;
    is_filler = false;
    filler_seq = 0;
    tx_user_frames_generated = 0;
    tx_user_chunks_dma_cleared = 0;
    dma_ring_head = 0;
    dma_ring_tail = 0;

    tx_chunk_words_written = 0;
    rrc_chunk_bytes_written = 0;

    rrc_history = 0;
    flush_pending_frames();
  }

public:
  bool rrc_fpga_auto =
      true; // true = automatic L and span selection for FPGA mode
  bool get_rrc_enabled() const { return rrc_enabled; }
  float get_rrc_alpha() const { return rrc_alpha; }
  uint8_t get_rrc_filter_span() const { return rrc_filter_span; }
  uint32_t get_rrc_min_dac_rate() const { return rrc_min_dac_rate; }
  uint32_t get_rrc_fixed_L() const { return rrc_fixed_L; }
  uint32_t get_rrc_L() const { return rrc_L; }
  bool get_use_rrc() const { return use_rrc; }
  bool is_running() const { return running; }
  bool get_cic_bypass() const { return cic_bypass; }
  void set_cic_bypass(bool bypass) {
    lock_config();
    cic_bypass = bypass;
    send_fpga_spi_config();
    unlock_config();
  }

  enum ModulationMode : uint8_t {
    MOD_BPSK     = 0x00, // bit0=0, bit1=0
    MOD_QPSK     = 0x01, // bit0=1 (use I+Q, no extra delay)
    MOD_OQPSK    = 0x05, // bit0=1 + bit2=1 (use I+Q, Q delayed by L/2)
    MOD_IF_BPSK  = 0x02, // bit1=1 (IF on, BPSK)
    MOD_IF_QPSK  = 0x03, // bit0=1 + bit1=1 (IF on, QPSK)
    MOD_IF_OQPSK = 0x07  // bit0+bit1+bit2=1 (IF on, OQPSK)
  };

  uint8_t mod_mode = MOD_BPSK;
  uint8_t if_phase_offset = 0; // 0,64,128,192 for 0/90/180/270 deg at 30MHz
  uint8_t if_amplitude = 15;   // 0=full, 15=off

  void set_modulation_mode(uint8_t mode) {
    lock_config();
    mod_mode = mode;
    send_fpga_spi_config();
    unlock_config();
  }

  void set_digital_if(uint8_t amplitude, uint8_t phase_offset) {
    lock_config();
    if_amplitude = amplitude & 0x0F;
    if_phase_offset = phase_offset;
    send_fpga_spi_config();
    unlock_config();
  }

  uint8_t get_modulation_mode() const { return mod_mode; }
  uint8_t get_if_amplitude() const { return if_amplitude; }
  uint8_t get_if_phase_offset() const { return if_phase_offset; }

  uint32_t choose_L(uint32_t symbolrate, uint32_t span) {
    Serial.printf("[DEBUG] choose_L(%lu, span=%lu)\n",
                  (unsigned long)symbolrate, (unsigned long)span);

    // The FPGA state machine requires exactly (span / 2) + 2 cycles to compute.
    // We add +1 cycle to guarantee the ST_IDLE transition clears safely.
    uint32_t required_cycles = (span / 2) + 3;
    uint32_t max_sample_rate = 90000000 / required_cycles;

    // 1. Try to find an exact divisor of 90 MHz
    for (int l_val = 128; l_val >= 4; l_val--) {
      uint32_t sym_L = symbolrate * l_val;
      if (sym_L <= max_sample_rate && 90000000 % sym_L == 0) {
        return l_val;
      }
    }

    // 2. If no exact divisor, find the largest safe L
    for (int l_val = 128; l_val >= 4; l_val--) {
      uint32_t sym_L = symbolrate * l_val;
      if (sym_L <= max_sample_rate) {
        return l_val;
      }
    }

    return 4; // Absolute minimum fallback
  }

  uint32_t choose_span(float alpha) {
    if (alpha <= 0.0f)
      return 40;
    uint32_t span = (uint32_t)ceil(10.0f / alpha);
    span = (span + 7) & ~7; // Round to next multiple of 8
    if (span < 16)
      span = 16;
    if (span > 40)
      span = 40;
    return span;
  }

  void send_fpga_spi_config() {
    if (rrc_fixed_L == 0)
      return; // FPGA mode disabled

    // 1. Calculate RRC/RC filter coefficients and scale them to signed 16-bit
    uint32_t filter_len = rrc_filter_span * rrc_fixed_L + 1;
    double alpha = rrc_alpha;
    double L_d = (double)rrc_fixed_L;
    bool use_rrc_filter = use_rrc;

    double h_max = 0.0;
    for (uint32_t m = 0; m < filter_len - 1; m++) {
      double coeff =
          get_filter_coeff(m, filter_len, L_d, alpha, use_rrc_filter);
      if (fabs(coeff) > h_max) {
        h_max = fabs(coeff);
      }
    }
    double scale = (h_max > 1e-9) ? (32767.0 / h_max) : 32767.0;

    double y_max = 0.0;
    for (uint32_t p = 0; p < rrc_fixed_L; p++) {
      double sum = 0.0;
      for (uint32_t k = 0; k < rrc_filter_span; k++) {
        sum += fabs(get_filter_coeff(k * rrc_fixed_L + p, filter_len, L_d,
                                     alpha, use_rrc_filter));
      }
      if (sum > y_max) {
        y_max = sum;
      }
    }
    double scaled_y_max = y_max * scale;

    // 2. Compute CIC/ZOH Shift
    double R = 90000000.0 / ((double)symbolrate_hz * (double)rrc_fixed_L);
    double CIC_Max = scaled_y_max; // Pure ZOH mode, CIC filter has been removed
    int shift = (int)ceil(log2(CIC_Max / 127.0));
    if (shift < 0)
      shift = 0;
    if (shift > 79)
      shift = 79;

    Serial.printf("[SPI_CFG] h_max = %.6f, scale = %.2f, scaled_y_max = %.2f, "
                  "R = %.2f, CIC_Max = %.2e, shift = %d\n",
                  h_max, scale, scaled_y_max, R, CIC_Max, shift);
    Serial.print("[SPI_CFG] First 10 RRC Coeffs: ");
    for (uint32_t m = 0; m < (filter_len < 10 ? filter_len : 10); m++) {
      double coeff =
          get_filter_coeff(m, filter_len, L_d, alpha, use_rrc_filter);
      int16_t scaled_coeff = (int16_t)round(coeff * scale);
      Serial.printf("%d ", scaled_coeff);
    }
    Serial.println();

    // Send Register 0x00 (L)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_L = (0x00 << 8) | (rrc_fixed_L & 0xFF);
    spi_write16_blocking(spi0, &reg_L, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Send Register 0x01 (Span)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_span = (0x01 << 8) | (rrc_filter_span & 0xFF);
    spi_write16_blocking(spi0, &reg_span, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Send Register 0x02 (CIC Shift)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_shift = (0x02 << 8) | (shift & 0xFF);
    spi_write16_blocking(spi0, &reg_shift, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Send Register 0x03 (Type)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_type = (0x03 << 8) | (use_rrc ? 1 : 0);
    spi_write16_blocking(spi0, &reg_type, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Send Register 0x04 (CIC Bypass)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_bypass = (0x04 << 8) | (cic_bypass ? 1 : 0);
    spi_write16_blocking(spi0, &reg_bypass, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    uint32_t phase_ticks_fpga =
        90000000 / ((uint32_t)symbolrate_hz * rrc_fixed_L);
    if (phase_ticks_fpga > 65535)
      phase_ticks_fpga = 65535;
    if (phase_ticks_fpga < 1)
      phase_ticks_fpga = 1;

    // Send Register 0x05 (phase_ticks LSB)
    uint16_t reg_pticks_lsb = (0x05 << 8) | (phase_ticks_fpga & 0xFF);
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_blocking(spi0, &reg_pticks_lsb, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Send Register 0x06 (phase_ticks MSB)
    uint16_t reg_pticks_msb = (0x06 << 8) | ((phase_ticks_fpga >> 8) & 0xFF);
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_blocking(spi0, &reg_pticks_msb, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // 3. Dynamic RRC Coefficient Stream to SPI (avoiding large static buffers)
    // Configure SPI to 8-bit format for the byte-accurate burst write
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    delayMicroseconds(5);

    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint8_t cmd_byte = 0x10;
    spi_write_blocking(spi0, &cmd_byte, 1);

    uint8_t chunk_buf[64]; // Fits 32 coefficients (2 bytes per coeff)

    for (uint32_t chunk = 0; chunk < 256; chunk++) {
      for (uint32_t i = 0; i < 32; i++) {
        uint32_t target_idx = chunk * 32 + i;

        // Inverse mapping to find physical k and p
        uint32_t k_mod_8 = target_idx & 7;
        uint32_t tmp = target_idx >> 3;
        uint32_t p = tmp & 127;
        uint32_t k_div_8 = tmp >> 7;
        uint32_t k = (k_div_8 << 3) | k_mod_8;

        int16_t scaled_coeff = 0;
        if (k < rrc_filter_span && p < rrc_fixed_L) {
          uint32_t m = k * rrc_fixed_L + p;
          double coeff =
              get_filter_coeff(m, filter_len, L_d, alpha, use_rrc_filter);
          scaled_coeff = (int16_t)round(coeff * scale);
        }

        chunk_buf[2 * i] = scaled_coeff & 0xFF;
        chunk_buf[2 * i + 1] = (scaled_coeff >> 8) & 0xFF;
      }
      spi_write_blocking(spi0, chunk_buf, 64);
    }

    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(5);

    // Restore SPI format to 16-bit
    spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    delayMicroseconds(10);

    // Read back and verify registers
    uint16_t req_L = (0x80 | 0x00) << 8;
    uint16_t req_span = (0x80 | 0x01) << 8;
    uint16_t req_shift = (0x80 | 0x02) << 8;
    uint16_t req_type = (0x80 | 0x03) << 8;
    uint16_t req_bypass = (0x80 | 0x04) << 8;
    uint16_t req_pticks_lsb = (0x80 | 0x05) << 8;
    uint16_t req_pticks_msb = (0x80 | 0x06) << 8;
    uint16_t resp_L = 0, resp_span = 0, resp_shift = 0, resp_type = 0,
             resp_bypass = 0, resp_pticks_lsb = 0, resp_pticks_msb = 0;

    // Read L
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_L, &resp_L, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Read Span
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_span, &resp_span, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Read Shift
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_shift, &resp_shift, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Read Type
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_type, &resp_type, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Read Bypass
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_bypass, &resp_bypass, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Read phase_ticks LSB
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_pticks_lsb, &resp_pticks_lsb, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // Read phase_ticks MSB
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    spi_write16_read16_blocking(spi0, &req_pticks_msb, &resp_pticks_msb, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    uint16_t resp_pticks =
        (resp_pticks_lsb & 0xFF) | ((resp_pticks_msb & 0xFF) << 8);

    Serial.printf("[SPI_DBG] Read-back: L=%d (exp %d), Span=%d (exp %d), "
                  "Shift=%d (exp %d), Type=%d (exp %d), Bypass=%d (exp %d), "
                  "phase_ticks=%d (exp %d)\n",
                  resp_L & 0xFF, rrc_fixed_L, resp_span & 0xFF, rrc_filter_span,
                  resp_shift & 0xFF, shift, resp_type & 0xFF, use_rrc ? 1 : 0,
                  resp_bypass & 0xFF, cic_bypass ? 1 : 0, resp_pticks,
                  (uint16_t)phase_ticks_fpga);

    Serial.print("FPGA SPI Configuration Sent: CIC Shift = ");
    Serial.print(shift);
    Serial.print(", L = ");
    Serial.print(rrc_fixed_L);
    Serial.print(", Span = ");
    Serial.println(rrc_filter_span);

    // 4. Send Modem Mode Register (0x07)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_mode = (0x07 << 8) | (mod_mode & 0xFF);
    spi_write16_blocking(spi0, &reg_mode, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // 5. Send IF Phase Offset Register (0x08)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_if_phase = (0x08 << 8) | (if_phase_offset & 0xFF);
    spi_write16_blocking(spi0, &reg_if_phase, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    // 6. Send IF Amplitude Register (0x09)
    gpio_put(PIN_SPI_CS, 0);
    delayMicroseconds(2);
    uint16_t reg_if_amp = (0x09 << 8) | (if_amplitude & 0x0F);
    spi_write16_blocking(spi0, &reg_if_amp, 1);
    gpio_put(PIN_SPI_CS, 1);
    delayMicroseconds(10);

    Serial.printf("[SPI_CFG] ModMode=0x%02X, IF Phase=%d, IF Amp=%d\n",
                  mod_mode, if_phase_offset, if_amplitude);
  }

  bool verify_fpga_spi_connection() {
    uint16_t req = (0x80 | 0x7F) << 8; // Bit 15 is 1 (Read), address 0x7F

    for (int mode = 0; mode < 4; mode++) {
      spi_cpol_t cpol = (mode & 2) ? SPI_CPOL_1 : SPI_CPOL_0;
      spi_cpha_t cpha = (mode & 1) ? SPI_CPHA_1 : SPI_CPHA_0;

      spi_set_format(spi0, 16, cpol, cpha, SPI_MSB_FIRST);
      delayMicroseconds(10);

      gpio_put(PIN_SPI_CS, 0);
      delayMicroseconds(2);
      uint16_t resp = 0;
      spi_write16_read16_blocking(spi0, &req, &resp, 1);
      gpio_put(PIN_SPI_CS, 1);

      Serial.print("[SPI] Mode ");
      Serial.print(mode);
      Serial.print(" (CPOL=");
      Serial.print((mode & 2) ? "1" : "0");
      Serial.print(", CPHA=");
      Serial.print((mode & 1) ? "1" : "0");
      Serial.print("): Response = 0x");
      Serial.println(resp, HEX);

      if ((resp & 0xFF) == 0xA5) {
        Serial.print("[SPI] Success! Found working SPI Mode ");
        Serial.println(mode);
        return true;
      }
    }

    // Restore default Mode 0 if none worked
    spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    return false;
  }

  void set_rrc_fixed_L(uint32_t L) {
    lock_config();
    bool old_fpga = (rrc_fixed_L > 0);
    bool new_fpga = (L > 0);
    rrc_fixed_L = L;
    if (rrc_fixed_L > 0) {
      rrc_L = rrc_fixed_L;
    }

    if (old_fpga != new_fpga) {
      bool was_running = running;
      if (was_running) {
        stop();
      }
      init_pio(pin);
      if (was_running) {
        start();
      }
    }

    if (rrc_fixed_L == 0 && rrc_enabled) {
      compute_rrc_lut();
    }
    unlock_config();
  }

  void update_rrc_config(bool enabled, float alpha, uint8_t span,
                         uint32_t min_dac_rate, bool use_rrc_filter) {
    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    rrc_enabled = enabled;
    rrc_alpha = alpha;
    rrc_filter_span = span;
    rrc_min_dac_rate = min_dac_rate;
    use_rrc = use_rrc_filter;

    if (rrc_fpga_auto) {
      rrc_filter_span = choose_span(rrc_alpha); // Calculate span FIRST
      rrc_fixed_L = choose_L(symbolrate_hz, rrc_filter_span);
    }

    if (rrc_enabled && rrc_fixed_L == 0) {
      compute_rrc_lut();
    }

    if (rrc_fixed_L > 0) {
      rrc_L = rrc_fixed_L;
      send_fpga_spi_config();
    }

    // Re-apply symbol rate to update PIO clock divisor based on new L
    uint32_t old_rate = symbolrate_hz;
    symbolrate_hz = 0; // force update
    set_symbolrate(old_rate);

    clear_baseband_state();

    if (was_running)
      start();
    unlock_config();
  }

  alignas(4) uint8_t frame[MAX_FRAME_SIZE];
  volatile bool config_lock;
  volatile bool core1_processing;
  volatile uint32_t perf_mpdu_us = 0;
  volatile uint32_t perf_bb_us = 0;

  void lock_config() {
    config_lock = true;
    __dmb(); // Force memory controller to instantly broadcast lock state to
             // Core 1
    while (core1_processing) {
      __asm volatile("nop");
    }
  }

  uint8_t get_dma_chunks_count() const {
    uint16_t h = dma_ring_head;
    uint16_t t = dma_ring_tail;
    return (h >= t) ? (h - t) : (DMA_RING_SIZE - t + h);
  }

  uint32_t get_dma_underflows() const { return dma_underflows; }

  void unlock_config() {
    __dmb();
    config_lock = false;
    __dmb();
  }

  void enable_dma_irq_core1() {
    irq_set_exclusive_handler(DMA_IRQ_1, bpsk_dma_isr);
    irq_set_priority(DMA_IRQ_1, 0); // 0 = Highest hardware priority
    irq_set_enabled(DMA_IRQ_1, true);
  }

  __attribute__((always_inline)) inline uint16_t get_frame_size() const {
    if (ldpc_enabled)
      return CADU_ASM_SIZE +
             1020; // 4-byte CSM + 1020-byte LDPC codeword = 1024 bytes
    return ASM_SIZE + 255 * rs_interleave;
  }

  __attribute__((always_inline)) inline uint16_t get_rs_input_size() const {
    return 223 * rs_interleave;
  }

  __attribute__((always_inline)) inline uint16_t get_rs_output_size() const {
    return 255 * rs_interleave;
  }

  __attribute__((always_inline)) inline const uint8_t *
  get_randomizer_seq() const {
    return (randomizer_poly == 17) ? CCSDS_RANDOMIZER17_FAST_LUT.seq
                                   : CCSDS_RANDOMIZER8_FAST_LUT.seq;
  }

  BPSKModulator(uint pin, volatile uint8_t *fifo_buf, uint32_t rate = 1200)
      : pin(pin), sp_fifo(fifo_buf), symbolrate_hz(0), msg_len(0),
        msg_pending(false), running(false), rs_interleave(4),
        convolution_enabled(true), randomizer_enabled(true), randomizer_poly(8),
        reed_solomon_enabled(true), fecf_enabled(true), ldpc_enabled(false),
        conv_shift_reg(0), tx_accum_data(0), tx_accum_bits(0), punc_phase(0),
        conv_rate(0), vcdu_frame_count(0), config_lock(false),
        core1_processing(false), current_sp_size(0), current_sp_offset(0),
        is_filler(false), filler_seq(0), dma_ring_head(0), dma_ring_tail(0),
        dma_underflows(0), cic_bypass(false) {
    dma_chan_0 = -1;
    dma_chan_1 = -1;
    sm = 0xFFFFFFFF;
    for (int i = 0; i < MAX_DMA_WORDS; i++) {
      underflow_buf[i] = 0xFF00FF00;
    }
    g_modulator_instance = this;
    memset(vcdu_vcid_counters, 0, sizeof(vcdu_vcid_counters));

    // Initialize SPI at 100 kHz
    spi_init(spi0, 100000);
    spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);

    // Initialize CS pin
    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1);

    build_oid_frame();
    init_frame();
    init_pio(pin);
    set_symbolrate(rate);
  }

  ~BPSKModulator() {
    lock_config();
    stop();
    if (rrc_lut != nullptr) {
      free(rrc_lut);
      rrc_lut = nullptr;
    }
    unlock_config();
  }

  uint32_t get_fifo_head() const { return sp_fifo_head; }

  uint32_t get_sp_fifo_count() const {
    return (sp_fifo_head - sp_fifo_tail) & (SP_FIFO_SIZE - 1);
  }

  uint32_t get_sp_fifo_free() const {
    return SP_FIFO_SIZE - 1 - get_sp_fifo_count();
  }

  bool push_sp_byte(uint8_t b) {
    if (get_sp_fifo_free() == 0)
      return false;

    uint32_t head = sp_fifo_head;
    sp_fifo[head] = b;
    __dmb(); // Force byte to physical RAM before head increments to prevent
             // Core 1 from reading garbage
    sp_fifo_head = (head + 1) & (SP_FIFO_SIZE - 1);
    return true;
  }

  uint8_t pop_sp_byte() {
    if (get_sp_fifo_count() == 0)
      return 0;
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
    sp_fifo_tail = sp_fifo_head = 0;
    current_sp_size = 0;
    current_sp_offset = 0;
    is_filler = false;
    unlock_config();
  }

  void prepare_tx_stream() {
    // DMA prepares chunks synchronously via ISR. Empty wrapper preserves
    // compatibility.
  }

  __attribute__((section(".time_critical.build_vcdu_header"))) void
  build_vcdu_header(uint8_t *frame, uint16_t frame_offset, uint8_t vcid,
                    uint32_t frame_count) {
    // Build VCDU (Virtual Channel Data Unit) Transfer Frame Header (6 bytes)
    // per CCSDS 732.0-B-5 frame_offset: offset into frame buffer (typically
    // ASM_SIZE = 4 for RS blocks)

    uint8_t *header = frame + frame_offset;

    // Byte 0: TFVN (2 bits) = 01 | SCID LSB (6 bits, bits 7-2 of 10-bit SCID)
    header[0] =
        (VCDU_TRANSFER_FRAME_VERSION << 6) | ((VCDU_SPACECRAFT_ID >> 2) & 0x3F);

    // Byte 1: SCID LSB (2 bits, bits 1-0 of 10-bit SCID) | VCID (6 bits)
    header[1] = ((VCDU_SPACECRAFT_ID & 0x03) << 6) | (vcid & 0x3F);

    // Bytes 2-4: Frame Count (24 bits, 3 octets)
    header[2] = (frame_count >> 16) & 0xFF; // Bits 23-16
    header[3] = (frame_count >> 8) & 0xFF;  // Bits 15-8
    header[4] = frame_count & 0xFF;         // Bits 7-0

    // Byte 5: Replay Flag (1 bit) | Cycle Use Flag (1 bit) | SCID MSB (2 bits,
    // bits 9-8) | Frame Count Cycle (4 bits)
    header[5] = (VCDU_REPLAY_FLAG << 7) | (VCDU_CYCLE_USE_FLAG << 6) |
                ((VCDU_SPACECRAFT_ID >> 8) & 0x03) << 4 |
                (VCDU_FRAME_COUNT_CYCLE & 0x0F);
  }

  void reset_vcdu_counters() {
    vcdu_frame_count = 0;
    memset(vcdu_vcid_counters, 0, sizeof(vcdu_vcid_counters));
  }

  __attribute__((section(".time_critical.build_mpdu_header"))) void
  build_mpdu_header(uint8_t *frame, uint16_t frame_offset,
                    uint16_t first_header_ptr) {
    // Build MPDU (Multiplexing Protocol Data Unit) Header (2 bytes) per CCSDS
    // 732.0-B-5 Bits 0-15: First Header Pointer (16-bit value, position of
    // first packet start in MPDU Packet Zone) frame_offset: offset into frame
    // buffer

    uint8_t *header = frame + frame_offset;

    // MPDU Byte 0: FHP bits 8-15 (upper byte)
    header[0] = (first_header_ptr >> 8) & 0xFF;

    // MPDU Byte 1: FHP bits 0-7 (lower byte)
    header[1] = first_header_ptr & 0xFF;
  }

  void randomize_buffer_data(uint8_t *buffer, uint16_t start_index) {
    const uint8_t *seq = get_randomizer_seq();
    for (uint16_t i = start_index; i < get_frame_size(); i++) {
      buffer[i] ^= seq[i - start_index];
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
    uint16_t length_to_crc;
    if (ldpc_enabled) {
      length_to_crc = 892 - (fecf_enabled ? 2 : 0);
    } else {
      length_to_crc =
          (reed_solomon_enabled ? get_rs_input_size() : get_rs_output_size()) -
          (fecf_enabled ? 2 : 0);
    }
    uint16_t oid_payload_size = length_to_crc - VCDU_HEADER_SIZE;
    for (uint16_t i = 0; i < oid_payload_size; i++) {
      oid_frame[ASM_SIZE + VCDU_HEADER_SIZE + i] = OID_FRAME_PATTERN.pattern[i];
    }

    if (fecf_enabled) {
      uint16_t crc = calculate_fecf(oid_frame + ASM_SIZE, length_to_crc);
      oid_frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
      oid_frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
    }

    // Apply Error Correction & Randomization
    if (ldpc_enabled) {
      // Per CCSDS 131.0-B-5 §9.5.3: The Codeword Sync Marker (CSM) is NOT
      // encoded. Because we directly map 1 Transfer Frame to 1 LDPC info block
      // (no slicing), we use the CSM (which is identical to the ASM: 1A CF FC
      // 1D) as the unencoded header. Per CCSDS 131.0-B-5 §8.3: Randomization
      // happens AFTER encoding Input: VCDU (6) + payload (884) = 892 bytes

      ldpc_78_encode(oid_frame + ASM_SIZE, 892, ldpc_codeword_buf);

      // Apply randomization to the codeword output
      if (randomizer_enabled) {
        uint32_t *cw_words = (uint32_t *)(ldpc_codeword_buf);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = 1020 / 4;
        for (uint16_t i = 0; i < words; i++) {
          cw_words[i] ^= lut_words[i];
        }
      }

      // Copy codeword to frame, leaving ASM intact
      memcpy(oid_frame + CADU_ASM_SIZE, ldpc_codeword_buf, 1020);
    } else if (reed_solomon_enabled) {
      const int I = rs_interleave;
      static const int RS_K = 223;
      static const int RS_N = 255;
      for (int i = 0; i < I; i++) {
        uint8_t msg[RS_K];
        uint8_t parity[RS_N - RS_K];
        for (int c = 0; c < RS_K; c++) {
          msg[c] = oid_frame[ASM_SIZE + c * I + i];
        }
        rs_encode(msg, parity);
        for (int c = 0; c < (RS_N - RS_K); c++) {
          oid_frame[ASM_SIZE + RS_K * I + c * I + i] = parity[c];
        }
      }

      if (randomizer_enabled) {
        uint32_t *frame_words = (uint32_t *)(oid_frame + ASM_SIZE);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = get_rs_output_size() / 4;
        for (uint16_t i = 0; i < words; i++) {
          frame_words[i] ^= lut_words[i];
        }
        const uint8_t *seq = get_randomizer_seq();
        for (uint16_t i = words * 4; i < get_rs_output_size(); i++) {
          oid_frame[ASM_SIZE + i] ^= seq[i];
        }
      }
    } else {
      if (randomizer_enabled) {
        uint32_t *frame_words = (uint32_t *)(oid_frame + ASM_SIZE);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = get_rs_output_size() / 4;
        for (uint16_t i = 0; i < words; i++) {
          frame_words[i] ^= lut_words[i];
        }
        const uint8_t *seq = get_randomizer_seq();
        for (uint16_t i = words * 4; i < get_rs_output_size(); i++) {
          oid_frame[ASM_SIZE + i] ^= seq[i];
        }
      }
    }
  }

  __attribute__((section(".time_critical.init_idle_data_frame"))) void
  init_idle_data_frame(uint8_t vcid = VCDU_DEFAULT_VCID) {
    // Build an MPDU Idle Data frame per CCSDS 732.0-B-5 Section 4.1.4.1
    // These frames contain VCDU/MPDU headers with idle data in the Packet Zone
    // Used to maintain synchronous transmission when no real packets are
    // available

    // ASM at bytes 0-3
    frame[0] = 0x1A;
    frame[1] = 0xCF;
    frame[2] = 0xFC;
    frame[3] = 0x1D;

    // VCDU header at bytes 4-9 with specified VCID
    build_vcdu_header(frame, VCDU_OFFSET, vcid, vcdu_vcid_counters[vcid]);
    vcdu_vcid_counters[vcid & 0x3F]++; // Increment per-VCID counter
    vcdu_frame_count++;                // Increment master frame counter

    uint16_t length_to_crc;
    if (ldpc_enabled) {
      length_to_crc = 892 - (fecf_enabled ? 2 : 0);
    } else {
      length_to_crc =
          (reed_solomon_enabled ? get_rs_input_size() : get_rs_output_size()) -
          (fecf_enabled ? 2 : 0);
    }

    if (vcid == 0x3F) {
      // OID Frame: No MPDU header
      uint16_t payload_size = length_to_crc - VCDU_HEADER_SIZE;
      for (uint16_t i = 0; i < payload_size; i++) {
        frame[VCDU_OFFSET + VCDU_HEADER_SIZE + i] =
            OID_FRAME_PATTERN.pattern[i];
      }
    } else {
      // MPDU Idle Frame
      build_mpdu_header(frame, MPDU_OFFSET, MPDU_IDLE_DATA);
      uint16_t payload_size =
          length_to_crc - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;
      for (uint16_t i = 0; i < payload_size; i++) {
        frame[PAYLOAD_OFFSET + i] = 0xFF; // All-ones idle pattern
      }
    }

    if (fecf_enabled) {
      uint16_t crc = calculate_fecf(frame + ASM_SIZE, length_to_crc);
      frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
      frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
    }

    // Apply Error Correction & Randomization
    if (ldpc_enabled) {
      // NOTE: Virtual fill (18 leading zeros) is handled internally by LDPC
      // encoder per CCSDS 131.0-B-5 §7.3.4 steps 1-2. Do NOT zero-out the frame
      // here. The encoder expects the raw 892-byte block and prefixes virtual
      // fill internally.

      // Per CCSDS 131.0-B-5 §8.3: Randomization happens AFTER encoding, not
      // before Per CCSDS 131.0-B-5 §9.5.3: The Codeword Sync Marker (CSM) is
      // NOT encoded. Because we directly map 1 Transfer Frame to 1 LDPC info
      // block (no slicing), we use the CSM (which is identical to the ASM: 1A
      // CF FC 1D) as the unencoded header. Input: VCDU (6) + MPDU (2) + payload
      // (884) = 892 bytes

      ldpc_78_encode(frame + ASM_SIZE, 892, ldpc_codeword_buf);

      // Apply randomization to the codeword output
      if (randomizer_enabled) {
        uint32_t *cw_words = (uint32_t *)(ldpc_codeword_buf);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = 1020 / 4;
        for (uint16_t i = 0; i < words; i++) {
          cw_words[i] ^= lut_words[i];
        }
      }

      // Copy codeword to frame, leaving ASM intact
      memcpy(frame + CADU_ASM_SIZE, ldpc_codeword_buf, 1020);
    } else if (reed_solomon_enabled) {
      const int I = rs_interleave;
      static const int RS_K = 223;
      static const int RS_N = 255;
      for (int i = 0; i < I; i++) {
        uint8_t msg[RS_K];
        uint8_t parity[RS_N - RS_K];
        for (int c = 0; c < RS_K; c++) {
          msg[c] = frame[ASM_SIZE + c * I + i];
        }
        rs_encode(msg, parity);
        for (int c = 0; c < (RS_N - RS_K); c++) {
          frame[ASM_SIZE + RS_K * I + c * I + i] = parity[c];
        }
      }

      if (randomizer_enabled) {
        uint32_t *frame_words = (uint32_t *)(frame + ASM_SIZE);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = get_rs_output_size() / 4;
        for (uint16_t i = 0; i < words; i++) {
          frame_words[i] ^= lut_words[i];
        }
        const uint8_t *seq = get_randomizer_seq();
        for (uint16_t i = words * 4; i < get_rs_output_size(); i++) {
          frame[ASM_SIZE + i] ^= seq[i];
        }
      }
    } else {
      // Non-RS mode: just copy the frame with randomization if enabled
      if (randomizer_enabled) {
        uint32_t *frame_words = (uint32_t *)(frame + ASM_SIZE);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = get_rs_output_size() / 4;
        for (uint16_t i = 0; i < words; i++) {
          frame_words[i] ^= lut_words[i];
        }
        const uint8_t *seq = get_randomizer_seq();
        for (uint16_t i = words * 4; i < get_rs_output_size(); i++) {
          frame[ASM_SIZE + i] ^= seq[i];
        }
      }
    }
  }

  __attribute__((always_inline)) inline uint8_t
  get_pending_frames_count() const {
    return pending_frame_queue.get_level();
  }

  __attribute__((always_inline)) inline bool
  push_pending_frame(const uint8_t *frame_data, bool is_user_frame = true) {
    if (pending_frame_queue.try_add(
            reinterpret_cast<const PendingFrame *>(frame_data))) {
      // Only increment the user frame counter if the frame is not an OID frame
      // (VCID 63)
      uint8_t vcid_byte = frame_data[5];
      if (randomizer_enabled) {
        vcid_byte ^= get_randomizer_seq()[1]; // Revert the XOR for byte 5
      }
      bool is_oid_frame = (vcid_byte & 0x3F) == 0x3F;
      if (!is_oid_frame) {
        tx_user_frames_generated++;
      }
      return true;
    }
    return false;
  }

  __attribute__((always_inline)) inline bool
  pop_pending_frame(uint8_t *frame_data) {
    return pending_frame_queue.try_remove(
        reinterpret_cast<PendingFrame *>(frame_data));
  }

  uint8_t *get_oid_frame() { return oid_frame; }

  uint32_t get_active_user_chunks() const {
    return tx_user_frames_generated - tx_user_chunks_dma_cleared;
  }

  bool queue_oid_frame() { return push_pending_frame(oid_frame); }

  bool queue_idle_data_frame(uint8_t vcid = VCDU_DEFAULT_VCID) {
    // Build an idle data MPDU frame with specified VCID and queue it
    lock_config();
    init_idle_data_frame(vcid);
    bool result = push_pending_frame(frame);
    unlock_config();
    return result;
  }

  __attribute__((always_inline)) inline bool can_queue_binary_frame() const {
    uint8_t count = get_pending_frames_count();
    if (reed_solomon_enabled) {
      return (count <= (PENDING_FRAME_QUEUE_SIZE - 2));
    }
    return (count < PENDING_FRAME_QUEUE_SIZE - 1);
  }

  __attribute__((section(".time_critical.has_data_to_send"))) bool
  has_data_to_send() const {
    uint32_t fifo_count = get_sp_fifo_count();

    if (current_sp_offset < current_sp_size) {
      if (is_filler)
        return true; // We can always finish auto-generated filler packets
                     // instantly!

      // Actively mid-packet! We MUST ensure the FIFO has enough bytes to safely
      // continue without underflowing mid-VCDU and injecting 0x00s!
      uint16_t length_to_crc =
          ldpc_enabled ? (892 - (fecf_enabled ? 2 : 0))
                       : ((reed_solomon_enabled ? get_rs_input_size()
                                                : get_rs_output_size()) -
                          (fecf_enabled ? 2 : 0));
      uint16_t payload_size =
          length_to_crc - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;

      uint32_t bytes_needed = current_sp_size - current_sp_offset;
      if (bytes_needed > payload_size)
        bytes_needed = payload_size;

      if (fifo_count >= bytes_needed)
        return true;

      return false; // Starved mid-packet! Emit an OID frame and wait for USB
                    // data.
    }

    if (get_sp_fifo_free() == 0)
      return true; // FIFO is 100% full (emergency flush to unblock USB)

    if (fifo_count >= 6) {
      // Only check the 3-bit Version Number (000) to validate the Space Packet
      // header. Do NOT enforce Sequence Flags (0xC0), as large video frames
      // will be heavily segmented! Do NOT enforce Type/SecHeader (0x18), as
      // tools may safely inject timestamps!
      if ((peek_sp_byte(0) & 0xE0) != 0x00) {
        return true;
      }

      uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
      uint32_t total_len = pdl + 7;

      if (total_len >= SP_FIFO_SIZE) {
        return true; // Corrupted length! Let the generator pop and discard it.
      }

      if (fifo_count >= total_len)
        return true;
    }
    return false; // Not enough data, safe to bypass frame generator and send
                  // pre-calculated OID Idle Frames
  }

  __attribute__((section(".time_critical.generate_and_queue_mpdu"))) bool
  generate_and_queue_mpdu(uint8_t vcid = VCDU_DEFAULT_VCID) {
    if (!can_queue_binary_frame())
      return false;

    uint16_t fhp = MPDU_NO_START_PACKET;
    uint16_t length_to_crc;
    if (ldpc_enabled) {
      length_to_crc = 892 - (fecf_enabled ? 2 : 0);
    } else {
      length_to_crc =
          (reed_solomon_enabled ? get_rs_input_size() : get_rs_output_size()) -
          (fecf_enabled ? 2 : 0);
    }
    uint16_t payload_size = length_to_crc - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;

    for (uint16_t i = 0; i < payload_size; i++) {
      // If the current packet is finished, try to find a new one in the FIFO.
      if (current_sp_offset >= current_sp_size) {
        current_sp_offset = 0;
        current_sp_size = 0;
        is_filler = false;

        while (get_sp_fifo_count() >= 6) {
          if ((peek_sp_byte(0) & 0xE0) != 0x00) {
            pop_sp_byte(); // Corrupted byte, drop and resync
            continue;
          }
          uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
          uint32_t total_len = pdl + 7;
          if (total_len >= SP_FIFO_SIZE) {
            pop_sp_byte(); // Corrupted length, drop and resync
            continue;
          }

          if (get_sp_fifo_count() >= total_len) {
            // Found a complete packet!
            current_sp_size = total_len;
            if (fhp == MPDU_NO_START_PACKET) {
              fhp = i; // This is the first packet to start in this frame
            }
          }
          break; // Exit the while loop whether we found a packet or not
        }

        if (current_sp_size == 0) {
          // No user data available. Generate an APID 2047 (Idle Packet) to pad
          // the VCDU.
          uint16_t remaining = payload_size - i;
          // Space packet length must be at least 7 bytes (6 byte header + 1
          // byte payload)
          uint16_t filler_size = (remaining >= 7) ? remaining : 7;

          current_sp_size = filler_size;
          is_filler = true;

          if (fhp == MPDU_NO_START_PACKET) {
            fhp = i; // The filler packet starts here
          }

          filler_header[0] = 0x07; // APID 2047 (Idle Packet)
          filler_header[1] = 0xFF;
          filler_header[2] = 0xC0 | ((filler_seq >> 8) &
                                     0x3F); // Sequence flags = 11 (unsegmented)
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
          // APID 2047 Idle Data is set to all ones
          mpdu_payload_buf[i] = 0xFF;
        }
        current_sp_offset++;
      } else {
        mpdu_payload_buf[i] = pop_sp_byte();
        current_sp_offset++;
      }
    }

    // Now assemble the frame
    frame[0] = 0x1A;
    frame[1] = 0xCF;
    frame[2] = 0xFC;
    frame[3] = 0x1D;

    build_vcdu_header(frame, ASM_SIZE, vcid, vcdu_vcid_counters[vcid]);
    vcdu_vcid_counters[vcid & 0x3F]++;
    vcdu_frame_count++;

    build_mpdu_header(frame, ASM_SIZE + VCDU_HEADER_SIZE, fhp);
    memcpy(frame + PAYLOAD_OFFSET, mpdu_payload_buf, payload_size);

    if (fecf_enabled) {
      uint16_t crc = calculate_fecf(frame + ASM_SIZE, length_to_crc);
      frame[ASM_SIZE + length_to_crc] = (crc >> 8) & 0xFF;
      frame[ASM_SIZE + length_to_crc + 1] = crc & 0xFF;
    }

    if (ldpc_enabled) {
      // Per CCSDS 131.0-B-5 §9.5.3: The Codeword Sync Marker (CSM) is NOT
      // encoded. Because we directly map 1 Transfer Frame to 1 LDPC info block
      // (no slicing), we use the CSM (which is identical to the ASM: 1A CF FC
      // 1D) as the unencoded header. Per CCSDS 131.0-B-5 §8.3: Randomization
      // happens AFTER encoding, not before Input: VCDU (6) + MPDU (2) + payload
      // (884) = 892 bytes

      ldpc_78_encode(frame + ASM_SIZE, 892, ldpc_codeword_buf);

      // Apply randomization to the codeword output
      if (randomizer_enabled) {
        uint32_t *cw_words = (uint32_t *)(ldpc_codeword_buf);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = 1020 / 4;
        for (uint16_t i = 0; i < words; i++) {
          cw_words[i] ^= lut_words[i];
        }
      }

      // Copy codeword to frame, leaving ASM intact
      memcpy(frame + CADU_ASM_SIZE, ldpc_codeword_buf, 1020);
    } else if (reed_solomon_enabled) {
      const int I = rs_interleave;
      static const int RS_K = 223;
      static const int RS_N = 255;

      for (int i = 0; i < I; i++) {
        uint8_t msg[RS_K];
        uint8_t parity[RS_N - RS_K];
        for (int c = 0; c < RS_K; c++) {
          msg[c] = frame[ASM_SIZE + c * I + i];
        }
        rs_encode(msg, parity);
        for (int c = 0; c < (RS_N - RS_K); c++) {
          frame[ASM_SIZE + RS_K * I + c * I + i] = parity[c];
        }
      }

      // Legacy Randomizer placement for RS frames
      if (randomizer_enabled) {
        uint32_t *frame_words = (uint32_t *)(frame + ASM_SIZE);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = get_rs_output_size() / 4;
        for (uint16_t i = 0; i < words; i++) {
          frame_words[i] ^= lut_words[i];
        }
        const uint8_t *seq = get_randomizer_seq();
        for (uint16_t i = words * 4; i < get_rs_output_size(); i++) {
          frame[ASM_SIZE + i] ^= seq[i];
        }
      }
    } else {
      // Non-RS mode: just copy the frame with randomization if enabled
      if (randomizer_enabled) {
        uint32_t *frame_words = (uint32_t *)(frame + ASM_SIZE);
        const uint32_t *lut_words = (const uint32_t *)(get_randomizer_seq());
        uint16_t words = get_rs_output_size() / 4;
        for (uint16_t i = 0; i < words; i++) {
          frame_words[i] ^= lut_words[i];
        }
        const uint8_t *seq = get_randomizer_seq();
        for (uint16_t i = words * 4; i < get_rs_output_size(); i++) {
          frame[ASM_SIZE + i] ^= seq[i];
        }
      }
    }

    return push_pending_frame(frame);
  }

  __attribute__((section(".time_critical.generate_and_queue_idle_frame"))) bool
  generate_and_queue_idle_frame(uint8_t vcid = VCDU_DEFAULT_VCID) {
    if (!can_queue_binary_frame())
      return false;
    init_idle_data_frame(vcid);
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
    if (was_running)
      stop();

    memset(frame, 0, MAX_FRAME_SIZE);
    clear_baseband_state();
    prepare_tx_stream();

    if (was_running)
      start();
    unlock_config();
  }

  void set_rs_interleave(uint8_t i) {
    if (i != 1 && i != 2 && i != 4 && i != 5 && i != 8)
      return;
    if (rs_interleave == i)
      return;

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    rs_interleave = i;
    clear_baseband_state();
    build_oid_frame();

    if (was_running)
      start();
    unlock_config();
  }

  uint8_t get_rs_interleave() const { return rs_interleave; }

  void set_convolutional_encoding(bool enabled) {
    if (convolution_enabled == enabled)
      return; // Do not interrupt running stream if unchanged!

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    convolution_enabled = enabled;
    if (enabled)
      ldpc_enabled = false; // Mutually exclusive with LDPC
    clear_baseband_state();

    if (was_running)
      start();
    unlock_config();
  }

  bool get_convolutional_encoding() const { return convolution_enabled; }

  void set_conv_rate(uint8_t rate) {
    if (rate > 4)
      rate = 4;
    if (conv_rate == rate)
      return; // Do not interrupt running stream if unchanged!

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    conv_rate = rate;
    clear_baseband_state();

    if (was_running)
      start();
    unlock_config();
  }

  uint8_t get_conv_rate() const { return conv_rate; }

  void set_reed_solomon_enabled(bool enabled) {
    if (reed_solomon_enabled == enabled)
      return; // Do not interrupt running stream if unchanged!

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    reed_solomon_enabled = enabled;
    if (enabled) {
      ldpc_enabled = false; // Mutually exclusive with LDPC
    }
    clear_baseband_state();
    build_oid_frame();

    if (was_running)
      start();
    unlock_config();
  }

  bool get_reed_solomon_enabled() const { return reed_solomon_enabled; }

  void set_ldpc_enabled(bool enabled) {
    if (ldpc_enabled == enabled)
      return;

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    ldpc_enabled = enabled;
    if (enabled) {
      reed_solomon_enabled = false; // Mutually exclusive with RS
      convolution_enabled = false;  // Mutually exclusive with Convolutional
    }

    clear_baseband_state();
    build_oid_frame();

    if (was_running)
      start();
    unlock_config();
  }
  bool get_ldpc_enabled() const { return ldpc_enabled; }

  void set_fecf_enabled(bool enabled) {
    if (fecf_enabled == enabled)
      return;

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    fecf_enabled = enabled;
    clear_baseband_state();
    build_oid_frame();

    if (was_running)
      start();
    unlock_config();
  }
  bool get_fecf_enabled() const { return fecf_enabled; }

  void set_randomizer_enabled(bool enabled) {
    if (randomizer_enabled == enabled)
      return; // Do not interrupt running stream if unchanged!

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    randomizer_enabled = enabled;
    clear_baseband_state();
    build_oid_frame();

    if (was_running)
      start();
    unlock_config();
  }

  bool get_randomizer_enabled() const { return randomizer_enabled; }

  void set_randomizer_poly(uint8_t poly) {
    if (poly != 8 && poly != 17)
      return;
    if (randomizer_poly == poly)
      return;

    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    randomizer_poly = poly;
    clear_baseband_state();
    build_oid_frame();

    if (was_running)
      start();
    unlock_config();
  }

  uint8_t get_randomizer_poly() const { return randomizer_poly; }

  void restart_transmission() {
    lock_config();
    reset_vcdu_counters(); // Reset frame counter and VCID counters on
                           // transmitter restart
    build_oid_frame();     // Rebuild OID frame with reset counters
    if (running) {
      stop();
      start();
    }
    unlock_config();
  }

  void set_symbolrate(uint32_t hz) {
    if (hz < 1 || hz > 10000000)
      return;
    if (symbolrate_hz == hz)
      return; // Do not interrupt running stream if unchanged!

    lock_config();

    // 1. Check state ONCE at the top and stop immediately
    bool was_running = running;
    if (was_running) {
      stop();
    }

    symbolrate_hz = hz;

    if (rrc_fpga_auto) {
      rrc_filter_span = choose_span(rrc_alpha); // Calculate span FIRST
      rrc_fixed_L = choose_L(symbolrate_hz, rrc_filter_span);
    }

    if (rrc_enabled && rrc_fixed_L == 0) {
      compute_rrc_lut();
    }

    if (rrc_fixed_L > 0) {
      rrc_L = rrc_fixed_L;
      send_fpga_spi_config();
    }

    uint32_t sample_rate = hz;
    if (rrc_enabled && rrc_fixed_L == 0) {
      sample_rate = hz * rrc_L;
    }

    float pio_instructions_per_symbol = 2.0f;
    float clkdiv = (float)clock_get_hz(clk_sys) /
                   (pio_instructions_per_symbol * sample_rate);

    if (clkdiv < 1.0f)
      clkdiv = 1.0f;
    else if (clkdiv > 65535.996f)
      clkdiv = 65535.996f;

    // Now 100% safe because the state machine is definitively halted
    pio_sm_set_clkdiv(pio, sm, clkdiv);

    // 2. Safely restart everything at the very end
    if (was_running) {
      start();
    }

    unlock_config();
  }

  uint32_t get_symbolrate() const { return symbolrate_hz; }

  void reset_baseband_queues() {
    lock_config();
    bool was_running = running;
    if (was_running)
      stop();

    clear_baseband_state();

    if (was_running)
      start();
    unlock_config();
  }

  void init_frame() {
    memcpy(frame, oid_frame, MAX_FRAME_SIZE);
    prepare_tx_stream();
  }

  void print_debug_info() const {
    Serial.print("  dma_ring_head: ");
    Serial.println(dma_ring_head);
    Serial.print("  dma_ring_tail: ");
    Serial.println(dma_ring_tail);
    Serial.print("  underflow_buf[0-3]: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(underflow_buf[i], HEX);
      if (i < 3)
        Serial.print(" ");
    }
    Serial.println();
  }

  __attribute__((section(".time_critical.process_baseband_to_dma"))) bool
  process_baseband_to_dma() {
    uint16_t h = dma_ring_head;
    uint16_t t = dma_ring_tail;

    uint16_t active_chunks = (h >= t) ? (h - t) : (DMA_RING_SIZE - t + h);
    if (active_chunks >= DMA_RING_SIZE - 3)
      return false; // Queue full

    if (!pop_pending_frame(current_tx_frame)) {
      return false; // Queue empty, wait for Core 1 to generate a dynamic frame.
    }

    DmaChunk *chunk = &dma_ring[h];

    uint8_t vcid_byte = current_tx_frame[5];
    if (randomizer_enabled) {
      vcid_byte ^= get_randomizer_seq()[1]; // Revert the XOR for byte 5
    }
    bool chunk_is_user = ((vcid_byte & 0x3F) != 0x3F);
    uint16_t frame_len = get_frame_size();

    if (rrc_enabled && rrc_lut != nullptr) {
      uint32_t bytes_written = rrc_chunk_bytes_written; // LOAD STATE
      uint8_t *dest = (uint8_t *)chunk->words;
      uint32_t max_bytes = MAX_DMA_WORDS * 4;
      uint32_t L = rrc_L;
      uint32_t mask = (1 << rrc_filter_span) - 1;

      uint32_t words_to_copy = L >> 2;

      for (uint16_t i = 0; i < frame_len; i++) {
        uint8_t b = current_tx_frame[i];

        if (!convolution_enabled) {
          for (int bit = 7; bit >= 0; bit--) {
            uint8_t symbol = (b >> bit) & 1;
            rrc_history = ((rrc_history << 1) | symbol) & mask;

            if (bytes_written + L > max_bytes) {
              chunk->length = bytes_written / 4;
              chunk->is_user_data = chunk_is_user;
              __dmb();
              dma_ring_head = (h + 1) % DMA_RING_SIZE;

              h = (h + 1) % DMA_RING_SIZE;
              while (true) {
                uint16_t tail = dma_ring_tail;
                uint16_t active =
                    (h >= tail) ? (h - tail) : (DMA_RING_SIZE - tail + h);
                if (active < DMA_RING_SIZE - 3)
                  break;
                if (!running || config_lock)
                  return false;
                delayMicroseconds(10);
              }
              chunk = &dma_ring[h];
              dest = (uint8_t *)chunk->words;
              bytes_written = 0;
            }

            uint32_t *dst_word = (uint32_t *)(dest + bytes_written);
            const uint32_t *src_word =
                (const uint32_t *)(rrc_lut + (rrc_history * L));
            uint32_t words_to_copy = L >> 2; // Divide by 4

            for (uint32_t w = 0; w < words_to_copy; w++) {
              dst_word[w] = src_word[w];
            }
            bytes_written += L;
          }
        } else {
          uint16_t out_word = CCSDS_CONV_FAST_LUT.byte_out[b] ^
                              CCSDS_CONV_FAST_LUT.state_out[conv_shift_reg];
          conv_shift_reg = CCSDS_CONV_FAST_LUT.next_state[b];

          if (conv_rate == 0) { // Rate 1/2
            for (int s = 15; s >= 0; s--) {
              uint8_t symbol = (out_word >> s) & 1;
              rrc_history = ((rrc_history << 1) | symbol) & mask;

              if (bytes_written + L > max_bytes) {
                chunk->length = bytes_written / 4;
                chunk->is_user_data = chunk_is_user;
                __dmb();
                dma_ring_head = (h + 1) % DMA_RING_SIZE;

                h = (h + 1) % DMA_RING_SIZE;
                while (true) {
                  uint16_t tail = dma_ring_tail;
                  uint16_t active =
                      (h >= tail) ? (h - tail) : (DMA_RING_SIZE - tail + h);
                  if (active < DMA_RING_SIZE - 3)
                    break;
                  if (!running || config_lock)
                    return false;
                  delayMicroseconds(10);
                }
                chunk = &dma_ring[h];
                dest = (uint8_t *)chunk->words;
                bytes_written = 0;
              }

              uint32_t *dst_word = (uint32_t *)(dest + bytes_written);
              const uint32_t *src_word =
                  (const uint32_t *)(rrc_lut + (rrc_history * L));

              for (uint32_t w = 0; w < words_to_copy; w++) {
                dst_word[w] = src_word[w];
              }
              bytes_written += L;
            }
          } else { // Punctured rates
            uint32_t punc_entry1 =
                CCSDS_PUNC_FAST_LUT
                    .table[conv_rate][punc_phase][(out_word >> 8) & 0xFF];
            punc_phase = (punc_entry1 >> 16) & 0xFF;
            uint8_t bits_to_add1 = (punc_entry1 >> 8) & 0xFF;

            for (int s = bits_to_add1 - 1; s >= 0; s--) {
              uint8_t symbol = (punc_entry1 >> s) & 1;
              rrc_history = ((rrc_history << 1) | symbol) & mask;

              if (bytes_written + L > max_bytes) {
                chunk->length = bytes_written / 4;
                chunk->is_user_data = chunk_is_user;
                __dmb();
                dma_ring_head = (h + 1) % DMA_RING_SIZE;

                h = (h + 1) % DMA_RING_SIZE;
                while (true) {
                  uint16_t tail = dma_ring_tail;
                  uint16_t active =
                      (h >= tail) ? (h - tail) : (DMA_RING_SIZE - tail + h);
                  if (active < DMA_RING_SIZE - 3)
                    break;
                  if (!running || config_lock)
                    return false;
                  delayMicroseconds(10);
                }
                chunk = &dma_ring[h];
                dest = (uint8_t *)chunk->words;
                bytes_written = 0;
              }

              uint32_t *dst_word = (uint32_t *)(dest + bytes_written);
              const uint32_t *src_word =
                  (const uint32_t *)(rrc_lut + (rrc_history * L));

              for (uint32_t w = 0; w < words_to_copy; w++) {
                dst_word[w] = src_word[w];
              }
              bytes_written += L;
            }

            uint32_t punc_entry2 =
                CCSDS_PUNC_FAST_LUT
                    .table[conv_rate][punc_phase][out_word & 0xFF];
            punc_phase = (punc_entry2 >> 16) & 0xFF;
            uint8_t bits_to_add2 = (punc_entry2 >> 8) & 0xFF;

            for (int s = bits_to_add2 - 1; s >= 0; s--) {
              uint8_t symbol = (punc_entry2 >> s) & 1;
              rrc_history = ((rrc_history << 1) | symbol) & mask;

              if (bytes_written + L > max_bytes) {
                chunk->length = bytes_written / 4;
                chunk->is_user_data = chunk_is_user;
                __dmb();
                dma_ring_head = (h + 1) % DMA_RING_SIZE;

                h = (h + 1) % DMA_RING_SIZE;
                while (true) {
                  uint16_t tail = dma_ring_tail;
                  uint16_t active =
                      (h >= tail) ? (h - tail) : (DMA_RING_SIZE - tail + h);
                  if (active < DMA_RING_SIZE - 3)
                    break;
                  if (!running || config_lock)
                    return false;
                  delayMicroseconds(10);
                }
                chunk = &dma_ring[h];
                dest = (uint8_t *)chunk->words;
                bytes_written = 0;
              }

              uint32_t *dst_word = (uint32_t *)(dest + bytes_written);
              const uint32_t *src_word =
                  (const uint32_t *)(rrc_lut + (rrc_history * L));
              uint32_t words_to_copy = L >> 2; // Divide by 4

              for (uint32_t w = 0; w < words_to_copy; w++) {
                dst_word[w] = src_word[w];
              }
              bytes_written += L;
            }
          }
        }
      }

      rrc_chunk_bytes_written =
          bytes_written; // SAVE STATE (Do NOT push partial chunk)
      return true;
    }

    // --- Non-RRC path ---
    uint32_t words_written = tx_chunk_words_written; // LOAD STATE

    if (!convolution_enabled) {
      uint64_t accum_data = tx_accum_data;
      uint8_t accum_bits = tx_accum_bits;

      for (uint16_t i = 0; i < frame_len; i++) {
        accum_data = (accum_data << 8) | current_tx_frame[i];
        accum_bits += 8;

        while (accum_bits >= 32) {
          accum_bits -= 32;
          uint32_t raw_word = (uint32_t)(accum_data >> accum_bits);
          chunk->words[words_written] = raw_word;
          words_written += 1;
          accum_data &= (1ULL << accum_bits) - 1;

          if (words_written >= MAX_DMA_WORDS) {
            chunk->length = words_written;
            chunk->is_user_data = chunk_is_user;
            __dmb();
            dma_ring_head = (h + 1) % DMA_RING_SIZE;
            h = (h + 1) % DMA_RING_SIZE;
            while (true) {
              uint16_t tail = dma_ring_tail;
              uint16_t active =
                  (h >= tail) ? (h - tail) : (DMA_RING_SIZE - tail + h);
              if (active < DMA_RING_SIZE - 3)
                break;
              if (!running || config_lock)
                return false;
              delayMicroseconds(10);
            }
            chunk = &dma_ring[h];
            words_written = 0;
          }
        }
      }
      tx_accum_data = accum_data;
      tx_accum_bits = accum_bits;
    } else {
      uint64_t accum_data = tx_accum_data;
      uint8_t accum_bits = tx_accum_bits;
      uint8_t current_punc_phase = punc_phase;
      uint8_t current_conv_rate = conv_rate;

      for (uint16_t i = 0; i < get_frame_size(); i++) {
        uint8_t b = current_tx_frame[i];
        uint16_t out_word = CCSDS_CONV_FAST_LUT.byte_out[b] ^
                            CCSDS_CONV_FAST_LUT.state_out[conv_shift_reg];
        conv_shift_reg = CCSDS_CONV_FAST_LUT.next_state[b];

        if (current_conv_rate == 0) { // Rate 1/2
          accum_data = (accum_data << 16) | out_word;
          accum_bits += 16;
        } else {
          uint32_t punc_entry1 =
              CCSDS_PUNC_FAST_LUT.table[current_conv_rate][current_punc_phase]
                                       [(out_word >> 8) & 0xFF];
          current_punc_phase = (punc_entry1 >> 16) & 0xFF;
          uint8_t bits_to_add1 = (punc_entry1 >> 8) & 0xFF;
          accum_data = (accum_data << bits_to_add1) | (punc_entry1 & 0xFF);
          accum_bits += bits_to_add1;

          uint32_t punc_entry2 =
              CCSDS_PUNC_FAST_LUT.table[current_conv_rate][current_punc_phase]
                                       [out_word & 0xFF];
          current_punc_phase = (punc_entry2 >> 16) & 0xFF;
          uint8_t bits_to_add2 = (punc_entry2 >> 8) & 0xFF;
          accum_data = (accum_data << bits_to_add2) | (punc_entry2 & 0xFF);
          accum_bits += bits_to_add2;
        }

        while (accum_bits >= 32) {
          accum_bits -= 32;
          uint32_t raw_word = (uint32_t)(accum_data >> accum_bits);
          chunk->words[words_written] = raw_word;
          words_written += 1;
          accum_data &= (1ULL << accum_bits) - 1;

          if (words_written >= MAX_DMA_WORDS) {
            chunk->length = words_written;
            chunk->is_user_data = chunk_is_user;
            __dmb();
            dma_ring_head = (h + 1) % DMA_RING_SIZE;
            h = (h + 1) % DMA_RING_SIZE;
            while (true) {
              uint16_t tail = dma_ring_tail;
              uint16_t active =
                  (h >= tail) ? (h - tail) : (DMA_RING_SIZE - tail + h);
              if (active < DMA_RING_SIZE - 3)
                break;
              if (!running || config_lock)
                return false;
              delayMicroseconds(10);
            }
            chunk = &dma_ring[h];
            words_written = 0;
          }
        }
      }
      tx_accum_data = accum_data;
      tx_accum_bits = accum_bits;
      punc_phase = current_punc_phase;
    }

    // Leftover bits in tx_accum_data (tx_accum_bits > 0) are NOT flushed at the
    // end of the frame. They are carried over to the next frame to prevent
    // inserting dummy symbols/padding between frames, which would break
    // receiver CADU frame synchronization.

    if (words_written > 0) {
      chunk->length = words_written;
      chunk->is_user_data = chunk_is_user;
      __dmb();
      dma_ring_head = (h + 1) % DMA_RING_SIZE;
    }
    tx_chunk_words_written = 0; // Reset for next frame
    return true;
  }

  __attribute__((section(".time_critical.rearm_channel"))) void
  rearm_channel(int chan) {
    uint16_t h = dma_ring_head;
    uint16_t t = dma_ring_tail;

    if (h != t) {
      // Zero-copy: Pass the chunk directly from Core 1's ring memory to the DMA
      // hardware!
      dma_channel_set_read_addr(chan, (const void *)dma_ring[t].words, false);
      dma_channel_set_trans_count(chan, dma_ring[t].length, false);
      if (dma_ring[t].is_user_data)
        tx_user_chunks_dma_cleared++;
      dma_ring_tail = (t + 1) % DMA_RING_SIZE;
    } else {
      // Queue starved! Send underflow signal to prevent a DMA hardware halt
      dma_channel_set_read_addr(chan, (const void *)underflow_buf, false);
      uint32_t underflow_len = (rrc_fixed_L > 0) ? 32 : MAX_DMA_WORDS;
      dma_channel_set_trans_count(chan, underflow_len, false);
      dma_underflows++;
    }
  }

  __attribute__((section(".time_critical.handle_dma_irq"))) void
  handle_dma_irq() {
    if (!running) {
      // Safely sink phantom interrupts from aborted transfers to prevent memory
      // corruption
      if (dma_channel_get_irq1_status(dma_chan_0))
        dma_channel_acknowledge_irq1(dma_chan_0);
      if (dma_channel_get_irq1_status(dma_chan_1))
        dma_channel_acknowledge_irq1(dma_chan_1);
      return;
    }

    bool ch0_fired = dma_channel_get_irq1_status(dma_chan_0);
    bool ch1_fired = dma_channel_get_irq1_status(dma_chan_1);

    if (ch0_fired) {
      dma_channel_acknowledge_irq1(dma_chan_0);
      rearm_channel(dma_chan_0);
    }
    if (ch1_fired) {
      dma_channel_acknowledge_irq1(dma_chan_1);
      rearm_channel(dma_chan_1);
    }

    // If neither channel is busy, the hardware ping-pong chain has completely
    // halted (e.g., due to ISR latency). We must jump-start the channel we JUST
    // re-armed to guarantee immediate recovery!
    if (!dma_channel_is_busy(dma_chan_0) && !dma_channel_is_busy(dma_chan_1)) {
      if (ch0_fired)
        dma_channel_start(dma_chan_0);
      else if (ch1_fired)
        dma_channel_start(dma_chan_1);
    }
  }

  void init_pio(uint pin) {
    pio = pio0;
    if (pio_offset_fpga == 0xFFFFFFFF) {
      pio_offset_fpga = pio_add_program(pio, &bpsk_fpga_program_default);
    }

    uint offset = pio_offset_fpga;

    if (sm != 0xFFFFFFFF) {
      pio_sm_unclaim(pio, sm);
      sm = 0xFFFFFFFF;
    }
    if (dma_chan_0 >= 0) {
      dma_channel_unclaim(dma_chan_0);
      dma_chan_0 = -1;
    }
    if (dma_chan_1 >= 0) {
      dma_channel_unclaim(dma_chan_1);
      dma_chan_1 = -1;
    }

    sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = pio_get_default_sm_config();

    // FPGA mode: GP20 is BCLK and GP21 is data.
    sm_config_set_out_pins(&c, PIN_FPGA_DATA, 1);
    sm_config_set_sideset_pins(&c, PIN_FPGA_BCLK);
    sm_config_set_sideset(&c, 1, false, false);

    sm_config_set_wrap(&c, offset, offset + 1);

    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_gpio_init(pio, PIN_FPGA_BCLK);
    pio_gpio_init(pio, PIN_FPGA_DATA);
    for (int p = PIN_FPGA_BCLK; p <= PIN_FPGA_DATA; p++) {
      gpio_set_dir(p, GPIO_OUT);
      gpio_set_drive_strength(p, GPIO_DRIVE_STRENGTH_2MA);
      gpio_set_slew_rate(p, GPIO_SLEW_RATE_SLOW);
    }

    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_FPGA_BCLK, 2, true);

    uint32_t current_rate = symbolrate_hz;
    symbolrate_hz = 0; // force update
    set_symbolrate(current_rate);

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

    dma_channel_set_irq1_enabled(dma_chan_0, true);
    dma_channel_set_irq1_enabled(dma_chan_1, true);

    // irq handler configuration deferred to enable_dma_irq_core1() on Core 1
  }

  bool tx_fifo_empty() const { return pio_sm_is_tx_fifo_empty(pio, sm); }
  bool tx_fifo_full() const { return pio_sm_is_tx_fifo_full(pio, sm); }
  uint get_sm() const { return sm; }
  void start() {
    if (running)
      return;

    Serial.println("[MOD] Starting modulator...");

    clear_baseband_state(); // Cleanly reset ring, history, conv states, and
                            // queues

    if (rrc_fixed_L > 0) {
      send_fpga_spi_config();
    }

    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, false);

    pio_sm_set_consecutive_pindirs(pio, sm, PIN_FPGA_BCLK, 2, true);

    // Reset the Program Counter (PC) to the start of the FPGA program.
    pio_sm_exec(pio, sm, pio_encode_jmp(pio_offset_fpga));
    uint32_t underflow_len = (rrc_fixed_L > 0) ? 32 : MAX_DMA_WORDS;
    dma_channel_configure(dma_chan_0, &c0, &pio->txf[sm],
                          (const void *)underflow_buf, underflow_len, false);
    dma_channel_configure(dma_chan_1, &c1, &pio->txf[sm],
                          (const void *)underflow_buf, underflow_len, false);

    // Clear any pending interrupts to prevent phantom start interrupts
    dma_channel_acknowledge_irq1(dma_chan_0);
    dma_channel_acknowledge_irq1(dma_chan_1);
    irq_clear(DMA_IRQ_1);

    dma_channel_set_irq1_enabled(dma_chan_0, true);
    dma_channel_set_irq1_enabled(dma_chan_1, true);

    running = true; // MUST be true before starting DMA so the ISR doesn't
                    // reject the very first interrupt
    __dmb(); // Ensure running state is visible to the ISR before it can be
             // triggered

    dma_channel_start(dma_chan_0);

    pio_sm_set_enabled(pio, sm, true);
    Serial.println("[MOD] Modulator started.");
  }

  void stop() {
    Serial.println("[MOD] Stopping modulator...");
    running = false; // Instantly prevent the ISR from processing new chunks
    __dmb();
    while (core1_processing) {
      __asm volatile("nop");
    }

    // CRITICAL FIX: Disable PIO and clear FIFOs *BEFORE* aborting DMA!
    // This forcefully drops the DREQ signal. Aborting a DMA channel
    // with an active DREQ permanently locks up the RP2350 bus!
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    // Disable IRQs so the ISR doesn't re-arm the channels during abort
    dma_channel_set_irq1_enabled(dma_chan_0, false);
    dma_channel_set_irq1_enabled(dma_chan_1, false);

    // Break DMA chaining to prevent race conditions during abort
    dma_channel_config c_abort_0 = dma_get_channel_config(dma_chan_0);
    channel_config_set_chain_to(&c_abort_0, dma_chan_0);
    dma_channel_set_config(dma_chan_0, &c_abort_0, false);

    dma_channel_config c_abort_1 = dma_get_channel_config(dma_chan_1);
    channel_config_set_chain_to(&c_abort_1, dma_chan_1);
    dma_channel_set_config(dma_chan_1, &c_abort_1, false);

    // Now abort the channels safely (DREQ is guaranteed cleared)
    dma_channel_abort(dma_chan_0);
    dma_channel_abort(dma_chan_1);

    // Wait for both DMA channels to completely finish aborting
    while (dma_channel_is_busy(dma_chan_0) || dma_channel_is_busy(dma_chan_1)) {
      __asm volatile("nop");
    }

    // Clear pending IRQs to prevent phantom Restarts mid-flush
    dma_channel_acknowledge_irq1(dma_chan_0);
    dma_channel_acknowledge_irq1(dma_chan_1);
    Serial.println("[MOD] Modulator stopped.");
  }
};

#endif