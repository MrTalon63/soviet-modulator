#pragma once

// --- Legacy 8-Bit DAC Pins ---
#define PIN_DAC_BCLK 0           // DAC Clock (used for old PIO)
#define PIN_DAC_DATA_BASE 1      // DAC Data [1:8]

// --- New FPGA Pins ---
#define PIN_FPGA_BCLK 20         // BCLK for FPGA Mode
#define PIN_FPGA_DATA 21         // 1-bit baseband payload for FPGA

// --- SPI Pins (for FPGA Configuration) ---
#define PIN_SPI_SCK 18
#define PIN_SPI_TX 19
#define PIN_SPI_RX 16
#define PIN_SPI_CS 17

// --- System Pins ---
#define LED_PIN 25
#define PIN_SMPS_MODE 23
#define SI5351_SDA_PIN 26
#define SI5351_SCL_PIN 27

// --- Hardware Selection ---
// The original MODULATING_PIN mapping for legacy reference
#define MODULATING_PIN PIN_DAC_BCLK
