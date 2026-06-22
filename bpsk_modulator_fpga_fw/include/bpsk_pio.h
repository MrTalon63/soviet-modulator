#pragma once

#include <cstdint>
#include <hardware/pio.h>
#include "pins.h"

static const uint16_t bpsk_fpga_program[] = {
    0x6001, // 0: out pins, 1         side 0
    0xB042, // 1: nop                 side 1
};

static const pio_program_t bpsk_fpga_program_default = {
    .instructions = bpsk_fpga_program,
    .length = 2,
    .origin = -1,
};
