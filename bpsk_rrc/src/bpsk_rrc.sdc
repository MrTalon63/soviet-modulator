//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file

create_clock -name xtal_27mhz -period 37.037 -waveform {0 18.518} [get_ports {xtal_27mhz_pin}]

set_false_path -from [get_ports {hardware_reset_n}]
set_false_path -from [get_ports {spi_cs_n spi_sck spi_mosi}]