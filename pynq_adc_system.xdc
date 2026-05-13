## PYNQADC system: dual SPI master + sweep FSM

set_property PACKAGE_PIN H16 [get_ports clk_125m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_125m]
create_clock -period 8.000 -name clk_125m [get_ports clk_125m]

## LEDs
set_property PACKAGE_PIN R14 [get_ports led0]
set_property IOSTANDARD LVCMOS33 [get_ports led0]

set_property PACKAGE_PIN P14 [get_ports led1]
set_property IOSTANDARD LVCMOS33 [get_ports led1]

set_property PACKAGE_PIN N16 [get_ports led2]
set_property IOSTANDARD LVCMOS33 [get_ports led2]

set_property PACKAGE_PIN M14 [get_ports led3]
set_property IOSTANDARD LVCMOS33 [get_ports led3]

## SPI-A to ESP32-P4 (Master, PmodB P1-P4)
set_property PACKAGE_PIN W14 [get_ports p4_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_mosi]

set_property PACKAGE_PIN Y14 [get_ports p4_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_miso]

set_property PACKAGE_PIN T11 [get_ports p4_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_sclk]

set_property PACKAGE_PIN T10 [get_ports p4_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_cs_n]

## SPI-B to PYNQDDS (Master, PmodB P7-P10)
set_property PACKAGE_PIN V16 [get_ports dds_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_mosi]

set_property PACKAGE_PIN W16 [get_ports dds_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_miso]

set_property PACKAGE_PIN V12 [get_ports dds_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_sclk]

set_property PACKAGE_PIN W13 [get_ports dds_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_cs_n]

## AD9226 channel A (hardware pin names converted through verified RPi/GPIO skew map)
## ACK hardware Y18 -> XDC V6
set_property PACKAGE_PIN V6 [get_ports adc_a_clk]
set_property IOSTANDARD LVCMOS33 [get_ports adc_a_clk]

## A1..A12, A1 is LSB. Converted pins:
## A1  hardware W18 -> XDC W18
## A2  hardware Y19 -> XDC Y6
## A3  hardware W19 -> XDC W19
## A4  hardware C20 -> XDC C20
## A5  hardware V6  -> XDC Y18
## A6  hardware W6  -> XDC W6
## A7  hardware U7  -> XDC U7
## A8  hardware U18 -> XDC Y7
## A9  hardware V7  -> XDC V7
## A10 hardware U19 -> XDC F20
## A11 hardware U8  -> XDC U8
## A12 hardware F19 -> XDC F19
set_property PACKAGE_PIN W18 [get_ports {adc_a_data[0]}]
set_property PACKAGE_PIN Y6  [get_ports {adc_a_data[1]}]
set_property PACKAGE_PIN W19 [get_ports {adc_a_data[2]}]
set_property PACKAGE_PIN C20 [get_ports {adc_a_data[3]}]
set_property PACKAGE_PIN Y18 [get_ports {adc_a_data[4]}]
set_property PACKAGE_PIN W6  [get_ports {adc_a_data[5]}]
set_property PACKAGE_PIN U7  [get_ports {adc_a_data[6]}]
set_property PACKAGE_PIN Y7 [get_ports {adc_a_data[7]}]
set_property PACKAGE_PIN V7  [get_ports {adc_a_data[8]}]
set_property PACKAGE_PIN F20 [get_ports {adc_a_data[9]}]
set_property PACKAGE_PIN U8  [get_ports {adc_a_data[10]}]
set_property PACKAGE_PIN F19 [get_ports {adc_a_data[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_a_data[*]}]

## ORA hardware V8 -> XDC V8
set_property PACKAGE_PIN V8 [get_ports adc_a_ora]
set_property IOSTANDARD LVCMOS33 [get_ports adc_a_ora]

## AD9226 channel B (hardware pin names converted through verified RPi/GPIO skew map)
## BCK hardware F20 -> XDC U19
set_property PACKAGE_PIN U19 [get_ports adc_b_clk]
set_property IOSTANDARD LVCMOS33 [get_ports adc_b_clk]

## B1..B12, B1 is LSB. Converted pins:
## B1  hardware V10 -> XDC V10
## B2  hardware Y17 -> XDC Y17
## B3  hardware W10 -> XDC W10
## B4  hardware B20 -> XDC B20
## B5  hardware Y16 -> XDC Y16
## B6  hardware B19 -> XDC B19
## B7  hardware Y6  -> XDC Y19
## B8  hardware A20 -> XDC A20
## B9  hardware Y7  -> XDC U18
## B10 hardware Y9  -> XDC Y9
## B11 hardware W8  -> XDC W8
## B12 hardware W9  -> XDC W9
set_property PACKAGE_PIN V10 [get_ports {adc_b_data[0]}]
set_property PACKAGE_PIN Y17 [get_ports {adc_b_data[1]}]
set_property PACKAGE_PIN W10 [get_ports {adc_b_data[2]}]
set_property PACKAGE_PIN B20 [get_ports {adc_b_data[3]}]
set_property PACKAGE_PIN Y16 [get_ports {adc_b_data[4]}]
set_property PACKAGE_PIN B19 [get_ports {adc_b_data[5]}]
set_property PACKAGE_PIN Y19 [get_ports {adc_b_data[6]}]
set_property PACKAGE_PIN A20 [get_ports {adc_b_data[7]}]
set_property PACKAGE_PIN U18 [get_ports {adc_b_data[8]}]
set_property PACKAGE_PIN Y9  [get_ports {adc_b_data[9]}]
set_property PACKAGE_PIN W8  [get_ports {adc_b_data[10]}]
set_property PACKAGE_PIN W9  [get_ports {adc_b_data[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_b_data[*]}]

## B13 hardware Y8 -> XDC Y8, temporarily treated as ORB; confirm module silkscreen/schematic.
set_property PACKAGE_PIN Y8 [get_ports adc_b_orb]
set_property IOSTANDARD LVCMOS33 [get_ports adc_b_orb]
