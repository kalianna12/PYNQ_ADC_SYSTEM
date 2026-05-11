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
