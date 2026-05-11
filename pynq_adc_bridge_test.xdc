## PYNQADC bridge test: UART + SPI1 reserved to ESP32-P4 + SPI2 to PYNQDDS

set_property PACKAGE_PIN H16 [get_ports clk_125m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_125m]
create_clock -period 8.000 -name clk_125m [get_ports clk_125m]

## UART debug on Arduino J4, verified
set_property PACKAGE_PIN T14 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
set_property PULLUP true [get_ports uart_rx]

set_property PACKAGE_PIN U12 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

## SPI1 to ESP32-P4 using PmodB P1-P4, kept idle in this test
set_property PACKAGE_PIN W14 [get_ports p4_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_mosi]

set_property PACKAGE_PIN Y14 [get_ports p4_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_miso]

set_property PACKAGE_PIN T11 [get_ports p4_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_sclk]

set_property PACKAGE_PIN T10 [get_ports p4_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports p4_spi_cs_n]

## SPI2 to PYNQDDS using PmodB P7-P10
## PYNQADC is SPI Master
set_property PACKAGE_PIN V16 [get_ports dds_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_mosi]

set_property PACKAGE_PIN W16 [get_ports dds_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_miso]

set_property PACKAGE_PIN V12 [get_ports dds_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_sclk]

set_property PACKAGE_PIN W13 [get_ports dds_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports dds_spi_cs_n]

## Debug LEDs on PYNQADC board
## LD0 heartbeat, LD1 SPI transaction blink, LD2 off, LD3 RX event blink
set_property PACKAGE_PIN R14 [get_ports led0]
set_property IOSTANDARD LVCMOS33 [get_ports led0]

set_property PACKAGE_PIN P14 [get_ports led1]
set_property IOSTANDARD LVCMOS33 [get_ports led1]

set_property PACKAGE_PIN N16 [get_ports led2]
set_property IOSTANDARD LVCMOS33 [get_ports led2]

set_property PACKAGE_PIN M14 [get_ports led3]
set_property IOSTANDARD LVCMOS33 [get_ports led3]
