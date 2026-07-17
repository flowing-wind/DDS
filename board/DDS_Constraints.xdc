set_property PACKAGE_PIN V8 [get_ports dac0_clk]
set_property PACKAGE_PIN V11 [get_ports dac1_clk]
set_property PACKAGE_PIN N16 [get_ports ext_rst]
set_property PACKAGE_PIN U7 [get_ports spi_cs_n]
set_property PACKAGE_PIN T5 [get_ports spi_miso]
set_property PACKAGE_PIN U5 [get_ports spi_mosi]
set_property PACKAGE_PIN V7 [get_ports spi_sck]
set_property PACKAGE_PIN U18 [get_ports sys_clk]
set_property PACKAGE_PIN L15 [get_ports {led[1]}]
set_property PACKAGE_PIN H15 [get_ports {led[0]}]
set_property PACKAGE_PIN V15 [get_ports {dac1_data[0]}]
set_property PACKAGE_PIN W15 [get_ports {dac1_data[1]}]
set_property PACKAGE_PIN W14 [get_ports {dac1_data[2]}]
set_property PACKAGE_PIN Y14 [get_ports {dac1_data[3]}]
set_property PACKAGE_PIN V12 [get_ports {dac1_data[4]}]
set_property PACKAGE_PIN W13 [get_ports {dac1_data[5]}]
set_property PACKAGE_PIN U14 [get_ports {dac1_data[6]}]
set_property PACKAGE_PIN U15 [get_ports {dac1_data[7]}]
set_property PACKAGE_PIN V13 [get_ports {dac1_data[8]}]
set_property PACKAGE_PIN U13 [get_ports {dac1_data[9]}]
set_property PACKAGE_PIN Y11 [get_ports {dac1_data[10]}]
set_property PACKAGE_PIN W11 [get_ports {dac1_data[11]}]
set_property PACKAGE_PIN Y13 [get_ports {dac1_data[12]}]
set_property PACKAGE_PIN Y12 [get_ports {dac1_data[13]}]
set_property PACKAGE_PIN V10 [get_ports {dac0_data[0]}]
set_property PACKAGE_PIN W10 [get_ports {dac0_data[1]}]
set_property PACKAGE_PIN W9 [get_ports {dac0_data[2]}]
set_property PACKAGE_PIN Y9 [get_ports {dac0_data[3]}]
set_property PACKAGE_PIN Y8 [get_ports {dac0_data[4]}]
set_property PACKAGE_PIN Y7 [get_ports {dac0_data[5]}]
set_property PACKAGE_PIN Y6 [get_ports {dac0_data[6]}]
set_property PACKAGE_PIN W6 [get_ports {dac0_data[7]}]
set_property PACKAGE_PIN V6 [get_ports {dac0_data[8]}]
set_property PACKAGE_PIN U10 [get_ports {dac0_data[9]}]
set_property PACKAGE_PIN T9 [get_ports {dac0_data[10]}]
set_property PACKAGE_PIN U9 [get_ports {dac0_data[11]}]
set_property PACKAGE_PIN U8 [get_ports {dac0_data[12]}]
set_property PACKAGE_PIN W8 [get_ports {dac0_data[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac0_data[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dac1_data[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports dac0_clk]
set_property IOSTANDARD LVCMOS33 [get_ports dac1_clk]
set_property IOSTANDARD LVCMOS33 [get_ports ext_rst]
set_property IOSTANDARD LVCMOS33 [get_ports spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports spi_sck]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk]

# ---------------------------------------------------------------------------
# AD9226 dual ADC module - FILL IN THE PINS, then uncomment.
#
# Port index mapping (this is where the module's backwards silk screen gets
# undone, so read it carefully): the module numbers its data bus with the MSB
# on silk "D0". Map header silk D0 -> adc0_data[11] (MSB) down to silk
# D11 -> adc0_data[0] (LSB). Same for the AD2/B channel. adc0_clk drives ACLK
# and adc1_clk drives BCLK - two pins, two ODDRs, identical phase. ATR/BTR are
# the over-range flags; if you leave them unwired, tie the ports to 0 in a
# wrapper instead.
#
# set_property PACKAGE_PIN ?? [get_ports adc0_clk]         ;# ACLK
# set_property PACKAGE_PIN ?? [get_ports adc1_clk]         ;# BCLK
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[11]}]  ;# AD1 silk D0 (MSB)
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[10]}]  ;# AD1 silk D1
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[9]}]   ;# AD1 silk D2
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[8]}]   ;# AD1 silk D3
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[7]}]   ;# AD1 silk D4
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[6]}]   ;# AD1 silk D5
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[5]}]   ;# AD1 silk D6
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[4]}]   ;# AD1 silk D7
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[3]}]   ;# AD1 silk D8
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[2]}]   ;# AD1 silk D9
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[1]}]   ;# AD1 silk D10
# set_property PACKAGE_PIN ?? [get_ports {adc0_data[0]}]   ;# AD1 silk D11 (LSB)
# set_property PACKAGE_PIN ?? [get_ports adc0_otr]         ;# ATR
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[11]}]  ;# AD2 silk D0 (MSB)
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[10]}]  ;# AD2 silk D1
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[9]}]   ;# AD2 silk D2
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[8]}]   ;# AD2 silk D3
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[7]}]   ;# AD2 silk D4
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[6]}]   ;# AD2 silk D5
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[5]}]   ;# AD2 silk D6
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[4]}]   ;# AD2 silk D7
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[3]}]   ;# AD2 silk D8
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[2]}]   ;# AD2 silk D9
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[1]}]   ;# AD2 silk D10
# set_property PACKAGE_PIN ?? [get_ports {adc1_data[0]}]   ;# AD2 silk D11 (LSB)
# set_property PACKAGE_PIN ?? [get_ports adc1_otr]         ;# BTR
# set_property IOSTANDARD LVCMOS33 [get_ports adc0_clk]
# set_property IOSTANDARD LVCMOS33 [get_ports adc1_clk]
# set_property IOSTANDARD LVCMOS33 [get_ports {adc0_data[*]}]
# set_property IOSTANDARD LVCMOS33 [get_ports {adc1_data[*]}]
# set_property IOSTANDARD LVCMOS33 [get_ports adc0_otr]
# set_property IOSTANDARD LVCMOS33 [get_ports adc1_otr]
