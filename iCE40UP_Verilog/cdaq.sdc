# ##############################################################################

# iCEcube SDC

# Version:            2017.08.27940

# File Generated:     Aug 8 2018 11:30:10

# ##############################################################################

####---- CreateClock list ----2
create_clock  -period 83.33 -waveform {0.00 41.67} -name {ICE_CLK} [get_ports {ICE_CLK}] 
create_clock  -period 8.33 -waveform {0.00 4.17} -name {clk_from_adc} [get_ports {clk_from_adc}] 

