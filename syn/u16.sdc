set_time_format -unit ns -decimal_places 3

derive_clock_uncertainty

create_clock -name {CLK_100MHZ} -period 10 [get_ports {CLK_100MHZ}]

derive_pll_clocks
