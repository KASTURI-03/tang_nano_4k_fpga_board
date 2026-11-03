//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11.03 Education 
//Created Time: 2026-09-16 13:29:53
create_clock -name clk_27mhz -period 37.037 -waveform {0 18.518} [get_ports {clk_27mhz}]
create_clock -name cam_pclk -period 41.667 -waveform {0 22.792} [get_ports {cam_pclk}]
