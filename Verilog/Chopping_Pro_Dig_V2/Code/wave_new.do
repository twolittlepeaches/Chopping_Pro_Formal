onerror {resume}
radix define -hex {
    "spi_signals" "-default",
    "default" "-default",
    "default" "-default",
    "default" "-default",
    "default" "",
    -default default
}
radix define -decimal {
    "phase_signals" "-default",
    "default" "-default",
    "default" "-default",
    "default" "-default",
    "default" "",
    -default default
}
radix define -binary {
    "clock_signals" "-default",
    "default" "-default",
    "default" "-default",
    "default" "-default",
    "default" "",
    -default default
}
quietly WaveActivateNextPane {} 0
add wave -noupdate -divider {SPI Interface}
add wave -noupdate /digital_top_tb/SCLK
add wave -noupdate /digital_top_tb/RST_N
add wave -noupdate /digital_top_tb/MOSI
add wave -noupdate /digital_top_tb/MISO
add wave -noupdate -divider {Clock Signals}
add wave -noupdate /digital_top_tb/FREF
add wave -noupdate /digital_top_tb/ckv
add wave -noupdate /digital_top_tb/ckvd
add wave -noupdate /digital_top_tb/CKR
add wave -noupdate /digital_top_tb/CLK_SD
add wave -noupdate -divider {DCO Control}
add wave -noupdate /digital_top_tb/OTW_T_H
add wave -noupdate /digital_top_tb/OTW_T_L
add wave -noupdate /digital_top_tb/DCO_sdm
add wave -noupdate -format Analog-Step -height 92 -max 4262399999.9999995 -min 3586000000.0 /digital_top_tb/DCO_inst/fv
add wave -noupdate -divider Calib
add wave -noupdate -format Analog-Step -height 92 -max 2047.0 -min -2048.0 -radix decimal /digital_top_tb/digital_top_inst/digital_main_inst/LMS_Calib_inst/INVKDTC
add wave -noupdate -format Analog-Step -height 92 -max 2048.0 -min -2048.0 -radix decimal /digital_top_tb/digital_top_inst/digital_main_inst/LMS_Calib_inst/OFFSET_DTC_calib
add wave -noupdate -format Analog-Step -height 92 -max 5.0000000000000009 -min -54.0 -radix decimal /digital_top_tb/digital_top_inst/digital_main_inst/LMS_Calib_inst/ALPHA_ODD
add wave -noupdate -divider {Phase Error}
add wave -noupdate -format Analog-Step -height 92 -max 255.0 -min -255.0 -radix decimal /digital_top_tb/digital_top_inst/digital_main_inst/PHE
add wave -noupdate -divider {DTC Control}
add wave -noupdate -format Analog-Step -height 92 -max 2048.0 -min -2048.0 -radix decimal /digital_top_tb/digital_top_inst/digital_main_inst/PHR_F
add wave -noupdate -format Analog-Step -height 92 -max 1022.9999999999999 -radix unsigned /digital_top_tb/digital_top_inst/digital_main_inst/phase_adder_inst/DTC_P_CTRL_10b
add wave -noupdate -format Analog-Step -height 92 -max 1024.0 -radix unsigned /digital_top_tb/digital_top_inst/digital_main_inst/phase_adder_inst/DTC_N_CTRL_10b
add wave -noupdate -format Analog-Step -height 92 -max 1022.9999999999999 -radix unsigned /digital_top_tb/digital_top_inst/digital_main_inst/DTC_P_CTRL_10b_final
add wave -noupdate -format Analog-Step -height 92 -max 1024.0 -radix unsigned /digital_top_tb/digital_top_inst/digital_main_inst/DTC_N_CTRL_10b_final
add wave -noupdate /digital_top_tb/DTC_P_top_4b
add wave -noupdate /digital_top_tb/DTC_P_top_6u
add wave -noupdate /digital_top_tb/DTC_P_bot_4u
add wave -noupdate /digital_top_tb/DTC_N_top_4b
add wave -noupdate /digital_top_tb/DTC_N_top_6u
add wave -noupdate /digital_top_tb/DTC_N_bot_4u
add wave -noupdate /digital_top_tb/MAIN_DTC_clk_delay
add wave -noupdate /digital_top_tb/OFFSET_DTC_clk_delay
add wave -noupdate -divider {TDC Interface}
add wave -noupdate /digital_top_tb/TDC_Q
add wave -noupdate /digital_top_tb/MAIN_DTC_clk_delayi
add wave -noupdate /digital_top_tb/OFFSET_DTC_clk_delay
add wave -noupdate -divider {Chopper & MMD}
add wave -noupdate /digital_top_tb/MAIN_DTC_clk_ref
add wave -noupdate /digital_top_tb/OFFSET_DTC_clk_ref
add wave -noupdate /digital_top_tb/MMD_RN
add wave -noupdate -divider {Power Supply}
add wave -noupdate /digital_top_tb/VD
add wave -noupdate /digital_top_tb/VS
add wave -noupdate -divider {Reserved Outputs}
add wave -noupdate /digital_top_tb/Reserved_OUT1
add wave -noupdate /digital_top_tb/DCO_coarse_ENi
add wave -noupdate -divider {Test Control}
add wave -noupdate /digital_top_tb/MAIN_DTC_TEST_EN
add wave -noupdate /digital_top_tb/DTC_TEST_MODE
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {741359976438 fs} 0}
quietly wave cursor active 1
configure wave -namecolwidth 300
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {734587114736 fs} {748705943435 fs}
