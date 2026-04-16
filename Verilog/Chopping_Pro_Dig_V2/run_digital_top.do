# Questasim run script for Digital Top testbench
# Usage: vsim -do run_digital_top.do

# Create work library
vlib work

# Compile design files with high precision
echo "Compiling Digital Top design files..."

set CODE_PATH "../../Verilog/Chopping_Pro_Dig_V2/Code"

# Compile all design files (same as original)
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/FREF_generator.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/DCO.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/sd_mod.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/mmd.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/digital_main.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/SPI_slave.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/digital_top.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/TDC.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/DTC_analog_10b.v
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/chopper.v

# Testbench 通常也在代码目录下，也需要加上
vlog -work work +define+HIGH_PRECISION ${CODE_PATH}/digital_top_tb.v

echo "Compilation completed"

# Start simulation with femtosecond resolution
vsim -t 1fs -vopt -voptargs=+acc work.digital_top_tb

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


# Run simulation
echo "Starting Digital Top simulation with SPI configuration..."
echo "SPI will configure all registers before starting digital_main"

# Run the full simulation
run -all

# Auto-zoom to show the complete simulation
wave zoom full

echo "Simulation completed"
echo "Verify:"
echo "  1. SPI configuration sequence"
echo "  2. Digital main startup after SPI config"
echo "  3. DCO frequency control"
echo "  4. Phase error convergence"
echo "  5. Dynamic reconfiguration at 2ms"