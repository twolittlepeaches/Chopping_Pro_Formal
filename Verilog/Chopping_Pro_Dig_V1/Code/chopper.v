
`timescale 1ps / 1fs

module chopper #(
    parameter real DELAY = 30.0  // Delay in picoseconds, default 30ps
)(
    // Power supply (inout type)
    input wire VDD_0V9,
    input wire VSS_0V9,
    
    // Input signals
    input  wire IN0,              // FREF
    input  wire IN1,              // MMD_MUX_OUT
    input  wire CHOPPER_SEL,      // Chopper selection control
    
    // Output signals
    output wire OUT0,             // MAIN_DTC_clk_ref
    output wire OUT1              // OFFSET_DTC_clk_ref
);

    // Chopper multiplexing with shared delay parameter
    assign #(DELAY) OUT0 = CHOPPER_SEL ? IN1 : IN0;
    assign #(DELAY) OUT1 = CHOPPER_SEL ? IN0 : IN1;

endmodule