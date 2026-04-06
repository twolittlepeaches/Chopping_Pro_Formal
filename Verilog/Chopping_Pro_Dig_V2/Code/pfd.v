`timescale 1ps/1fs
module pfd (
    input  wire REF,     // Reference clock
    input  wire FB,      // Feedback clock
    input  wire RST_N,   // Active low reset
    output wire UP,      // UP signal
    output wire DOWN     // DOWN signal
);
    // Internal registers
    reg up_ff;           // UP flip-flop
    reg down_ff;         // DOWN flip-flop
    wire reset_internal; // Internal reset signal
    
    // Internal reset generation
    // Reset when both UP and DOWN are high
    assign #200 reset_internal = up_ff & down_ff;
    
    // UP flip-flop
    // Set on REF rising edge
    always @(posedge REF or negedge RST_N or posedge reset_internal) begin
        if (!RST_N) begin
            up_ff <= 1'b0;
        end else if (reset_internal) begin
            up_ff <= 1'b0;
        end else begin
            up_ff <= 1'b1;  // Set on REF rising edge
        end
    end
    
    // DOWN flip-flop  
    // Set on FB rising edge
    always @(posedge FB or negedge RST_N or posedge reset_internal) begin
        if (!RST_N) begin
            down_ff <= 1'b0;
        end else if (reset_internal) begin
            down_ff <= 1'b0;
        end else begin
            down_ff <= 1'b1;  // Set on FB rising edge
        end
    end
    
    // Output assignment
    assign UP = up_ff;
    assign DOWN = down_ff;
endmodule
