// DTC Analog DFT Control Module
// Periodically switches between two DTC control words (d1 and d2)
`timescale 1ps/1fs

module dtc_dft_ctrl(
  output reg [9:0] out_code,  // 10-bit DTC control code output
  input [7:0] p1,             // Number of cycles for d1
  input [7:0] p2,             // Number of cycles for d2
  input [9:0] d1,             // First DTC control word
  input [9:0] d2,             // Second DTC control word
  input clk,                  // Clock input
  input edge_sel,             // Edge selection: 0=posedge, 1=negedge
  input rstn,                 // Active low reset
  input dtc_test_mode         // Test mode enable: 0=disable, 1=active
);

  // Internal signals
  reg [8:0] cnt;              // 9-bit counter (max 512 cycles)
  wire [8:0] psum;            // Sum of p1 and p2
  wire clk2;                  // Selected clock edge
  
  // Clock edge selection
  assign clk2 = edge_sel ? ~clk : clk;
  
  // Period sum
  assign psum = p1 + p2;
  
  // Main control logic
  always @(posedge clk2 or negedge rstn) begin
    if (~rstn) begin
      out_code <= 10'd0;
      cnt <= 9'd0;
    end else begin
      if (dtc_test_mode) begin
        // Test mode active: normal operation
        if (cnt < (p1 - 1)) begin
          // Output d1 for p1 cycles
          out_code <= d1;
          cnt <= cnt + 1'b1;
        end else if (cnt < (psum - 1)) begin
          // Output d2 for p2 cycles
          out_code <= d2;
          cnt <= cnt + 1'b1;
        end else begin
          // Reset counter and restart with d1
          cnt <= 9'd0;
          out_code <= d1;
        end
      end
      // else: dtc_test_mode = 0, keep out_code and cnt unchanged (no toggling)
    end
  end

endmodule