
`timescale 1ps/1fs

module TDC #(
    parameter TDC_BITS = 8                    // 8-bit output
)(
    input  wire fref,
    input  wire fdiv,
    input  wire TDC_EN,
    output wire [TDC_BITS-1:0] TDC
    /*output wire clkc,
    output wire valid,
    output wire VP,
    output wire VN,
    output wire trigger,
    input wire VDD,
    input wire VDD_REF,
    input wire VSS,
    input wire IREF,
    input wire Vrefp,
    input wire Vrefn,
    input wire Vcm*/
);

//--------Variable Mapping------//
wire [TDC_BITS-1:0] tdc_out;
wire start;
wire stop;
wire RST_N;
assign start = fref;
assign stop = fdiv;
assign TDC = tdc_out;
assign RST_N = 1'b1;

wire up_out;
wire down_out;


pfd pfd_inst (
        .REF(start),
        .FB(stop),
        .RST_N(RST_N),
        .UP(up_out),
        .DOWN(down_out)
    );
// TDC Core instantiation, which includes Charge Pump and ADC.
tdc_core #(
    .TDC_BITS(TDC_BITS),              // 8-bit output
    .DELAY_RESOLUTION(0.25),   // 0.25ps resolution  
    .MEASUREMENT_RANGE(64.0),  // 64ps measurement range
    .OUTPUT_DELAY(100.0)       // 100ps output delay
) tdc_inst (
    .start(up_out),            // Connect to up_out signal
    .stop(down_out),           // Connect to down_out signal  
    .tdc_out(tdc_out)       // 8-bit TDC output
);




endmodule
//`timescale 1ps/1fs
/*
//================================================================
//  TDC Top + PDF + TDC Core ()
//    analog delay, AMS
//================================================================
module TDC #(
parameter TDC_BITS = 8,         // TDC output bits
parameter DELAY_RESOLUTION = 0.25, // 0.25ps resolution
parameter MEASUREMENT_RANGE = 64.0, // 64ps max range
parameter OUTPUT_DELAY = 100.0   // 100ps output update delay
)(
input  wire fref,
input  wire fdiv,
input  wire TDC_EN,
output reg  [TDC_BITS-1:0] TDC,  // Final TDC output
output wire clkc,
output wire valid,
output wire VP,
output wire VN,
output wire trigger,
input  wire VDD,
input  wire VDD_REF,
input  wire VSS,
input  wire IREF,
input  wire Vrefp,
input  wire Vrefn,
input  wire Vcm
);

//================================================================
// Internal Node Mapping & Constants
//================================================================
wire start = fref;
wire stop = fdiv;
wire RST_N = 1'b1;
wire up,dn;
localparam MAX_CODE = (2**TDC_BITS) - 1;

// Unused outputs (tie off to avoid AMS warnings)
assign clkc    = 1'b0;
assign valid   = 1'b1;
assign VP      = 1'b0;
assign VN      = 1'b0;
assign trigger = 1'b0;

//================================================================
// PFD (Event-Driven Implementation)
//================================================================
reg up_ff, down_ff;
wire reset_flag;
assign #200 reset_flag = down_ff & up_ff;

// up_ff: set on start rising edge, clear on reset
always @(posedge start or negedge RST_N or posedge reset_flag) begin
  if (!RST_N)
    up_ff <= 1'b0;
  else if (!reset_flag)begin
    up_ff <= 1'b1;end
  else begin
    up_ff <= 1'b0;end
end

always @(posedge stop or negedge RST_N or posedge reset_flag) begin
  if (!RST_N)
    down_ff <= 1'b0;
  else if (!reset_flag)begin
    down_ff <= 1'b1;end
  else begin
    down_ff <= 1'b0;end
end

wire up_out  = up_ff;
wire down_out = down_ff;

//================================================================
// TDC Core (Event-Driven Implementation)
//================================================================
real start_time;
real stop_time;
real time_diff;
integer tdc_code;

// Capture start rising edge timestamp (only on up_out edge)
always @(posedge up_out) begin
  if (TDC_EN) begin
    start_time <= $realtime;
  end
end

always @(posedge down_out) begin
  if (TDC_EN) begin
    stop_time <= $realtime;
  end
end

// Event-driven measurement: trigger on stop falling edge
// Compute and update output immediately (no #delay blocking)
always @(negedge down_out or negedge up_out) begin
  if (TDC_EN) begin
    time_diff = (stop_time - start_time);

    // Convert to TDC code with clipping
    tdc_code = ($rtoi(time_diff / DELAY_RESOLUTION))/2;
    tdc_code = tdc_code + 127;

    if (tdc_code < 0) begin
      TDC <= 0;
    end else if (tdc_code > MAX_CODE) begin
      TDC <= MAX_CODE;
    end else begin
      TDC <= tdc_code[TDC_BITS-1:0];
    end
  end
end

//================================================================
// Initialize
//================================================================
initial begin
  TDC = 127; // *****:
  up_ff = 0;
  down_ff = 0;
  start_time = 0.0;
  stop_time = 0.0;
  time_diff = 0.0;
end
endmodule*/