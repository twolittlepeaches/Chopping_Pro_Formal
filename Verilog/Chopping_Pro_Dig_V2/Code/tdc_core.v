`timescale 1ps/1fs

module tdc_core #(
    parameter TDC_BITS = 8,                    // 8-bit output
    parameter DELAY_RESOLUTION = 0.25,         // 0.25ps resolution
    parameter MEASUREMENT_RANGE = 64.0,        // 64ps measurement range (0~64ps)
    parameter OUTPUT_DELAY = 100.0             // 100ps delay after stop to update output
)(
    input  wire start,
    input  wire stop,
    output reg [TDC_BITS-1:0] tdc_out
);
    
    // Internal parameters
    localparam MAX_CODE = (2**TDC_BITS) - 1;  // 255 for 8-bit
    
    // Time capture variables
    real start_rise_time;
    real start_pulse_width;
    real stop_rise_time;
    real stop_fall_time;
    real stop_pulse_width;
    real time_diff;
    integer tdc_code;
    
    // Capture start rising edge
    always @(posedge start) begin
        start_rise_time = $realtime;
    end
    
    // Capture stop rising edge
    always @(posedge stop) begin
        stop_rise_time = $realtime;
    end
    
    // Process measurement on stop falling edge
    always @(negedge stop) begin
        stop_fall_time = $realtime;
        
        // Calculate pulse widths
        // Use stop falling edge as approximation for start falling edge
        start_pulse_width = stop_fall_time - start_rise_time;
        stop_pulse_width = stop_fall_time - stop_rise_time;
        
        // Schedule output update after OUTPUT_DELAY
        #(OUTPUT_DELAY) begin
            // Calculate time difference: start pulse width - stop pulse width
            time_diff = start_pulse_width - stop_pulse_width;
            
            // Convert to TDC codes
            tdc_code = $rtoi(time_diff / DELAY_RESOLUTION);
            
            // Apply range limits
            if (tdc_code < 0) begin
                tdc_out <= 0;  // Below 0ps
            end else if (tdc_code > MAX_CODE) begin
                tdc_out <= MAX_CODE;  // Above max range
            end else begin
                tdc_out <= tdc_code[TDC_BITS-1:0];
            end
        end
    end
    
    // Initialize
    initial begin
        tdc_out = 0;
        start_rise_time = 0;
        stop_rise_time = 0;
        stop_fall_time = 0;
        start_pulse_width = 0;
        stop_pulse_width = 0;
    end
    
    
endmodule
