`timescale 1ps/1fs

module TDC #(
    parameter TDC_BITS = 8
)(
    input  wire fref,
    input  wire fdiv,
    input  wire TDC_EN,
    output reg signed [TDC_BITS-1:0] TDC,
    output wire clkc,
    output reg  valid,
    output wire VP,
    output wire VN,
    output wire trigger,
    input wire VDD,
    input wire VDD_REF,
    input wire VSS,
    input wire IREF,
    input wire Vrefp,
    input wire Vrefn,
    input wire Vcm
);

    assign clkc = fref;
    integer code;  

    // real-valued time stamps
    real t_ref;
    real t_div;
    real dt_ps;

    // conversion scale (±65ps → ±127)
    real scale;

    initial begin
        TDC   = 0;
        valid = 0;
        t_ref = 0.0;
        t_div = 0.0;
        dt_ps = 0.0;
        scale = 127.0 / 65.0;
    end

    // capture REF rising edge
    always @(posedge fref) begin
        if (TDC_EN)
            t_ref = $realtime;
    end

    // capture DIV rising edge and compute Δt
    always @(posedge fdiv) begin
        if (TDC_EN) begin
            t_div = $realtime;

            dt_ps = t_div - t_ref;  // can be negative

            // outside valid measurement range
            if (dt_ps > 65.0 || dt_ps < -65.0) begin
                TDC   <= 0;
                valid <= 0;
            end
            else begin   
                code = $rtoi(dt_ps * scale);
                TDC   <= code[7:0]; // 8-bit signed
                valid <= 1;
            end
        end
    end

endmodule
