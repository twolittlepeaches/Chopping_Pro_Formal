module DTC_decoder #(
    parameter N = 4
) (
    input  [N-1:0] data_in,
    output [2**N-2:0] data_out
);

genvar i;
generate
    for(i = 0; i < 2**N-1; i = i+1) begin : gen_thermometer
        assign data_out[i] = (data_in > i);
    end
endgenerate

endmodule