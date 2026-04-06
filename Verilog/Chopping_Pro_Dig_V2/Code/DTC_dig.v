
module DTC_dig(
    input clk,
    input DTC_rstn,
    input [9:0] DTC_PLL_code,
    output reg [3:0] DTC_top_cap_ctrl_4bit_binary,
    output reg [62:0] DTC_top_cap_ctrl_6bit_unary,
    output reg [14:0] DTC_bot_cap_ctrl_4bit_unary
);
wire [62:0] PLL_cap_ctrl_6bit_unary;
wire [14:0] PLL_cap_ctrl_4bit_unary;

always @(negedge clk or negedge DTC_rstn)begin
  if(DTC_rstn==0)begin// reset
    DTC_top_cap_ctrl_4bit_binary<=0;
    DTC_top_cap_ctrl_6bit_unary<=0;
    DTC_bot_cap_ctrl_4bit_unary<=0;
  end
  else begin// decoder write, top cap controled by 4bits binary and 63bits unary, bottom cap controled by 4bits unary
      DTC_top_cap_ctrl_4bit_binary<=DTC_PLL_code[3:0];// top cap 4bits binary = code [3:0]
      DTC_top_cap_ctrl_6bit_unary<=PLL_cap_ctrl_6bit_unary;// top cap 63bits unary decoder from code [9:4]
      DTC_bot_cap_ctrl_4bit_unary<=PLL_cap_ctrl_4bit_unary;// bottom cap 15bits unary decoder from code [9:6]
  end
end

DTC_decoder #(
    .N ( 4 ))
 u4_PLL_decoder (
    .data_in                 ( DTC_PLL_code   [9:6]    ),

    .data_out                ( PLL_cap_ctrl_4bit_unary  [14:0] )
);

DTC_decoder #(
    .N ( 6 ))
 u6_PLL_decoder (
    .data_in                 ( DTC_PLL_code   [9:4]    ),

    .data_out                ( PLL_cap_ctrl_6bit_unary  [62:0] )
);

endmodule