`timescale 1ps/1fs

module Tracking #(
		parameter W_PHEi=4,
		parameter W_PHEf=8,
		parameter W_INVKDCO=8,
		parameter W_OTW_T=13
		)
(RST_N,CKR,PHE,INVKDCO,ALPHA,RHO,TUNE_I_EN,OTW_T_H,OTW_T_L,OTW_MANUAL_EN,OTW_MANUAL,OTW_SD,DCO_SD_EN,DCO_SD_EN_Q,DCO_DEM_EN);
//The integer+fractional parts should be referred from OTW_T, or from INVKDCO. Should match.
input RST_N;
input CKR;
input [W_PHEi+W_PHEf-1:0] PHE;//(1+3,8)
input [W_INVKDCO-1:0] INVKDCO;
input [3:0] ALPHA;
input [4:0] RHO;
input TUNE_I_EN;
input OTW_MANUAL_EN;
input [W_OTW_T-1:0] OTW_MANUAL;
output [62:0] OTW_T_H;
output [126:0] OTW_T_L;
output [4:0] OTW_SD;
input DCO_SD_EN;
output DCO_SD_EN_Q;
input DCO_DEM_EN;

reg DCO_SD_EN_Q;
wire [W_OTW_T-1:0] OTW_T_tmp;
wire [63:0] OTW_T_Htmp;
wire [127:0] OTW_T_Ltmp;
reg [126:0] OTW_T_L;
reg [62:0] OTW_T_H;
reg [4:0] OTW_SD;
wire [W_OTW_T-1:0] OTW_T_binary;
wire signed [W_PHEi+W_PHEf-1:0] PHE;
wire signed [W_PHEi+W_PHEf+W_INVKDCO-1:0] tune_in;

//---------------------DCO gain normalization--------------------
//INVKDCO = fr/KDCO/2^4
assign tune_in= PHE*$signed({1'b0,INVKDCO});//(1+12,7) = (1+0,11)*(1+12,-4) This integer part will not exceed 7 effective number bits, Once '(-2)*(-4)' not happen, cutting one bit in advance has no issue.

wire signed [W_PHEi+W_PHEf+W_INVKDCO+3:0] TUNE_PI_in;
wire signed [W_PHEi+W_PHEf+W_INVKDCO+3:0] TUNE_P;
wire signed [W_PHEi+W_PHEf+W_INVKDCO+3:0] tune_RHO;
wire signed [W_PHEi+W_PHEf+W_INVKDCO+4:0] TUNE_I_add;
wire 		[W_PHEi+W_PHEf+W_INVKDCO+3:0] TUNE_I_D;
reg  signed [W_PHEi+W_PHEf+W_INVKDCO+3:0] TUNE_I;
wire signed [W_PHEi+W_PHEf+W_INVKDCO+5:0] TUNE_OUT;//(1+15,8)
wire POS_TUNEI_OV;
wire NEG_TUNEI_OV;
wire POS_TUNEOUT_OV;
wire NEG_TUNEOUT_OV;
assign TUNE_PI_in=$signed({tune_in,{4{1'b0}}});//(1+12,11)
assign TUNE_P = TUNE_PI_in >>> ALPHA;//(1+12,11)
assign tune_RHO = TUNE_PI_in >>> RHO;//(1+12,11)
assign TUNE_I_add = TUNE_I+tune_RHO;//(1+13,11)
assign POS_TUNEI_OV = (~TUNE_I_add[24])&(TUNE_I_add[23]);//There are two ways for ov. One is to detect the adders and sum; another is to expand the sum range by extra 1 bit.
assign NEG_TUNEI_OV = (TUNE_I_add[24])&(~TUNE_I_add[23]);//
assign TUNE_I_D = TUNE_I_EN?(POS_TUNEI_OV?{1'b0,{23{1'b1}}}:(NEG_TUNEI_OV?{1'b1,{23{1'b0}}}:TUNE_I_add)):TUNE_I;
always @(posedge CKR, negedge RST_N) begin
	if(~RST_N) begin
		TUNE_I <= {(W_PHEi+W_PHEf+W_INVKDCO+3){1'b0}};
	end else begin
		TUNE_I <= $signed(TUNE_I_D);
	end
end
assign TUNE_OUT = $signed(TUNE_P)+$signed(TUNE_I_D);
assign POS_TUNEOUT_OV = (~TUNE_OUT[25])&(|TUNE_OUT[24:23]);//(1+14,11) --> (1+12,11)
assign NEG_TUNEOUT_OV = (TUNE_OUT[25])&(|(~TUNE_OUT[24:23]));

assign OTW_T_tmp[W_OTW_T-1] = POS_TUNEOUT_OV?(1'b0):(NEG_TUNEOUT_OV?(1'b1):TUNE_OUT[25]);
assign OTW_T_tmp[W_OTW_T-2:0] = POS_TUNEOUT_OV?({12{1'b0}}):(NEG_TUNEOUT_OV?({12{1'b1}}):(~TUNE_OUT[22:11]));
assign OTW_T_binary = OTW_MANUAL_EN?OTW_MANUAL:OTW_T_tmp;

assign OTW_T_Htmp = {64 {1'b1}} >> (7'd64-OTW_T_binary[12:7]);
always @(posedge CKR, negedge RST_N) begin
	if (~RST_N) begin
		OTW_T_H <= 63'd0;
	end else begin
		OTW_T_H <= OTW_T_Htmp[62:0];
	end
end
assign OTW_T_Ltmp = {128 {1'b1}} >> (8'd128-OTW_T_binary[6:0]);
always @(posedge CKR, negedge RST_N) begin
	if (~RST_N) begin
		OTW_T_L <= 127'd0;
	end else begin
		OTW_T_L <= OTW_T_Ltmp[126:0];
	end
end

always @(posedge CKR, negedge RST_N) begin
	if (~RST_N) begin
		OTW_SD <= 5'd0;
		DCO_SD_EN_Q <= 1'b0;
	end else begin
		if (~OTW_MANUAL_EN) begin
			OTW_SD <= {5{DCO_SD_EN}}&(~TUNE_OUT[10:6]);
			DCO_SD_EN_Q <= DCO_SD_EN;
		end
	end
end 
endmodule
