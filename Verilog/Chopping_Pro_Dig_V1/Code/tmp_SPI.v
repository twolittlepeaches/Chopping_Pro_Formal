`timescale 1ps/1fs
module tmp_SPI(FCW_I,FCW_F,TDC_OFT,INVKDTC_MANUAL,OTW_PVT,
ALPHA,RHO,INVKDCO,TUNE_I_EN,TDC_OUT2SPI_FREEZE,DTCQ_SD_EN,OTW_MANUAL_EN,OTW_MANUAL,
BB_EN,DCO_SD_EN,MAIN_DTC_TEST_EN,TDC_EN,DCO_DEM_EN,CAL_INVKDTC_EN,CAL_TDV2_EN,CAL_OFTDTC_EN,TDV2_EN,CHOPPER_EN,
DTC_TEST_P1,DTC_TEST_P2,DTC_TEST_DCW1,DTC_TEST_DCW2,DTC_TEST_EDGE_SEL,DTC_TEST_MODE,
INVKDTC_OUT2SPI_FREEZE,
Tvd2_mis_OUT2SPI_FREEZE,
OFTDTC_OUT2SPI_FREEZE,
INVKDTC_OUT2SPI,
Tvd2_mis_OUT2SPI,
OFTDTC_OUT2SPI,
TDC_TEST_MODE,
OFFSET_DTC_CTRL_manual,
DTC_CTRL_manual,
RSV0,RSV1);

output [4:0] FCW_I;
output [11:0] FCW_F;
output [7:0] TDC_OFT;
output [11:0] INVKDTC_MANUAL;
output [5:0] OTW_PVT;
output [3:0] ALPHA;
output [4:0] RHO;
output [7:0] INVKDCO;
output TUNE_I_EN;
output TDC_OUT2SPI_FREEZE;
output DTCQ_SD_EN;
output OTW_MANUAL_EN;
output [12:0] OTW_MANUAL;
output BB_EN;
output        DCO_SD_EN;
output CHOPPER_EN;
output MAIN_DTC_TEST_EN;
output TDC_EN;
output DCO_DEM_EN;
output CAL_INVKDTC_EN;
output CAL_TDV2_EN;
output CAL_OFTDTC_EN;
output TDV2_EN;

output [7:0] DTC_TEST_P1;
output [7:0] DTC_TEST_P2;
output [9:0] DTC_TEST_DCW1;
output [9:0] DTC_TEST_DCW2;
output DTC_TEST_EDGE_SEL;
output DTC_TEST_MODE;

  output INVKDTC_OUT2SPI_FREEZE;
  output Tvd2_mis_OUT2SPI_FREEZE;
  output OFTDTC_OUT2SPI_FREEZE;
  input [11:0] INVKDTC_OUT2SPI;
  input [11:0] Tvd2_mis_OUT2SPI;
  input [11:0] OFTDTC_OUT2SPI;

output TDC_TEST_MODE;
output [9:0] OFFSET_DTC_CTRL_manual;
output [9:0] DTC_CTRL_manual;
output [7:0] RSV0;
output [7:0] RSV1;

reg DCO_SD_EN;
reg DPD_EN;
reg CHOPPER_EN;
reg TDV2_EN;
reg CAL_INVKDTC_EN;
reg CAL_TDV2_EN;
reg CAL_OFTDTC_EN;
reg TDC_OUT2SPI_FREEZE;

assign FCW_I = 5'd10; // Take 4.25GHz as the typical test frequency.
assign FCW_F = 16'b0000_0100_0000;//1000_0100_0000,test1: 4'b0000_0100_0000 for 0.25GHz
assign TDC_OFT = 8'b1000_0000;
assign INVKDTC_MANUAL = 12'd1881;//1881
assign OTW_PVT = 6'd20;
assign ALPHA = 4'd3;
assign RHO   = 5'd8;
assign INVKDCO = 8'd120;
assign TUNE_I_EN = 1'b1;
//assign TDC_OUT2SPI_FREEZE = 1'b1;
assign DTCQ_SD_EN = 1'b0;
assign OTW_MANUAL_EN = 1'b0;
assign OTW_MANUAL = 13'd0;
assign BB_EN = 1'b1;
assign TDC_EN = 1'b1;
assign DCO_DEM_EN = 1'b0;
assign DTC_TEST_P1 = 8'd20;
assign DTC_TEST_P2 = 8'd20;
assign DTC_TEST_DCW1 = 10'd2;
assign DTC_TEST_DCW2 = 10'd3;
assign DTC_TEST_EDGE_SEL = 1'b0;
assign DTC_TEST_MODE = 1'b0;
assign INVKDTC_OUT2SPI_FREEZE = 1'b1;
assign Tvd2_mis_OUT2SPI_FREEZE = 1'b1;
assign OFTDTC_OUT2SPI_FREEZE = 1'b1;
assign TDC_TEST_MODE = 1'b0;
assign OFFSET_DTC_CTRL_manual = 10'd100;//100
assign DTC_CTRL_manual = 10'd50;

initial begin
	DCO_SD_EN <= 1'b0;
	CHOPPER_EN <= 1'b0;
        CAL_INVKDTC_EN <= 1'b0;
        TDV2_EN <= 1'b0;
        CAL_TDV2_EN <= 1'b0;
        CAL_OFTDTC_EN <= 1'b0;
        TDC_OUT2SPI_FREEZE<=1'b1;
	//#10_000_000 DCO_SD_EN <= 1'b1;
       // #100_000_000 TDC_OUT2SPI_FREEZE<=1'b0;
	#130_000_000 ;
        TDV2_EN <= 1'b0;//swap
	CHOPPER_EN <= 1'b0;
        CAL_INVKDTC_EN <= 1'b1;
        CAL_TDV2_EN <= 1'b0;//(if cal tdv2 and oftdtc,must en chopper,else not lock)&&chopper can not be en alone!!!
        CAL_OFTDTC_EN <= 1'b0;
end

endmodule
