`timescale 1ps/1fs

module top_sim_tb();

wire FREF;
wire rst_n;

wire MAIN_DTC_clk_delayi;
wire VD,VS;
wire CKVS;
wire [7:0] TDC_Q;
wire TDC_CALIB_EN;
wire ckv;
wire CKR;
wire [5:0] OTW_PVT;
wire [62:0] OTW_T_H;
wire [126:0] OTW_T_L;
wire [7:0] DCO_sdm;
wire CLK_SD;
wire [4:0] OTW_SD;
wire DCO_SD_EN_Q;
wire CHOPPER_SEL;
wire CHOPPER_EN;
reg MAIN_DTC_DIV_OUT;
wire MAIN_DTC_TEST_OUTPUT;
wire TDC_EN;
wire ckvd;
wire DCO_DEM_EN;
wire CAL_INVKDTC_EN;
wire [7:0] RSV0;
wire [7:0] RSV1;

wire [7:0] DTC_TEST_P1;
wire [7:0] DTC_TEST_P2;
wire [9:0] DTC_TEST_DCW1;
wire [9:0] DTC_TEST_DCW2;
wire DTC_TEST_EDGE_SEL;
wire DTC_TEST_MODE;

wire INVKDTC_OUT2SPI_FREEZE;
wire Tvd2_mis_OUT2SPI_FREEZE;
wire OFTDTC_OUT2SPI_FREEZE;
wire [11:0] INVKDTC_OUT2SPI;
wire [11:0] Tvd2_mis_OUT2SPI;
wire [11:0] OFTDTC_OUT2SPI;
wire [9:0] OFFSET_DTC_CTRL_manual;
wire [9:0] DTC_CTRL_manual;


FREF_generator #(.Fr(100e6)) FREF_generator_inst(FREF,rst_n);

DCO DCO_inst (
    .OTW_C(OTW_PVT),
    .OTW_M1({1'b0,OTW_T_H[62:48]}), .OTW_M2(OTW_T_H[47:32]), .OTW_M3(OTW_T_H[31:16]), .OTW_M4(OTW_T_H[15:0]),
    .OTW_F1({1'b0,OTW_T_L[126:96]}), .OTW_F2(OTW_T_L[95:64]), .OTW_F3({OTW_T_L[63:32]}), .OTW_F4(OTW_T_L[31:0]),
    .OTW_sd(DCO_sdm),
    .ckv(ckv),
    .VD(VD), .VS(VS)
);


sd_mod sd_mod_inst(CLK_SD,DCO_SD_EN_Q,OTW_SD,DCO_sdm);

// CLK_SD generation from divider by 8.
reg [2:0] cnt_ckv;
initial begin cnt_ckv <= 3'd0; end
always @(posedge ckv) begin
    cnt_ckv <= cnt_ckv + 1'b1;
end
assign CLK_SD = cnt_ckv[2];

wire [4:0] FCW_I;
wire [11:0] FCW_F;
wire [7:0] TDC_OFT;
wire DTCQ_SD_EN;
wire BB_EN;
wire [11:0] INVKDTC_MANUAL;
wire [7:0] TDC_OUT2SPI;
wire TDC_OUT2SPI_FREEZE;
wire [7:0] INVKDCO;
wire [3:0] ALPHA;
wire [4:0] RHO;
wire TUNE_I_EN;
wire [12:0] OTW_MANUAL;
wire OTW_MANUAL_EN;
wire DCO_SD_EN;
wire MAIN_DTC_TEST_EN;
wire TDV2_EN;
wire TDC_TEST_MODE;

tmp_SPI tmp_SPI_inst(FCW_I,FCW_F,TDC_OFT,INVKDTC_MANUAL,
OTW_PVT,ALPHA,RHO,INVKDCO,TUNE_I_EN,TDC_OUT2SPI_FREEZE,DTCQ_SD_EN,OTW_MANUAL_EN,
OTW_MANUAL,BB_EN,DCO_SD_EN,MAIN_DTC_TEST_EN,
TDC_EN,DCO_DEM_EN,CAL_INVKDTC_EN,CAL_TDV2_EN,CAL_OFTDTC_EN,TDV2_EN,CHOPPER_EN,
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



//modification starts from here
wire [4:0] FCW;
wire [4:0] MMD_RN;
wire [7:0] tdc_result;
wire up_out;
wire down_out;
wire MMD_PHASE;
assign FCW = FCW_I;
mmd #(
    .WD(10)
) mmd_inst (
    .MMD_MUX_OUT(ckvd),
    .CLK(ckv),
    .CLKN(),
    .EN_MMD_MUX(1'b1),
    .SEL_PHASE(MMD_PHASE),
    .Rn(MMD_RN),
    .VDD_0p9(VD),
    .VSS_0p9(VS)
);
wire [3:0] DTC_top_4b;
wire [62:0] DTC_top_6u;
wire [14:0] DTC_bot_4u;
wire [3:0] OFFSET_DTC_top_4b;
wire [62:0] OFFSET_DTC_top_6u;
wire [14:0] OFFSET_DTC_bot_4u;
wire MAIN_DTC_clk_ref;
wire OFFSET_DTC_clk_ref;
wire OFFSET_DTC_clk_delay;


digital_main digital_main_inst (
    .CKR                    (CKR),
    .RST_N_SPI          (rst_n),
    .FCW_I                  (FCW_I),
    .FCW_F                  (FCW_F),
    .TDC_Q                  (tdc_result),
    .TDC_OFT                (TDC_OFT),
    .DTCQ_SD_EN             (DTCQ_SD_EN),
    .BB_EN                  (BB_EN),
    .INVKDTC_MANUAL         (INVKDTC_MANUAL),
    .TDC_OUT2SPI            (TDC_OUT2SPI),
    .TDC_OUT2SPI_FREEZE     (TDC_OUT2SPI_FREEZE),
    .INVKDCO                (INVKDCO),
    .ALPHA                  (ALPHA),
    .RHO                    (RHO),
    .TUNE_I_EN              (TUNE_I_EN),
    .OTW_T_H                (OTW_T_H),
    .OTW_T_L                (OTW_T_L),
    .OTW_MANUAL_EN          (OTW_MANUAL_EN),
    .OTW_MANUAL             (OTW_MANUAL),
    .OTW_SD                 (OTW_SD),
    .DCO_SD_EN              (DCO_SD_EN),
    .DCO_SD_EN_Q            (DCO_SD_EN_Q),
    .DCO_DEM_EN             (DCO_DEM_EN),
    .DTC_top_4b             (DTC_top_4b),
    .DTC_top_6u             (DTC_top_6u),
    .DTC_bot_4u             (DTC_bot_4u),
    .OFFSET_DTC_top_4b      (OFFSET_DTC_top_4b),
    .OFFSET_DTC_top_6u      (OFFSET_DTC_top_6u),
    .OFFSET_DTC_bot_4u      (OFFSET_DTC_bot_4u),
    .MAIN_DTC_TEST_EN       (MAIN_DTC_TEST_EN),
    .SWAP_EN                (CHOPPER_SEL),
    .CHOPPER_EN             (CHOPPER_EN),
    .CAL_INVKDTC_EN         (CAL_INVKDTC_EN),
    .CAL_TDV2_EN            (CAL_TDV2_EN),
    .CAL_OFTDTC_EN          (CAL_OFTDTC_EN),
    .MMD_PHASE_Q            (MMD_PHASE),
    .TDV2_EN                (TDV2_EN),
    .MMD_RN                 (MMD_RN),
    .DTC_TEST_P1            (DTC_TEST_P1),
    .DTC_TEST_P2            (DTC_TEST_P2),
    .DTC_TEST_DCW1          (DTC_TEST_DCW1),
    .DTC_TEST_DCW2          (DTC_TEST_DCW2),
    .DTC_TEST_EDGE_SEL      (DTC_TEST_EDGE_SEL),
    .DTC_TEST_MODE          (DTC_TEST_MODE),
    .INVKDTC_OUT2SPI_FREEZE(INVKDTC_OUT2SPI_FREEZE),
    .Tvd2_mis_OUT2SPI_FREEZE(Tvd2_mis_OUT2SPI_FREEZE),
    .OFTDTC_OUT2SPI_FREEZE(OFTDTC_OUT2SPI_FREEZE),
    .INVKDTC_OUT2SPI(INVKDTC_OUT2SPI),
    .Tvd2_mis_OUT2SPI(Tvd2_mis_OUT2SPI),
    .OFTDTC_OUT2SPI(OFTDTC_OUT2SPI),
    .TDC_TEST_MODE(TDC_TEST_MODE),
    .OFFSET_DTC_CTRL_manual(OFFSET_DTC_CTRL_manual),
    .DTC_CTRL_manual(DTC_CTRL_manual),
    .RSV0                   (RSV0),
    .RSV1                   (RSV1)
);


TDC #(
    .TDC_BITS(8)
) TDC_inst (
    .fref(MAIN_DTC_clk_delayi),
    .fdiv(OFFSET_DTC_clk_delay),
    .TDC_EN(TDC_EN),
    .TDC(tdc_result)
);


DTC_analog_10b #(
    .sigma_DNL(1),        // 1ps DNL
    .reso_binary(0.5),    // 0.5ps per binary LSB
    .reso_unary(8),       // 8ps per unary bit
    .base_delay(500)      // 500ps base delay
) DTC_analog_10b_inst (
    .vdd_ref(VD),
    .vdd(VD),
    .gnd(VS),
    .DTC_DL_cap_ctrl(5'd0),
    .DTC_Iref(1'b1),
    .DTC_vtop_tran(1'b1),
    .DTC_clk_ref(MAIN_DTC_clk_ref),
    .DTC_top_cap_ctrl_4bit_binary(DTC_top_4b),
    .DTC_top_cap_ctrl_6bit_unary(DTC_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary(DTC_bot_4u),
    .DTC_pre_clk_test(VD),
    .DTC_pre_clk_sel(VS),
    .OUT(MAIN_DTC_clk_delay)
);

assign OFFSET_DTC_in = TDC_TEST_MODE ? MAIN_DTC_clk_ref : OFFSET_DTC_clk_ref;

DTC_analog_10b #(
    .sigma_DNL(1),        // 1ps DNL
    .reso_binary(0.5),    // 0.5ps per binary LSB
    .reso_unary(8),       // 8ps per unary bit
    .base_delay(500)      // 500ps base delay
) Offset_DTC_analog_10b_inst (
    .vdd_ref(VD),
    .vdd(VD),
    .gnd(VS),
    .DTC_DL_cap_ctrl(5'd0),
    .DTC_Iref(1'b1),
    .DTC_vtop_tran(1'b1),
    .DTC_clk_ref(OFFSET_DTC_in),
    .DTC_top_cap_ctrl_4bit_binary(OFFSET_DTC_top_4b),
    .DTC_top_cap_ctrl_6bit_unary(OFFSET_DTC_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary(OFFSET_DTC_bot_4u),
    .DTC_pre_clk_test(1'b0),
    .DTC_pre_clk_sel(1'b0),
    .OUT(OFFSET_DTC_clk_delay)
);


chopper #(
    .DELAY(50.0) 
) chopper_inst (
    .VDD_0V9(VD),
    .VSS_0V9(VS),    
    .IN0(FREF),
    .IN1(ckvd),
    .CHOPPER_SEL(CHOPPER_SEL),    
    .OUT0(MAIN_DTC_clk_ref),
    .OUT1(OFFSET_DTC_clk_ref)
);

assign CKR = DTC_TEST_MODE ? FREF : ckvd;


// DTC DEMUX, select whether main DTC output is to TDC or PAD
assign MAIN_DTC_clk_delayi = MAIN_DTC_TEST_EN ? 1'b0 : MAIN_DTC_clk_delay;
assign MAIN_DTC_DEMUX_TEST = MAIN_DTC_TEST_EN ? MAIN_DTC_clk_delay : 1'b0;
initial begin MAIN_DTC_DIV_OUT <= 1'b0; end
always @(posedge MAIN_DTC_DEMUX_TEST) begin MAIN_DTC_DIV_OUT <= ~MAIN_DTC_DIV_OUT; end
assign MAIN_DTC_TEST_OUTPUT = MAIN_DTC_DIV_OUT;



endmodule
