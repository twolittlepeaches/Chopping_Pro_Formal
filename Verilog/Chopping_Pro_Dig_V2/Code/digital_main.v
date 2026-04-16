
`timescale 1ps/1fs

module digital_main (
    CKR,
    RST_N_SPI,
    FCW_I,
    FCW_F,
    TDC_OFT,
    DTCQ_SD_EN,
    TDC_Q,
    BB_EN,
    INVKDTC_MANUAL,
    TDC_OUT2SPI,
    TDC_OUT2SPI_FREEZE,
    INVKDCO,
    ALPHA,
    RHO,
    TUNE_I_EN,
    OTW_T_H,
    OTW_T_L,
    OTW_MANUAL_EN,
    OTW_MANUAL,
    OTW_SD,
    DCO_SD_EN,
    DCO_SD_EN_Q,
    DCO_DEM_EN,
    DTC_P_top_4b,
    DTC_P_top_6u,
    DTC_P_bot_4u,
    DTC_N_top_4b,
    DTC_N_top_6u,
    DTC_N_bot_4u,
    MAIN_DTC_TEST_EN,
    Chopping_EN,
    CHOPPER_EN,
    MASH_SEL,
    DIFF_EN,
    CAL_INVKDTC_EN,
    CAL_OFTDTC_EN,
    CAL_POLY_EN,
    MMD_RN,
    DTC_TEST_P1,
    DTC_TEST_P2,
    DTC_TEST_DCW1,
    DTC_TEST_DCW2,
    DTC_TEST_EDGE_SEL,
    DTC_TEST_MODE,
    INVKDTC_OUT2SPI_FREEZE,
    OFTDTC_OUT2SPI_FREEZE,
    INVKDTC_OUT2SPI,
    OFTDTC_OUT2SPI,
    TDC_TEST_MODE,
    OFFSET_DTC_CTRL_manual,
    DTC_CTRL_manual,
    RSV0,
    RSV1
);

        input         CKR;
        input         RST_N_SPI;

        input  [4:0]  FCW_I;                   // integer FCW
        input [11:0]  FCW_F;                   // fractional FCW

        // TDC inputs / readback
        input  [7:0]  TDC_Q;                   // TDC (analog) output
        input  [7:0]  TDC_OFT;
        input         DTCQ_SD_EN;              // expand (not used)
        input         BB_EN;
        input [11:0]  INVKDTC_MANUAL;          // Inverse KDTC manual control (calib)
        output [7:0]  TDC_OUT2SPI;             // TDC output for SPI readback
        input         TDC_OUT2SPI_FREEZE;      // freeze SPI readback

        // DCO control interface
        input  [7:0]  INVKDCO;                 // Inverse DCO gain (fr/KDC0/2^4)
        input  [3:0]  ALPHA;                   // proportional gain parameter
        input  [4:0]  RHO;                     // integral gain parameter
        input         TUNE_I_EN;               // integral path enable
        input         OTW_MANUAL_EN;           // manual OTW override enable
        input [12:0]  OTW_MANUAL;              // manual OTW value
        output [62:0] OTW_T_H;                 // DCO tuning word - high (thermometer)
        output[126:0] OTW_T_L;                 // DCO tuning word - low (thermometer)
        output  [4:0] OTW_SD;                  // DCO tuning word - sigma-delta part
        input         DCO_SD_EN;               // DCO sigma-delta enable
        output        DCO_SD_EN_Q;             // DCO sigma-delta enable (registered)
        input         DCO_DEM_EN;              // DCO dynamic element matching enable

        // DTC outputs (main)
        output  [3:0] DTC_P_top_4b;              // top caps 4-bit binary
        output [62:0] DTC_P_top_6u;              // top caps 6-bit unary
        output [14:0] DTC_P_bot_4u;              // bottom caps 4-bit unary

        // DTC outputs (offset)
        output  [3:0] DTC_N_top_4b;
        output [62:0] DTC_N_top_6u;
        output [14:0] DTC_N_bot_4u;

        // Calibration control
        input         CAL_INVKDTC_EN;          // invKDTC calibration enable
        input         CAL_OFTDTC_EN;           // Offset DTC calibration enable
        input         CAL_POLY_EN;             // Legendre POLY calibration enable (temp, no SPI yet)

        // Advanced features
        input         CHOPPER_EN;              // chopper (SWAP) enable
        output        Chopping_EN;                 // SWAP enable output
        input         MASH_SEL;                // MASH order: 0=MASH1, 1=MASH1-1
        input         DIFF_EN;                 // differential DTC: 0=SE, 1=DIFF
        output  [4:0] MMD_RN;                  // MMD division ratio

        // DTC test mode
        input         MAIN_DTC_TEST_EN;        // main DTC test enable (not used)
        input  [7:0]  DTC_TEST_P1;             // test mode period 1
        input  [7:0]  DTC_TEST_P2;             // test mode period 2
        input  [9:0]  DTC_TEST_DCW1;           // test DTC control word 1
        input  [9:0]  DTC_TEST_DCW2;           // test DTC control word 2
        input         DTC_TEST_EDGE_SEL;       // test DTC edge select
        input         DTC_TEST_MODE;           // DTC test mode enable: 0/1

        input INVKDTC_OUT2SPI_FREEZE;
        input OFTDTC_OUT2SPI_FREEZE;
        output  [11:0] INVKDTC_OUT2SPI;
        output  [11:0] OFTDTC_OUT2SPI;

        input TDC_TEST_MODE;
        input [9:0] OFFSET_DTC_CTRL_manual;
        input [9:0] DTC_CTRL_manual;
        // Reserved
        input  [7:0]  RSV0;                    // reserved input 0
        input  [7:0]  RSV1;                    // reserved input 1

  wire signed [11:0] PHE; 
  wire signed [11:0] PHE_CORR;   // poly-corrected phase error
  wire signed [15:0] ALPHA_ODD;  // POLY Legendre coefficient (monitor)
  wire [11:0] INVKDTC_CAL;
  wire [11:0] INVKDTC;
  wire [9:0]  DTC_P_CTRL_10b;
  wire [9:0]  DTC_N_CTRL_10b; 
  wire [9:0] OFFSET_DTC_CTRL_10b;
  wire [11:0] PHR_F;
  wire PHRF_OV;


  reg [7:0] random_ini;
  wire feedback;
  reg SWAP_EN_prev;
  reg SWAP_EN_DTC;
  reg Chopping_EN;
  reg    [7:0] random_tdv2;   
  wire   feedback_tdv2;  
  reg MMD_PHASE;    
  wire [9:0] DTC_TEST_DCWout;
  wire [9:0] DTC_P_CTRL_10b_final;
  wire [9:0] DTC_N_CTRL_10b_final ;

  reg rst_sync_1;
  reg rst_sync_2;
  wire RST_N;

always @(posedge CKR or negedge RST_N_SPI) begin
        if (~RST_N_SPI) begin
            rst_sync_1 <= 1'b0;
            rst_sync_2 <= 1'b0;
        end else begin
            rst_sync_1 <= 1'b1;
            rst_sync_2 <= rst_sync_1;
        end
end
assign RST_N =  rst_sync_2;


phase_adder #(
    .W_PHE(12),
    .W_ADC(8),
    .W_DTC_CTRL(10),
    .W_FCWF(12),
    .W_INVKDTC(12)
) phase_adder_inst (
    .CKR(CKR),
    .RST_N(RST_N),
    .MASH_SEL(MASH_SEL),
    .DIFF_EN(DIFF_EN),
    .Chopping_EN(SWAP_EN_DTC),
    .TDC_Q(TDC_Q),
    .TDC_OFT(TDC_OFT),
    .FCW_F(FCW_F),
    .PHE(PHE),
    .INVKDTC(INVKDTC),
    .DTCQ_SD_EN(DTCQ_SD_EN),
    .BB_EN(BB_EN),
    .TDC_OUT2SPI_FREEZE(TDC_OUT2SPI_FREEZE),
    .TDC_OUT2SPI(TDC_OUT2SPI),
    .PHRF_OV(PHRF_OV),
    .MASH2_CARRY_P(MASH2_CARRY_P),
    .MASH2_CARRY_N(MASH2_CARRY_N),
    .DTC_P_CTRL_10b(DTC_P_CTRL_10b),
    .DTC_N_CTRL_10b(DTC_N_CTRL_10b),
    .PHR_F(PHR_F),
    .MMD_PHASE(MMD_PHASE)
);


assign MMD_RN = FCW_I + PHRF_OV + (MASH2_CARRY_P ? 5'd1:5'd0)+ (MASH2_CARRY_N ? 5'b11111 : 5'd0);

Tracking Tracking_inst(
    .RST_N(RST_N),
    .CKR(CKR),
    .PHE(PHE_CORR),  // 使用POLY补偿后的相位误差
    .INVKDCO(INVKDCO),
    .ALPHA(ALPHA),
    .RHO(RHO),
    .TUNE_I_EN(TUNE_I_EN),
    .OTW_T_H(OTW_T_H),
    .OTW_T_L(OTW_T_L),
    .OTW_MANUAL_EN(OTW_MANUAL_EN),
    .OTW_MANUAL(OTW_MANUAL),
    .OTW_SD(OTW_SD),
    .DCO_SD_EN(DCO_SD_EN),
    .DCO_SD_EN_Q(DCO_SD_EN_Q),
    .DCO_DEM_EN(DCO_DEM_EN)
);

LMS_Calib #(
    .W_PHE(12),
    .W_INVKDTC(12),
    .W_FCWF(12),
    .W_DTC_CTRL(10)
) LMS_Calib_inst (
    .CKR(CKR),
    .RST_N(RST_N),
    .PHR_F(PHR_F),
    .PHE(PHE),
    .Chopping_EN(Chopping_EN),
    .CAL_INVKDTC_EN(CAL_INVKDTC_EN),
    .CAL_OFTDTC_EN(CAL_OFTDTC_EN),
    .INVKDTC_OUT2SPI_FREEZE(INVKDTC_OUT2SPI_FREEZE),
    .OFTDTC_OUT2SPI_FREEZE(OFTDTC_OUT2SPI_FREEZE),
    .INVKDTC_OUT2SPI(INVKDTC_OUT2SPI),
    .OFTDTC_OUT2SPI(OFTDTC_OUT2SPI),
    .INVKDTC(INVKDTC_CAL),
    .OFFSET_DTC_CTRL_10b(OFFSET_DTC_CTRL_10b),
    // --- POLY新增端口 ---
    .DTC_P_CTRL_10b(DTC_P_CTRL_10b),
    .DTC_N_CTRL_10b(DTC_N_CTRL_10b),
    .DIFF_EN(DIFF_EN),
    .MASH_SEL(MASH_SEL),
    .CAL_POLY_EN(CAL_POLY_EN),
    .PHE_CORR(PHE_CORR),
    .ALPHA_ODD(ALPHA_ODD)
);

assign INVKDTC = CAL_INVKDTC_EN ? INVKDTC_CAL : INVKDTC_MANUAL;

always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        SWAP_EN_prev <= 1'b0;
        SWAP_EN_DTC <= 1'b0;
    end else begin
        SWAP_EN_prev <= SWAP_EN_DTC;
        SWAP_EN_DTC <= CHOPPER_EN & random_ini[7];
    end
end
always @(negedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        Chopping_EN <= 1'b0; 
    end else begin
        Chopping_EN <= SWAP_EN_prev; // swap analog chopper
    end
end




assign feedback = random_ini[7];
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        random_ini <= 8'd253;
    end else begin
        if (~CHOPPER_EN) begin
            random_ini <= 8'd253;
        end else begin
            random_ini[0] <= feedback;
            random_ini[1] <= random_ini[0];
            random_ini[2] <= (random_ini[1] ^ feedback);
            random_ini[3] <= (random_ini[2] ^ feedback);
            random_ini[4] <= (random_ini[3] ^ feedback);
            random_ini[5] <= random_ini[4];
            random_ini[6] <= random_ini[5];
            random_ini[7] <= random_ini[6];
        end
    end
end


assign DTC_P_CTRL_10b_final = TDC_TEST_MODE ? 
                            DTC_CTRL_manual : (DTC_TEST_MODE ? DTC_TEST_DCWout : DTC_P_CTRL_10b);
DTC_dig DTC_P_dig_inst (
    .clk                            (CKR),
    .DTC_rstn                       (RST_N),
    .DTC_PLL_code                   (DTC_P_CTRL_10b_final),
    // Outputs
    .DTC_top_cap_ctrl_4bit_binary   (DTC_P_top_4b),
    .DTC_top_cap_ctrl_6bit_unary    (DTC_P_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary    (DTC_P_bot_4u)
);

assign DTC_N_CTRL_10b_final = TDC_TEST_MODE ? 
                            OFFSET_DTC_CTRL_manual : (DTC_N_CTRL_10b + OFFSET_DTC_CTRL_10b);

DTC_dig DTC_N_dig_inst (
    .clk                            (CKR),
    .DTC_rstn                       (RST_N),
    .DTC_PLL_code                   (DTC_N_CTRL_10b_final),
    // Outputs
    .DTC_top_cap_ctrl_4bit_binary   (DTC_N_top_4b),
    .DTC_top_cap_ctrl_6bit_unary    (DTC_N_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary    (DTC_N_bot_4u)
);

dtc_dft_ctrl dtc_dft_ctrl_inst (
    .out_code      (DTC_TEST_DCWout),
    .p1            (DTC_TEST_P1),
    .p2            (DTC_TEST_P2),
    .d1            (DTC_TEST_DCW1),
    .d2            (DTC_TEST_DCW2),
    .clk           (CKR),
    .edge_sel      (DTC_TEST_EDGE_SEL),
    .rstn          (RST_N),
    .dtc_test_mode (DTC_TEST_MODE)
);

endmodule