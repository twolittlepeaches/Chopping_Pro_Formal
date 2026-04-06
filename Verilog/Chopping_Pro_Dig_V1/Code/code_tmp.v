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
    DTC_top_4b,
    DTC_top_6u,
    DTC_bot_4u,
    OFFSET_DTC_top_4b,
    OFFSET_DTC_top_6u,
    OFFSET_DTC_bot_4u,
    MAIN_DTC_TEST_EN,
    SWAP_EN,
    MASH_SEL,                // MASH order: 0=MASH1, 1=MASH1-1
    DIFF_EN,                 // Differential enable
    CHOPPER_EN,
    CAL_INVKDTC_EN,
    CAL_TDV2_EN,
    CAL_OFTDTC_EN,
    MMD_PHASE_Q,
    TDV2_EN,
    MMD_RN,
    DTC_TEST_P1,
    DTC_TEST_P2,
    DTC_TEST_DCW1,
    DTC_TEST_DCW2,
    DTC_TEST_EDGE_SEL,
    DTC_TEST_MODE,
    INVKDTC_OUT2SPI_FREEZE,
    Tvd2_mis_OUT2SPI_FREEZE,
    OFTDTC_OUT2SPI_FREEZE,
    INVKDTC_OUT2SPI,
    Tvd2_mis_OUT2SPI,
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
        output  [3:0] DTC_top_4b;              // top caps 4-bit binary
        output [62:0] DTC_top_6u;              // top caps 6-bit unary
        output [14:0] DTC_bot_4u;              // bottom caps 4-bit unary

        // DTC outputs (offset)
        output  [3:0] OFFSET_DTC_top_4b;
        output [62:0] OFFSET_DTC_top_6u;
        output [14:0] OFFSET_DTC_bot_4u;

        // Calibration control
        input         CAL_INVKDTC_EN;          // invKDTC calibration enable
        input         CAL_TDV2_EN;             // TDV2 mismatch calibration enable
        input         CAL_OFTDTC_EN;           // Offset DTC calibration enable

        // Advanced features
        input         CHOPPER_EN;              // chopper (SWAP) enable
        output        SWAP_EN;                 // SWAP enable output
        input         MASH_SEL;                // MASH order: 0=MASH1, 1=MASH1-1
        input         DIFF_EN;                 // differential DTC: 0=SE, 1=DIFF
        input         TDV2_EN;                 // TDV2 (dual phase) enable
        output        MMD_PHASE_Q;             // MMD phase selection (registered)
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
        input Tvd2_mis_OUT2SPI_FREEZE;
        input OFTDTC_OUT2SPI_FREEZE;
        output  [11:0] INVKDTC_OUT2SPI;
        output  [11:0] Tvd2_mis_OUT2SPI;
        output  [11:0] OFTDTC_OUT2SPI;

        input TDC_TEST_MODE;
        input [9:0] OFFSET_DTC_CTRL_manual;
        input [9:0] DTC_CTRL_manual;
        // Reserved
        input  [7:0]  RSV0;                    // reserved input 0
        input  [7:0]  RSV1;                    // reserved input 1

  wire signed [11:0] PHE; 
  wire [11:0] INVKDTC_CAL;
  wire [11:0] INVKDTC;
  wire [9:0]  DTC_P_CTRL_10b;
  wire [9:0]  DTC_N_CTRL_10b;        // N-path code word from phase_adder (DIFF mode)
  wire [9:0] OFFSET_DTC_CTRL_10b;
  wire [11:0] PHR_F;
  wire PHRF_OV;
  wire MMD_UN; // underflow due to TDV2 and SWAP.
  // MASH2 carry corrections: diff(ov2) = MASH2_CARRY_P - MASH2_CARRY_N
  wire MASH2_CARRY_P;
  wire MASH2_CARRY_N;


  reg [7:0] random_ini;
  wire feedback;
  reg SWAP_EN_prev;
  reg SWAP_EN_DTC;
  reg SWAP_EN;
  reg    [7:0] random_tdv2;   
  wire   feedback_tdv2;  
  reg MMD_PHASE_Q;   
  reg MMD_PHASE;    
  wire [9:0] DTC_TEST_DCWout;
  reg MMD_UN_Q;
  wire [9:0] DTC_CTRL_10b_final;
  wire [9:0] OFFSET_DTC_CTRL_10b_final ;

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
    .SWAP_EN(SWAP_EN_DTC),
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
    .DTC_P_CTRL_10b(DTC_P_CTRL_10b),
    .DTC_N_CTRL_10b(DTC_N_CTRL_10b),
    .PHR_F(PHR_F),
    .MMD_PHASE(MMD_PHASE),
    .MMD_UN(MMD_UN),
    .MASH2_CARRY_P(MASH2_CARRY_P),
    .MASH2_CARRY_N(MASH2_CARRY_N)
);


always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        MMD_UN_Q <= 1'b0; 
    end else begin
        MMD_UN_Q <= MMD_UN; 
    end
end
// MMD division ratio:
// MASH1: OV = PHRF_OV (ov1 only)
// MASH2: OV = PHRF_OV + MASH2_CARRY_P - MASH2_CARRY_N  (ov1 + diff(ov2))
assign MMD_RN = FCW_I + PHRF_OV
    + (MASH2_CARRY_P ? 5'd1     : 5'd0)
    + (MASH2_CARRY_N ? 5'b11111 : 5'd0)
    + (MMD_UN        ? 5'b11111 : 5'd0)
    + (MMD_UN_Q      ? 5'd1     : 5'd0);

Tracking Tracking_inst(
    .RST_N(RST_N),
    .CKR(CKR),
    .PHE(PHE),
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
    .W_INVKDTC(12)
) LMS_Calib_inst (
    .CKR(CKR),
    .RST_N(RST_N),
    .PHR_F(PHR_F),
    .PHE(PHE),
    .SWAP_EN(SWAP_EN),// enable chopping
    .DIVD_EN(MMD_PHASE), // enable dual phase selection in MMD
    .CAL_INVKDTC_EN(CAL_INVKDTC_EN), // invKDTC calibration loop control
    .CAL_TDV2_EN(CAL_TDV2_EN), // TDV2 mismatch calibration loop control
    .CAL_OFTDTC_EN(CAL_OFTDTC_EN), // Offset DTC delay calibration loop control
    .INVKDTC_OUT2SPI_FREEZE(INVKDTC_OUT2SPI_FREEZE),
    .Tvd2_mis_OUT2SPI_FREEZE(Tvd2_mis_OUT2SPI_FREEZE),
    .OFTDTC_OUT2SPI_FREEZE(OFTDTC_OUT2SPI_FREEZE),
    .INVKDTC_OUT2SPI(INVKDTC_OUT2SPI),
    .Tvd2_mis_OUT2SPI(Tvd2_mis_OUT2SPI),
    .OFTDTC_OUT2SPI(OFTDTC_OUT2SPI),
    .INVKDTC(INVKDTC_CAL),
    .OFFSET_DTC_CTRL_10b(OFFSET_DTC_CTRL_10b)
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
        SWAP_EN <= 1'b0; 
    end else begin
        SWAP_EN <= SWAP_EN_prev; // swap analog chopper
    end
end

always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        MMD_PHASE <= 1'b0;
        MMD_PHASE_Q <= 1'b0;
    end else begin
        MMD_PHASE <= #400 TDV2_EN & random_tdv2[7] ; //note there is a risk of creating glitches togeter with mmd.
        MMD_PHASE_Q <= #400 MMD_PHASE;
    end
end
assign feedback_tdv2 = random_tdv2[7] ^ random_tdv2[5] ^ random_tdv2[4] ^ random_tdv2[0]; // x^8 + x^6 + x^5 + x + 1

always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        random_tdv2 <= 8'd177;  
    end else begin
        if (~TDV2_EN) begin
            random_tdv2 <= 8'd177;
        end else begin
            random_tdv2 <= {random_tdv2[6:0], feedback_tdv2};
        end
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


// Offset DTC control mux:
//   DIFF_EN=1 → N-path code word from phase_adder (differential mode)
//   DIFF_EN=0 → original LMS calibration word (single-ended mode)
wire [9:0] OFFSET_DTC_CTRL_10b_src;
assign OFFSET_DTC_CTRL_10b_src = DIFF_EN ? DTC_N_CTRL_10b : OFFSET_DTC_CTRL_10b;

assign OFFSET_DTC_CTRL_10b_final = TDC_TEST_MODE ? OFFSET_DTC_CTRL_manual : OFFSET_DTC_CTRL_10b_src;

DTC_dig OFFSET_DTC_dig_inst (
    .clk                            (CKR),
    .DTC_rstn                       (RST_N),
    .DTC_PLL_code                   (OFFSET_DTC_CTRL_10b_final),
    // Outputs
    .DTC_top_cap_ctrl_4bit_binary   (OFFSET_DTC_top_4b),
    .DTC_top_cap_ctrl_6bit_unary    (OFFSET_DTC_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary    (OFFSET_DTC_bot_4u)
);

assign DTC_CTRL_10b_final = TDC_TEST_MODE ? DTC_CTRL_manual : (DTC_TEST_MODE ? DTC_TEST_DCWout : DTC_P_CTRL_10b);
DTC_dig DTC_dig_inst (
    .clk                            (CKR),
    .DTC_rstn                       (RST_N),
    .DTC_PLL_code                   (DTC_CTRL_10b_final),
    // Outputs
    .DTC_top_cap_ctrl_4bit_binary   (DTC_top_4b),
    .DTC_top_cap_ctrl_6bit_unary    (DTC_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary    (DTC_bot_4u)
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