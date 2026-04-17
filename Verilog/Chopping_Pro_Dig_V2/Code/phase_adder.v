`timescale 1ps/1fs

module phase_adder #(
    parameter W_PHE      = 12,
    parameter W_ADC      = 8,
    parameter W_DTC_CTRL = 10,
    parameter W_FCWF     = 12,
    parameter W_INVKDTC  = 12
  ) (
    input  CKR,
    input  RST_N,
 
    input  MASH_SEL,
    input  DIFF_EN,
 
    input  Chopping_EN,
    input  [W_ADC-1:0]      TDC_Q,
    input  [W_ADC-1:0]      TDC_OFT,
 
    input  [W_FCWF-1:0]     FCW_F,
    output [W_PHE-1:0]      PHE,
 
    input  [W_INVKDTC-1:0]  INVKDTC,
    input  DTCQ_SD_EN,
 
    input  BB_EN,
    input  TDC_OUT2SPI_FREEZE,
    output [7:0]            TDC_OUT2SPI,
 
    output PHRF_OV,
    output MASH2_CARRY_P,
    output MASH2_CARRY_N,
 
    output [W_DTC_CTRL-1:0] DTC_P_CTRL_10b,
    output [W_DTC_CTRL-1:0] DTC_N_CTRL_10b,
 
    output reg [W_FCWF-1:0] PHR_F,
    input  MMD_PHASE
  );
 
  // ---------------------------------------------------------------------------
  // Declarations
  // ---------------------------------------------------------------------------
  wire signed [W_ADC-1:0]  tdc_norm;
  reg  signed [W_PHE-1:0]  PHE_F;
  reg                       Chopping_EN_Q, Chopping_EN_Q2;
 
  wire [W_FCWF:0]           FCW_faccum;
  reg  [W_FCWF-1:0]         PHR_F_advance;
 
  reg  [W_FCWF-1:0]         acc2;
  wire [W_FCWF:0]           acc2_sum;
  wire                      ov2;
  reg                       ov2_d1;
 
  localparam W_MULT = W_INVKDTC + W_FCWF;
  reg  [7:0]                TDC_OUT2SPI_reg;
 
  // ---------------------------------------------------------------------------
  // TDC decoder (修复时序对齐)
  // ---------------------------------------------------------------------------
  assign tdc_norm = TDC_Q - TDC_OFT;
 
  always @(posedge CKR, negedge RST_N) begin
    if (!RST_N) begin
      Chopping_EN_Q  <= 1'b0;
      Chopping_EN_Q2 <= 1'b0;
    end else begin
      Chopping_EN_Q  <= Chopping_EN;
      Chopping_EN_Q2 <= Chopping_EN_Q;
    end
  end
 
  always @(negedge CKR, negedge RST_N) begin
    if (!RST_N)
      PHE_F <= 8'd0;
    else
      // 【关键修复】使用 Chopping_EN_Q（延迟1个posedge），与模拟DTC接收的数据周期完美对齐！
      PHE_F <= Chopping_EN_Q2 ? -$signed({tdc_norm[7:0]}) : $signed({tdc_norm[7:0]});
  end
 
  // ---------------------------------------------------------------------------
  // acc1 & acc2
  // ---------------------------------------------------------------------------
  assign FCW_faccum = {1'b0, FCW_F} + {1'b0, PHR_F};
 
  always @(posedge CKR, negedge RST_N) begin
    if (!RST_N) begin
      PHR_F         <= {W_FCWF{1'b0}};
      PHR_F_advance <= {W_FCWF{1'b0}};
    end else begin
      PHR_F         <= FCW_faccum[W_FCWF-1:0];
      PHR_F_advance <= FCW_faccum[W_FCWF-1:0] + FCW_F;
    end
  end
 
  assign PHRF_OV = FCW_faccum[W_FCWF];
  assign acc2_sum = {1'b0, acc2} + {1'b0, FCW_faccum[W_FCWF-1:0]};
  assign ov2      = acc2_sum[W_FCWF];
 
  always @(posedge CKR, negedge RST_N) begin
    if (!RST_N) begin
      acc2   <= {W_FCWF{1'b0}};
      ov2_d1 <= 1'b0;
    end else begin
      acc2   <= MASH_SEL ? acc2_sum[W_FCWF-1:0] : {W_FCWF{1'b0}};
      ov2_d1 <= MASH_SEL ? ov2 : 1'b0;
    end
  end
 
  assign MASH2_CARRY_P = MASH_SEL &  ov2 & ~ov2_d1;
  assign MASH2_CARRY_N = MASH_SEL & ~ov2 &  ov2_d1;
 
  // ---------------------------------------------------------------------------
  // DTC multiplier (Unified SE & DIFF Signed Logic)
  //
  // MASH1-1 相位输入修正：
  //   MATLAB: PHRF_cur = acc1[k] - ov2[k]，均值自然为 0，无需额外减 0.5
  //   MASH1:  PHRF_cur = acc1[k] - 0.5，需要减 2048 居中
  //
  //   时序对齐（设当前 posedge 为第 n 拍）：
  //     PHR_F_advance = acc1[n+1]（超前一拍的 acc1）
  //     acc2_sum      = acc2[n-1] + acc1[n]，进位 ov2 = ov2[n]（组合信号）
  //
  //   MASH1-1 需要配对的是 acc1[n+1] 与 ov2[n+1]
  //   ov2[n+1] 的进位来自 acc2[n] + acc1[n+1]
  //   acc2[n] = acc2_sum[11:0]（本拍组合路径），acc1[n+1] = PHR_F_advance
  // ---------------------------------------------------------------------------
 

  // MASH1:   PHR_F_advance - 2048          （acc1 居中）
  // MASH1-1: PHR_F_advance - ov2_next*4096 （acc1 - ov2，均值已为 0）
wire signed [W_FCWF+1:0] acc1_centered =
    $signed({2'b00, PHR_F_advance})
    - (MASH_SEL ? (ov2 ? 14'sd4096 : 14'sd0) : 14'sd2048);
 
  wire signed [W_FCWF+1:0] invkdtc_signed = $signed({2'b00, INVKDTC});
 
  wire signed [27:0] prod_se_dynamic = acc1_centered * invkdtc_signed;
  wire signed [11:0] dyn_se_scaled   = prod_se_dynamic >>> 14;
 
  wire signed [W_FCWF+1:0] phi_half    = acc1_centered >>> 1;
  wire signed [27:0] prod_diff_dynamic = phi_half * invkdtc_signed;
  wire signed [11:0] dyn_diff_scaled = prod_diff_dynamic >>> 14;
 
  wire signed [11:0] target_dyn_P = DIFF_EN ? 
                        (Chopping_EN ?  dyn_diff_scaled : -dyn_diff_scaled) : 
                        (Chopping_EN ?  dyn_se_scaled   : -dyn_se_scaled);
 
  wire signed [11:0] target_dyn_N = DIFF_EN ? 
                        (Chopping_EN ? -dyn_diff_scaled :  dyn_diff_scaled) : 
                        12'd0;
 
  reg signed [11:0] dyn_P_reg;
  reg signed [11:0] dyn_N_reg;
 
  always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
      dyn_P_reg <= 12'd0;
      dyn_N_reg <= 12'd0;
    end else begin
      dyn_P_reg <= target_dyn_P;
      dyn_N_reg <= target_dyn_N;
    end
  end
 
  localparam signed [12:0] DTC_CENTER = 13'd512; 
  wire signed [12:0] dtc_p_sum = DTC_CENTER + dyn_P_reg;
  wire signed [12:0] dtc_n_sum = DTC_CENTER + dyn_N_reg;
 
  assign DTC_P_CTRL_10b = (dtc_p_sum[12] ? 10'd0 : (dtc_p_sum > 1023 ? 10'd1023 : dtc_p_sum[9:0]));
  assign DTC_N_CTRL_10b = (dtc_n_sum[12] ? 10'd0 : (dtc_n_sum > 1023 ? 10'd1023 : dtc_n_sum[9:0]));
 
  // ---------------------------------------------------------------------------
  // SPI readback
  // ---------------------------------------------------------------------------
  always @(posedge CKR, negedge RST_N) begin
    if (~RST_N)
      TDC_OUT2SPI_reg <= 8'd0;
    else if (~TDC_OUT2SPI_FREEZE)
      TDC_OUT2SPI_reg <= TDC_Q;
  end
  assign TDC_OUT2SPI = TDC_OUT2SPI_reg;
  assign PHE = {{3{PHE_F[7]}}, PHE_F, BB_EN};
 
endmodule