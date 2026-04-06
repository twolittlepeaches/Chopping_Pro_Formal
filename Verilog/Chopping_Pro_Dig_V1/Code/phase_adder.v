/*
`timescale 1ps/1fs

// =============================================================================
// phase_adder
//
// MASH_SEL : 0=MASH1  1=MASH1-1
// DIFF_EN  : 0=single-ended  1=differential
//
// DIFF DCW ranges (INVKDTC~1881, top-10-bit scale ~470):
//   SE  : DCW   ∈ [0,   470],  center=235
//   DIFF: DCW_P ∈ [117, 352],  center=235,  slope negative
//         DCW_N ∈ [117, 352],  center=235,  slope positive
//   X-shape: both cross at center when phi_signed=0 (acc1=0.5)
//
// Formula:
//   diff_p_in = oneminus/2 + 0.25  =  (0.75 - acc1/2)   →  P: decreases with acc1
//   diff_n_in = acc1/2     + 0.25  =  (acc1/2  + 0.25)  →  N: increases with acc1
//   All inputs unsigned, no sign arithmetic.
// =============================================================================

module phase_adder #(
    parameter W_PHE      = 12,
    parameter W_ADC      = 8,
    parameter W_DTC_CTRL = 10,
    parameter W_FCWF     = 12,
    parameter W_INVKDTC  = 12
  ) (
    input  CKR,
    input  RST_N,

    // Feature switches (SPI)
    input  MASH_SEL,
    input  DIFF_EN,

    // TDC
    input  Chopping_EN,
    input  [W_ADC-1:0]      TDC_Q,
    input  [W_ADC-1:0]      TDC_OFT,

    // FCW
    input  [W_FCWF-1:0]     FCW_F,

    // Phase error
    output [W_PHE-1:0]      PHE,

    // DTC gain (from LMS_Calib)
    input  [W_INVKDTC-1:0]  INVKDTC,
    input  DTCQ_SD_EN,

    // SPI readback / debug
    input  BB_EN,
    input  TDC_OUT2SPI_FREEZE,
    output [7:0]             TDC_OUT2SPI,

    // Overflow -> MMD_RN
    output PHRF_OV,
    output MASH2_CARRY_P,
    output MASH2_CARRY_N,

    // DTC control outputs
    output [W_DTC_CTRL-1:0] DTC_P_CTRL_10b,   // main   (P-path)
    output [W_DTC_CTRL-1:0] DTC_N_CTRL_10b,   // offset (N-path, DIFF only)

    // LMS side-channel
    output reg [W_FCWF-1:0] PHR_F,

    // TDV2 port kept for top-level compatibility (unused internally)
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
  wire                       ov2;
  reg                        ov2_d1;

  localparam W_MULT = W_INVKDTC + W_FCWF;   // 24 bits
  reg  [W_MULT-1:0]         mult_P;
  reg  [W_MULT-1:0]         mult_N;

  wire [W_FCWF:0]           oneminus_PHRF;   // 13-bit: 1 - PHR_F_advance

  reg  [7:0]                TDC_OUT2SPI_reg;

  // ---------------------------------------------------------------------------
  // TDC decoder
  // Latched on negedge so TDC result (ready after posedge) is cleanly captured.
  // Chopping_EN_Q2: aligned to the DTC swap that produced this TDC result.
  // ---------------------------------------------------------------------------
  assign tdc_norm = TDC_Q - TDC_OFT;

  always @(negedge CKR, negedge RST_N)
  begin
    if (!RST_N)
      PHE_F <= 8'd0;
    else
      PHE_F <= Chopping_EN_Q2 ? -$signed({tdc_norm[7:0]})
                              :  $signed({tdc_norm[7:0]});
  end

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      Chopping_EN_Q  <= 1'b0;
      Chopping_EN_Q2 <= 1'b0;
    end
    else
    begin
      Chopping_EN_Q  <= Chopping_EN;
      Chopping_EN_Q2 <= Chopping_EN_Q;
    end
  end

  // ---------------------------------------------------------------------------
  // acc1 — FCW fractional accumulator
  //   PHR_F         : current acc1, sent to LMS_Calib as correlation signal
  //   PHR_F_advance : acc1 one step ahead, used for DTC multiply (pipeline)
  //   PHRF_OV       : ov1 carry → MMD counts one extra VCO cycle
  // ---------------------------------------------------------------------------
  assign FCW_faccum = {1'b0, FCW_F} + {1'b0, PHR_F};

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      PHR_F         <= {W_FCWF{1'b0}};
      PHR_F_advance <= {W_FCWF{1'b0}};
    end
    else
    begin
      PHR_F         <= FCW_faccum[W_FCWF-1:0];
      PHR_F_advance <= FCW_faccum[W_FCWF-1:0] + FCW_F;
    end
  end

  assign PHRF_OV = FCW_faccum[W_FCWF];

  // ---------------------------------------------------------------------------
  // acc2 — MASH1-1 second accumulator
  //   MASH_SEL=0 : acc2 frozen, CARRY outputs always 0  →  MASH1
  //   MASH_SEL=1 : acc2 += acc1_new each cycle           →  MASH1-1
  //   diff(ov2) = MASH2_CARRY_P (+1) or MASH2_CARRY_N (-1)
  //   Full OV for MMD_RN = PHRF_OV + MASH2_CARRY_P - MASH2_CARRY_N
  // ---------------------------------------------------------------------------
  assign acc2_sum = {1'b0, acc2} + {1'b0, FCW_faccum[W_FCWF-1:0]};
  assign ov2      = acc2_sum[W_FCWF];

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      acc2   <= {W_FCWF{1'b0}};
      ov2_d1 <= 1'b0;
    end
    else
    begin
      acc2   <= MASH_SEL ? acc2_sum[W_FCWF-1:0] : {W_FCWF{1'b0}};
      ov2_d1 <= MASH_SEL ? ov2 : 1'b0;
    end
  end

  assign MASH2_CARRY_P = MASH_SEL &  ov2 & ~ov2_d1;
  assign MASH2_CARRY_N = MASH_SEL & ~ov2 &  ov2_d1;

  // ---------------------------------------------------------------------------
  // DTC multiplier
  //
  // ---- Single-ended (DIFF_EN=0) ----
  //   Chopping=0 :  DCW = (1-acc1) * INVKDTC   range [0, ~470]
  //   Chopping=1 :  DCW =  acc1    * INVKDTC   range [0, ~470]  (paths swapped)
  //
  // ---- Differential (DIFF_EN=1) ----
  //   Target (from MATLAB):
  //     DCW_P = -(phi_signed/2)*gain,  DCW_N = +(phi_signed/2)*gain
  //     where phi_signed = acc1 - 0.5 ∈ [-0.5, +0.5)
  //
  //   Fully unsigned implementation (no sign arithmetic):
  //     diff_p_in = oneminus/2 + 0.25 = (0.75 - acc1/2)
  //     diff_n_in = acc1/2     + 0.25 = (acc1/2 + 0.25)
  //
  //   Derivation:
  //     DCW_P = center - phi_signed/2 * INVKDTC
  //           = 0.5*INVKDTC - (acc1-0.5)/2 * INVKDTC
  //           = (0.75 - acc1/2) * INVKDTC
  //           = (oneminus/2 + 0.25) * INVKDTC   ← diff_p_in * INVKDTC
  //
  //     DCW_N = center + phi_signed/2 * INVKDTC
  //           = (acc1/2 + 0.25) * INVKDTC        ← diff_n_in * INVKDTC
  //
  //   Properties:
  //     diff_p_in + diff_n_in = 1.0 (constant)     → DCW_P + DCW_N = INVKDTC (const) ✓
  //     diff_n_in - diff_p_in = acc1 - 0.5 = phi_signed → DCW_N - DCW_P = phi_signed * INVKDTC ✓
  //     center (acc1=0.5): both = 0.5 * INVKDTC ≈ 235 (same as SE center) ✓
  //     range per path: [0.25, 0.75] * INVKDTC = [117, 352] ← half of SE range ✓
  //
  //   Chopping (SWAP): exchange P and N code words.
  // ---------------------------------------------------------------------------

  // 1. 将 acc1 (0~4095) 居中对齐到 0: -> 范围 [-2048, +2047]
  wire signed [W_FCWF:0] acc1_centered = $signed({1'b0, PHR_F_advance}) - $signed(13'h800);
  
  // 2. 将 INVKDTC 转为有符号数参与乘法 (扩宽防止符号位污染)
  wire signed [W_FCWF+1:0] invkdtc_signed = $signed({2'b00, INVKDTC});
  
  // 3. --- 单端模式 (SE) 的全幅摆动计算 ---
  // 动态摆动量: 13-bit * 14-bit = 27-bit
  wire signed [26:0] prod_se_dynamic = acc1_centered * invkdtc_signed;
  // 缩放回 10-bit 级别 (摆幅约在 ±235 左右)
  wire signed [11:0] dyn_se_scaled   = prod_se_dynamic >>> 14;

  // 4. --- 差分模式 (DIFF) 的半幅摆动计算 ---
  // 差分各承担一半: 除以 2 -> 范围 [-1024, +1023]
  wire signed [W_FCWF:0] phi_half    = acc1_centered >>> 1;
  wire signed [26:0] prod_diff_dynamic = phi_half * invkdtc_signed;
  // 缩放回 10-bit 级别 (摆幅约在 ±117 左右)
  wire signed [11:0] dyn_diff_scaled = prod_diff_dynamic >>> 14;

  // 5. --- 组合与 Chopping 极性分配 ---
  wire signed [11:0] target_dyn_P;
  wire signed [11:0] target_dyn_N;

  // P 路分配:
  // DIFF 模式下: 无 Chop 时随 acc1 减小 (负斜率), Chop 时正斜率
  // SE 模式下  : 同样保证无 Chop 时负斜率, Chop 时正斜率
  assign target_dyn_P = DIFF_EN ? 
                        (Chopping_EN ?  dyn_diff_scaled : -dyn_diff_scaled) : 
                        (Chopping_EN ?  dyn_se_scaled   : -dyn_se_scaled);

  // N 路分配:
  // DIFF 模式下: 与 P 路完全相反
  // SE 模式下  : 不承担动态调制，完全静止于 0 (后续会加上 DTC_CENTER)
  assign target_dyn_N = DIFF_EN ? 
                        (Chopping_EN ? -dyn_diff_scaled :  dyn_diff_scaled) : 
                        12'd0;

  // 6. --- 寄存器打拍 ---
  reg signed [11:0] dyn_P_reg;
  reg signed [11:0] dyn_N_reg;

  always @(posedge CKR, negedge RST_N)
  begin
    if (~RST_N) begin
      dyn_P_reg <= 12'd0;
      dyn_N_reg <= 12'd0;
    end else begin
      dyn_P_reg <= target_dyn_P;
      dyn_N_reg <= target_dyn_N;
    end
  end

  // ---------------------------------------------------------------------------
  // DTC 10-bit outputs (强制添加物理共模)
  // ---------------------------------------------------------------------------
  // 在此统一锁定 DTC 的绝对物理中心！两路在所有模式下都以此为基准。
  localparam signed [12:0] DTC_CENTER = 13'd512; 

  wire signed [12:0] dtc_p_sum = DTC_CENTER + dyn_P_reg;
  wire signed [12:0] dtc_n_sum = DTC_CENTER + dyn_N_reg;

  // 饱和截断逻辑（带防下溢和防上溢保护）
  assign DTC_P_CTRL_10b = (dtc_p_sum[12] ? 10'd0 : (dtc_p_sum > 1023 ? 10'd1023 : dtc_p_sum[9:0]));
  assign DTC_N_CTRL_10b = (dtc_n_sum[12] ? 10'd0 : (dtc_n_sum > 1023 ? 10'd1023 : dtc_n_sum[9:0]));

  // ---------------------------------------------------------------------------
  // SPI readback
  // ---------------------------------------------------------------------------
  // ---------------------------------------------------------------------------
  // SPI readback
  // ---------------------------------------------------------------------------
  always @(posedge CKR, negedge RST_N)
  begin
    if (~RST_N)
      TDC_OUT2SPI_reg <= 8'd0;
    else if (~TDC_OUT2SPI_FREEZE)
      TDC_OUT2SPI_reg <= TDC_Q;
  end
  assign TDC_OUT2SPI = TDC_OUT2SPI_reg;

  // ---------------------------------------------------------------------------
  // Phase error output
  // ---------------------------------------------------------------------------
  assign PHE = {{3{PHE_F[7]}}, PHE_F, BB_EN};

endmodule
  */
  /*
module phase_adder #(
    parameter W_PHE      = 12,
    parameter W_ADC      = 8,
    parameter W_DTC_CTRL = 10,
    parameter W_FCWF     = 12,
    parameter W_INVKDTC  = 12
  ) (
    input  CKR,
    input  RST_N,

    // Feature switches (SPI)
    input  MASH_SEL,
    input  DIFF_EN,

    // TDC
    input  Chopping_EN,
    input  [W_ADC-1:0]      TDC_Q,
    input  [W_ADC-1:0]      TDC_OFT,

    // FCW
    input  [W_FCWF-1:0]     FCW_F,

    // Phase error
    output [W_PHE-1:0]      PHE,

    // DTC gain
    input  [W_INVKDTC-1:0]  INVKDTC,
    input  DTCQ_SD_EN,

    // SPI readback / debug
    input  BB_EN,
    input  TDC_OUT2SPI_FREEZE,
    output [7:0]             TDC_OUT2SPI,

    // Overflow -> MMD_RN
    output PHRF_OV,
    output MASH2_CARRY_P,
    output MASH2_CARRY_N,

    // DTC control outputs
    output [W_DTC_CTRL-1:0] DTC_P_CTRL_10b,   // main  (P-path)
    output [W_DTC_CTRL-1:0] DTC_N_CTRL_10b,   // offset(N-path, DIFF only)

    // LMS side-channel
    output reg [W_FCWF-1:0] PHR_F,

    // TDV2 (kept as input only, unused internally after TDV2 removal)
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
  wire                       ov2;
  reg                        ov2_d1;

  localparam W_MULT = W_INVKDTC + W_FCWF;
  reg  [W_MULT-1:0]         mult_P;
  reg  [W_MULT-1:0]         mult_N;

  wire [W_FCWF:0]           oneminus_PHRF;

  reg  [7:0]                TDC_OUT2SPI_reg;

  // ---------------------------------------------------------------------------
  // TDC decoder
  // ---------------------------------------------------------------------------
  assign tdc_norm = TDC_Q - TDC_OFT;

  always @(negedge CKR, negedge RST_N)
  begin
    if (!RST_N)
      PHE_F <= 8'd0;
    else
      PHE_F <= Chopping_EN_Q2 ? -$signed({tdc_norm[7:0]})
            :  $signed({tdc_norm[7:0]});
  end

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      Chopping_EN_Q  <= 1'b0;
      Chopping_EN_Q2 <= 1'b0;
    end
    else
    begin
      Chopping_EN_Q  <= Chopping_EN;
      Chopping_EN_Q2 <= Chopping_EN_Q;
    end
  end

  // ---------------------------------------------------------------------------
  // acc1 — FCW fractional accumulator
  // ---------------------------------------------------------------------------
  assign FCW_faccum = {1'b0, FCW_F} + {1'b0, PHR_F};
// 13位 = 12位FCW_F + 12位PHR_F，进位就是OV(ov1)
  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      PHR_F         <= {W_FCWF{1'b0}};
      PHR_F_advance <= {W_FCWF{1'b0}};
    end
    else
    begin
      PHR_F         <= FCW_faccum[W_FCWF-1:0];// 当前acc1，送给LMS做相关
      PHR_F_advance <= FCW_faccum[W_FCWF-1:0] + FCW_F;// 提前一拍的acc1
    end
  end

  assign PHRF_OV = FCW_faccum[W_FCWF];// ov1，进位 → MMD多数一个周期

  // ---------------------------------------------------------------------------
  // acc2 — MASH2 (frozen at 0 when MASH_SEL=0)
  // ---------------------------------------------------------------------------
  assign acc2_sum = {1'b0, acc2} + {1'b0, FCW_faccum[W_FCWF-1:0]};
  assign ov2      = acc2_sum[W_FCWF];

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      acc2   <= {W_FCWF{1'b0}};
      ov2_d1 <= 1'b0;
    end
    else
    begin
      acc2   <= MASH_SEL ? acc2_sum[W_FCWF-1:0] : {W_FCWF{1'b0}};
      ov2_d1 <= MASH_SEL ? ov2 : 1'b0;
    end
  end

  assign MASH2_CARRY_P = MASH_SEL &  ov2 & ~ov2_d1;
  assign MASH2_CARRY_N = MASH_SEL & ~ov2 &  ov2_d1;

  // ---------------------------------------------------------------------------
  // DTC multiplier
  //
  // phi_signed = acc1 - 0.5
  // In W_FCWF unsigned fixed-point: flip MSB of acc1 = subtract 0.5 mod 1
  //   flip_msb(x) = {~x[MSB], x[MSB-1:0]}
  //
  // MATLAB DIFF_DTC_EN=1 (no swap):
  //   DCW_P = -(phi_signed/2) * gain  -->  prod_diff_p = flip_msb(acc1) * INVKDTC
  //   DCW_N = +(phi_signed/2) * gain  -->  prod_diff_n = (-flip_msb(acc1)) * INVKDTC
  //
  // -flip_msb(acc1) in unsigned fixed-point = 2's complement of flip_msb:
  //   neg_phi = ~{~acc1[MSB], acc1[MSB-1:0]} + 1
  //           = { acc1[MSB], ~acc1[MSB-1:0]} + 1
  //
  // This gives DCW_P = -DCW_N exactly, matching MATLAB lines 282-283.
  //
  // Single-ended (DIFF_EN=0):
  //   Chopping=0: DCW = (1-acc1)*INVKDTC  = oneminus * INVKDTC
  //   Chopping=1: DCW =  acc1   *INVKDTC  = PHR_F_advance * INVKDTC
  //
  // DIFF + Chopping: exchange P and N — matches MATLAB lines 284-286.
  // ---------------------------------------------------------------------------
  assign oneminus_PHRF = {1'b0, {W_FCWF{1'b1}}} - {1'b0, PHR_F_advance};
  //oneminus_PHRF = 1 - PHR_F_advance    13位，符号位为0


  // phi_signed/2 encoded as unsigned W_FCWF word for multiplier input
  wire [W_FCWF-1:0] phi_pos = 
      {~PHR_F_advance[W_FCWF-1],  PHR_F_advance[W_FCWF-2:0]};// 12位正数，符号位为0
  wire [W_FCWF-1:0] phi_neg = 
      { PHR_F_advance[W_FCWF-1], ~PHR_F_advance[W_FCWF-2:0]};// 12位负数，符号位为1，补码

  wire [W_MULT-1:0] prod_se_swap0 = oneminus_PHRF[W_FCWF-1:0] * INVKDTC;
  wire [W_MULT-1:0] prod_se_swap1 = PHR_F_advance              * INVKDTC;
  wire [W_MULT-1:0] prod_diff_p   = phi_pos * INVKDTC;
  wire [W_MULT-1:0] prod_diff_n   = phi_neg * INVKDTC;

  always @(posedge CKR, negedge RST_N)
  begin
    if (~RST_N)
    begin
      mult_P <= {W_MULT{1'b0}};//24位，12位INVKDTC，12位PHR_F_advance
      mult_N <= {W_MULT{1'b0}};//24位，12位INVKDTC，12位PHR_F_advance
    end
    else
    begin
      if (DIFF_EN)
      begin
        // No swap: P=phi_pos, N=phi_neg  (DCW_P = -DCW_N)
        // Swap:    P=phi_neg, N=phi_pos  (exchange paths for Chopping)
        mult_P <= Chopping_EN ? prod_diff_n : prod_diff_p;
        mult_N <= Chopping_EN ? prod_diff_p : prod_diff_n;
      end
      else
      begin
        mult_P <= Chopping_EN ? prod_se_swap1 : prod_se_swap0;
        mult_N <= {W_MULT{1'b0}};
      end
    end
  end

  // ---------------------------------------------------------------------------
  // DTC 10-bit outputs
  // ---------------------------------------------------------------------------
  assign DTC_P_CTRL_10b = mult_P[W_MULT-1 : W_MULT-W_DTC_CTRL];
  assign DTC_N_CTRL_10b = mult_N[W_MULT-1 : W_MULT-W_DTC_CTRL];

  // ---------------------------------------------------------------------------
  // SPI readback
  // ---------------------------------------------------------------------------
  always @(posedge CKR, negedge RST_N)
  begin
    if (~RST_N)
      TDC_OUT2SPI_reg <= 8'd0;
    else if (~TDC_OUT2SPI_FREEZE)
      TDC_OUT2SPI_reg <= TDC_Q;
  end
  assign TDC_OUT2SPI = TDC_OUT2SPI_reg;

  // ---------------------------------------------------------------------------
  // Phase error
  // ---------------------------------------------------------------------------
  assign PHE = {{3{PHE_F[7]}}, PHE_F, BB_EN};

endmodule*/


/*
`timescale 1ps/1fs

// =============================================================================
// phase_adder
//
// MASH_SEL : 0=MASH1  1=MASH1-1
// DIFF_EN  : 0=single-ended  1=differential
//
// DIFF DCW ranges (INVKDTC~1881, top-10-bit scale ~470):
//   SE  : DCW   ∈ [0,   470],  center=235
//   DIFF: DCW_P ∈ [117, 352],  center=235,  slope negative
//         DCW_N ∈ [117, 352],  center=235,  slope positive
//   X-shape: both cross at center when phi_signed=0 (acc1=0.5)
//
// Formula:
//   diff_p_in = oneminus/2 + 0.25  =  (0.75 - acc1/2)   →  P: decreases with acc1
//   diff_n_in = acc1/2     + 0.25  =  (acc1/2  + 0.25)  →  N: increases with acc1
//   All inputs unsigned, no sign arithmetic.
// =============================================================================

module phase_adder #(
    parameter W_PHE      = 12,
    parameter W_ADC      = 8,
    parameter W_DTC_CTRL = 10,
    parameter W_FCWF     = 12,
    parameter W_INVKDTC  = 12
  ) (
    input  CKR,
    input  RST_N,

    // Feature switches (SPI)
    input  MASH_SEL,
    input  DIFF_EN,

    // TDC
    input  Chopping_EN,
    input  [W_ADC-1:0]      TDC_Q,
    input  [W_ADC-1:0]      TDC_OFT,

    // FCW
    input  [W_FCWF-1:0]     FCW_F,

    // Phase error
    output [W_PHE-1:0]      PHE,

    // DTC gain (from LMS_Calib)
    input  [W_INVKDTC-1:0]  INVKDTC,
    input  DTCQ_SD_EN,

    // SPI readback / debug
    input  BB_EN,
    input  TDC_OUT2SPI_FREEZE,
    output [7:0]             TDC_OUT2SPI,

    // Overflow -> MMD_RN
    output PHRF_OV,
    output MASH2_CARRY_P,
    output MASH2_CARRY_N,

    // DTC control outputs
    output [W_DTC_CTRL-1:0] DTC_P_CTRL_10b,   // main   (P-path)
    output [W_DTC_CTRL-1:0] DTC_N_CTRL_10b,   // offset (N-path, DIFF only)

    // LMS side-channel
    output reg [W_FCWF-1:0] PHR_F,

    // TDV2 port kept for top-level compatibility (unused internally)
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
  wire                       ov2;
  reg                        ov2_d1;

  localparam W_MULT = W_INVKDTC + W_FCWF;   // 24 bits
  reg  [W_MULT-1:0]         mult_P;
  reg  [W_MULT-1:0]         mult_N;

  wire [W_FCWF:0]           oneminus_PHRF;   // 13-bit: 1 - PHR_F_advance

  reg  [7:0]                TDC_OUT2SPI_reg;

  // ---------------------------------------------------------------------------
  // TDC decoder
  // Latched on negedge so TDC result (ready after posedge) is cleanly captured.
  // Chopping_EN_Q2: aligned to the DTC swap that produced this TDC result.
  // ---------------------------------------------------------------------------
  assign tdc_norm = TDC_Q - TDC_OFT;

  always @(negedge CKR, negedge RST_N)
  begin
    if (!RST_N)
      PHE_F <= 8'd0;
    else
      PHE_F <= Chopping_EN_Q2 ? -$signed({tdc_norm[7:0]})
                              :  $signed({tdc_norm[7:0]});
  end

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      Chopping_EN_Q  <= 1'b0;
      Chopping_EN_Q2 <= 1'b0;
    end
    else
    begin
      Chopping_EN_Q  <= Chopping_EN;
      Chopping_EN_Q2 <= Chopping_EN_Q;
    end
  end

  // ---------------------------------------------------------------------------
  // acc1 — FCW fractional accumulator
  //   PHR_F         : current acc1, sent to LMS_Calib as correlation signal
  //   PHR_F_advance : acc1 one step ahead, used for DTC multiply (pipeline)
  //   PHRF_OV       : ov1 carry → MMD counts one extra VCO cycle
  // ---------------------------------------------------------------------------
  assign FCW_faccum = {1'b0, FCW_F} + {1'b0, PHR_F};

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      PHR_F         <= {W_FCWF{1'b0}};
      PHR_F_advance <= {W_FCWF{1'b0}};
    end
    else
    begin
      PHR_F         <= FCW_faccum[W_FCWF-1:0];
      PHR_F_advance <= FCW_faccum[W_FCWF-1:0] + FCW_F;
    end
  end

  assign PHRF_OV = FCW_faccum[W_FCWF];

  // ---------------------------------------------------------------------------
  // acc2 — MASH1-1 second accumulator
  //   MASH_SEL=0 : acc2 frozen, CARRY outputs always 0  →  MASH1
  //   MASH_SEL=1 : acc2 += acc1_new each cycle           →  MASH1-1
  //   diff(ov2) = MASH2_CARRY_P (+1) or MASH2_CARRY_N (-1)
  //   Full OV for MMD_RN = PHRF_OV + MASH2_CARRY_P - MASH2_CARRY_N
  // ---------------------------------------------------------------------------
  assign acc2_sum = {1'b0, acc2} + {1'b0, FCW_faccum[W_FCWF-1:0]};
  assign ov2      = acc2_sum[W_FCWF];

  always @(posedge CKR, negedge RST_N)
  begin
    if (!RST_N)
    begin
      acc2   <= {W_FCWF{1'b0}};
      ov2_d1 <= 1'b0;
    end
    else
    begin
      acc2   <= MASH_SEL ? acc2_sum[W_FCWF-1:0] : {W_FCWF{1'b0}};
      ov2_d1 <= MASH_SEL ? ov2 : 1'b0;
    end
  end

  assign MASH2_CARRY_P = MASH_SEL &  ov2 & ~ov2_d1;
  assign MASH2_CARRY_N = MASH_SEL & ~ov2 &  ov2_d1;

  // ---------------------------------------------------------------------------
  // DTC multiplier
  //
  // ---- Single-ended (DIFF_EN=0) ----
  //   Chopping=0 :  DCW = (1-acc1) * INVKDTC   range [0, ~470]
  //   Chopping=1 :  DCW =  acc1    * INVKDTC   range [0, ~470]  (paths swapped)
  //
  // ---- Differential (DIFF_EN=1) ----
  //   Target (from MATLAB):
  //     DCW_P = -(phi_signed/2)*gain,  DCW_N = +(phi_signed/2)*gain
  //     where phi_signed = acc1 - 0.5 ∈ [-0.5, +0.5)
  //
  //   Fully unsigned implementation (no sign arithmetic):
  //     diff_p_in = oneminus/2 + 0.25 = (0.75 - acc1/2)
  //     diff_n_in = acc1/2     + 0.25 = (acc1/2 + 0.25)
  //
  //   Derivation:
  //     DCW_P = center - phi_signed/2 * INVKDTC
  //           = 0.5*INVKDTC - (acc1-0.5)/2 * INVKDTC
  //           = (0.75 - acc1/2) * INVKDTC
  //           = (oneminus/2 + 0.25) * INVKDTC   ← diff_p_in * INVKDTC
  //
  //     DCW_N = center + phi_signed/2 * INVKDTC
  //           = (acc1/2 + 0.25) * INVKDTC        ← diff_n_in * INVKDTC
  //
  //   Properties:
  //     diff_p_in + diff_n_in = 1.0 (constant)     → DCW_P + DCW_N = INVKDTC (const) ✓
  //     diff_n_in - diff_p_in = acc1 - 0.5 = phi_signed → DCW_N - DCW_P = phi_signed * INVKDTC ✓
  //     center (acc1=0.5): both = 0.5 * INVKDTC ≈ 235 (same as SE center) ✓
  //     range per path: [0.25, 0.75] * INVKDTC = [117, 352] ← half of SE range ✓
  //
  //   Chopping (SWAP): exchange P and N code words.
  // ---------------------------------------------------------------------------
  assign oneminus_PHRF = {1'b0, {W_FCWF{1'b1}}} - {1'b0, PHR_F_advance};

  // oneminus/2 : right-shift by 1  →  MSB always 0, range [0, 0.5)
  wire [W_FCWF-1:0] oneminus_half = {1'b0, oneminus_PHRF[W_FCWF-1:1]};
  // acc1/2     : right-shift by 1  →  MSB always 0, range [0, 0.5)
  wire [W_FCWF-1:0] acc1_half     = {1'b0, PHR_F_advance[W_FCWF-1:1]};
  // 0.25 in W_FCWF unsigned fixed-point = 2^(W_FCWF-2) = 12'h400
  localparam [W_FCWF-1:0] QUARTER = {2'b01, {(W_FCWF-2){1'b0}}};

  // DIFF multiplier inputs: range [0.25, 0.75] → no overflow in W_FCWF bits
  wire [W_FCWF-1:0] diff_p_in = oneminus_half + QUARTER;
  wire [W_FCWF-1:0] diff_n_in = acc1_half     + QUARTER;

  // Four product candidates (all combinational, registered below)
  wire [W_MULT-1:0] prod_se_swap0 = oneminus_PHRF[W_FCWF-1:0] * INVKDTC;
  wire [W_MULT-1:0] prod_se_swap1 = PHR_F_advance              * INVKDTC;
  wire [W_MULT-1:0] prod_diff_p   = diff_p_in                  * INVKDTC;
  wire [W_MULT-1:0] prod_diff_n   = diff_n_in                  * INVKDTC;

  always @(posedge CKR, negedge RST_N)
  begin
    if (~RST_N)
    begin
      mult_P <= {W_MULT{1'b0}};
      mult_N <= {W_MULT{1'b0}};
    end
    else
    begin
      if (DIFF_EN)
      begin
        // No Chopping : P=diff_p (high when acc1 low), N=diff_n (low when acc1 low) → X shape
        // Chopping    : swap P and N paths
        mult_P <= Chopping_EN ? prod_diff_n : prod_diff_p;
        mult_N <= Chopping_EN ? prod_diff_p : prod_diff_n;
      end
      else
      begin
        // Single-ended: N-path always 0
        mult_P <= Chopping_EN ? prod_se_swap1 : prod_se_swap0;
        mult_N <= {W_MULT{1'b0}};
      end
    end
  end

  // ---------------------------------------------------------------------------
  // DTC 10-bit outputs
  // Take top W_DTC_CTRL bits of 24-bit product.
  // DCW_natural = floor(input_frac * INVKDTC / 4)
  //
  // DCW中心对齐（移位到512）：
  //   自然中心 = INVKDTC/8（DIFF模式，input_frac=0.5时）
  //     初始INVKDTC=1881 → 中心=235，range [117,352]
  //     校准后INVKDTC≈1456 → 中心=182，range [91,273]
  //   目标中心 = 512（INL曲线的对称中心，hardcode in DTC_analog_10b）
  //   offset = 512 - floor(INVKDTC/8)
  //     → 自动追踪INVKDTC变化，始终维持工作中心在code 512
  //
  //   移位后范围（INVKDTC≈1881）：
  //     SE:   [277, 747]，中心=512，u∈[-1.0, +1.0]（INL_HALFRANGE=235）
  //     DIFF: [394, 629]，中心=512，u∈[-0.5, +0.5] ✓（与MATLAB DIFF对应）
  //
  //   溢出分析：max shifted = 3/4×INVKDTC/4 + offset = 512+INVKDTC/8 ≤ 512+511=1023 ✓
  // ---------------------------------------------------------------------------
  wire [W_DTC_CTRL-1:0] dcw_P_raw      = mult_P[W_MULT-1 : W_MULT-W_DTC_CTRL];
  wire [W_DTC_CTRL-1:0] dcw_N_raw      = mult_N[W_MULT-1 : W_MULT-W_DTC_CTRL];

  // INVKDTC/8：取bit[W_INVKDTC-1:3]（9位），零扩展到10位
  wire [W_DTC_CTRL-1:0] dcw_center_nat = {1'b0, INVKDTC[W_INVKDTC-1:3]};
  wire [W_DTC_CTRL-1:0] dcw_offset     = 10'd512 - dcw_center_nat;

  assign DTC_P_CTRL_10b = dcw_P_raw + dcw_offset;
  assign DTC_N_CTRL_10b = DIFF_EN ? (dcw_N_raw + dcw_offset) : 10'b0;

  // ---------------------------------------------------------------------------
  // SPI readback
  // ---------------------------------------------------------------------------
  always @(posedge CKR, negedge RST_N)
  begin
    if (~RST_N)
      TDC_OUT2SPI_reg <= 8'd0;
    else if (~TDC_OUT2SPI_FREEZE)
      TDC_OUT2SPI_reg <= TDC_Q;
  end
  assign TDC_OUT2SPI = TDC_OUT2SPI_reg;

  // ---------------------------------------------------------------------------
  // Phase error output
  // ---------------------------------------------------------------------------
  assign PHE = {{3{PHE_F[7]}}, PHE_F, BB_EN};

endmodule

*/
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
  // ---------------------------------------------------------------------------
  wire signed [W_FCWF:0] acc1_centered = $signed({1'b0, PHR_F_advance}) - $signed(13'h800);
  wire signed [W_FCWF+1:0] invkdtc_signed = $signed({2'b00, INVKDTC});
  
  wire signed [26:0] prod_se_dynamic = acc1_centered * invkdtc_signed;
  wire signed [11:0] dyn_se_scaled   = prod_se_dynamic >>> 14;

  wire signed [W_FCWF:0] phi_half    = acc1_centered >>> 1;
  wire signed [26:0] prod_diff_dynamic = phi_half * invkdtc_signed;
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
