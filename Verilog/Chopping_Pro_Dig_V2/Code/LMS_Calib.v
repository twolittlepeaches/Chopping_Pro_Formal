
`timescale 1ps/1fs
 
// =============================================================================
// LMS_Calib.v  —  V2 终版（含 MASH1-1 支持）
//
// 新增：
//   MASH_SEL 端口：0=MASH1，1=MASH1-1
//   max_dc 根据 MASH_SEL 自动切换：
//     MASH1:   max_dc = INVKDTC >> 3  (对应 u_max = ±1.0)
//     MASH1-1: max_dc = INVKDTC >> 2  (对应 u_max = ±1.0，因摆幅翻倍)
//
// 校准逻辑（对应 Chopping_pro_td_v6.m）：
//   1. INVKDTC  sign-sign LMS: sign(PHE_CORR) × sign(PHR_F)
//   2. OFTDTC   sign-sign LMS: sign(PHE_CORR) × sign(Chopping)
//   3. POLY     sign-sign LMS: sign(PHE_CORR) × sign(P₃(u))
//      u = (DCW_P - DCW_N) / max_dc，归一化到 [-1,+1]
//      Chopping swap 时 u 取反（与 PHE 翻转同步）
//      P₃(u) = 0.5×(5u³ - 3u)
//      PHE_CORR = PHE - alpha_int × P₃(u_prev) >> 8
// =============================================================================
 
module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10,
    parameter [31:0] MU_POLY = 32'd128
) (
    input  CKR,
    input  RST_N,
 
    input  signed [W_PHE-1:0]      PHE,
    input  [W_FCWF-1:0]            PHR_F,
    input                          CAL_INVKDTC_EN,
    input                          INVKDTC_OUT2SPI_FREEZE,
    output reg [11:0]              INVKDTC_OUT2SPI,
    output reg [W_INVKDTC-1:0]    INVKDTC,
 
    input                          Chopping_EN,
    input                          CAL_OFTDTC_EN,
    input                          OFTDTC_OUT2SPI_FREEZE,
    output reg [11:0]              OFTDTC_OUT2SPI,
    output [9:0]                   OFFSET_DTC_CTRL_10b,
 
    input  [W_DTC_CTRL-1:0]        DTC_P_CTRL_10b,
    input  [W_DTC_CTRL-1:0]        DTC_N_CTRL_10b,
    input                          DIFF_EN,
    input                          MASH_SEL,       // 0=MASH1, 1=MASH1-1
    input                          CAL_POLY_EN,
 
    output signed [W_PHE-1:0]      PHE_CORR,
    output signed [15:0]           ALPHA_ODD
);
 
// =========================================================================
// 流水线寄存器（统一打一拍，与 PHE 对齐）
// =========================================================================
reg  [W_FCWF-1:0]     PHR_F_Q;
reg                   Chopping_EN_Q;
reg  [W_DTC_CTRL-1:0] DTC_P_Q;
reg  [W_DTC_CTRL-1:0] DTC_N_Q;
reg                   MASH_SEL_Q;      // 打拍防止毛刺，与码字对齐
 
reg  [11:0]           OFFSET_DTC_calib;
reg  signed [31:0]    alpha_odd_reg;
reg  signed [8:0]     poly_Q8_reg;
 
integer diff_c;
integer max_dc;
integer u_int;
integer u3_q8;
integer poly_int;
 
// =========================================================================
// PHE_CORR（组合逻辑）
// =========================================================================
wire signed [15:0] alpha_int    = alpha_odd_reg[31:16];
wire signed [24:0] corr_wire    = alpha_int * $signed(poly_Q8_reg);
wire signed [16:0] corr_shifted = corr_wire >>> 8;
 
wire signed [W_PHE-1:0] correction =
    (corr_shifted > $signed({1'b0, {(W_PHE-1){1'b1}}})) ? {1'b0, {(W_PHE-1){1'b1}}} :
    (corr_shifted < $signed({1'b1, {(W_PHE-1){1'b0}}})) ? {1'b1, {(W_PHE-1){1'b0}}} :
    corr_shifted[W_PHE-1:0];
 
wire signed [W_PHE:0] phe_sub = $signed(PHE) - $signed(correction);
 
assign PHE_CORR =
    (phe_sub > $signed({1'b0, {(W_PHE-1){1'b1}}})) ? {1'b0, {(W_PHE-1){1'b1}}} :
    (phe_sub < $signed({1'b1, {(W_PHE-1){1'b0}}})) ? {1'b1, {(W_PHE-1){1'b0}}} :
    phe_sub[W_PHE-1:0];
 
assign ALPHA_ODD           = alpha_int;
assign OFFSET_DTC_CTRL_10b = OFFSET_DTC_calib[11:2];
 
// =========================================================================
// 校准环路
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC          <= 12'd1881;
        OFFSET_DTC_calib <= 12'd0;
        PHR_F_Q          <= {W_FCWF{1'b0}};
        Chopping_EN_Q    <= 1'b0;
        DTC_P_Q          <= {W_DTC_CTRL{1'b0}};
        DTC_N_Q          <= {W_DTC_CTRL{1'b0}};
        MASH_SEL_Q       <= 1'b0;
        alpha_odd_reg    <= 32'd0;
        poly_Q8_reg      <= 9'd0;
    end else begin
 
        // ------------------------------------------------------------------
        // 流水线推进（所有 _Q 与当拍 PHE 对齐）
        // ------------------------------------------------------------------
        PHR_F_Q       <= PHR_F;
        Chopping_EN_Q <= Chopping_EN;
        DTC_P_Q       <= DTC_P_CTRL_10b;
        DTC_N_Q       <= DTC_N_CTRL_10b;
        MASH_SEL_Q    <= MASH_SEL;
 
        // ------------------------------------------------------------------
        // 1. INVKDTC 增益校准
        // ------------------------------------------------------------------
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
 
        // ------------------------------------------------------------------
        // 2. OFTDTC 偏置校准
        // ------------------------------------------------------------------
        OFFSET_DTC_calib <= OFFSET_DTC_calib + (CAL_OFTDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ Chopping_EN_Q) ? 12'h001 : 12'hFFF) : 1'b0);
 
        // ------------------------------------------------------------------
        // 3. POLY Legendre 校准
        //
        // u 归一化分母（对应 MATLAB max_diff_code）：
        //   MASH1:   max_dc = INVKDTC/8
        //            PHRF ∈ [-0.5, +0.5)，phi_half_max = 4096×0.25 = 1024
        //            max(P-N) = 2×1024×INVKDTC/16384 = INVKDTC/8
        //   MASH1-1: max_dc = INVKDTC/4
        //            PHRF = acc1-ov2 ∈ (-1, +1)，phi_half_max = 4096×0.5 = 2048
        //            max(P-N) = 2×2048×INVKDTC/16384 = INVKDTC/4
        // ------------------------------------------------------------------
        if (DIFF_EN)
            diff_c = $signed({1'b0, DTC_P_Q}) - $signed({1'b0, DTC_N_Q});
        else
            diff_c = $signed({1'b0, DTC_P_Q}) - 512;
 
        // MASH_SEL_Q 已对齐 DTC 码字
        max_dc = MASH_SEL_Q ? (INVKDTC >> 2) : (INVKDTC >> 3);
        if (max_dc == 0) max_dc = 1;
 
        u_int = diff_c * 256 / max_dc;
 
        // Chopping swap 时 u 取反（对应 MATLAB: if EN_SWAP: u_diff = -u_diff）
        if (Chopping_EN_Q) u_int = -u_int;
 
        // 钳位到 Q8 范围 [-256, +256]
        if      (u_int >  256) u_int =  256;
        else if (u_int < -256) u_int = -256;
 
        // P₃(u)×256：(5u³ - 3u)/2，Q8 格式
        u3_q8    = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;
 
        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;
 
        poly_Q8_reg <= poly_int[8:0];
 
        // sign-sign LMS 更新
        if (CAL_POLY_EN && poly_int != 0) begin
            if (~PHE_CORR[W_PHE-1] ^ (poly_int < 0))
                alpha_odd_reg <= alpha_odd_reg + MU_POLY;
            else
                alpha_odd_reg <= alpha_odd_reg - MU_POLY;
        end
 
    end
end
 
// =========================================================================
// SPI Readback
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC_OUT2SPI  <= 12'd0;
        OFTDTC_OUT2SPI   <= 12'd0;
    end else begin
        if (~INVKDTC_OUT2SPI_FREEZE) INVKDTC_OUT2SPI <= INVKDTC;
        if (~OFTDTC_OUT2SPI_FREEZE)  OFTDTC_OUT2SPI  <= OFFSET_DTC_calib;
    end
end
 
endmodule
 