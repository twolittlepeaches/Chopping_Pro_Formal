
/*
`timescale 1ps/1fs

// =============================================================================
// LMS_Calib.v  —  V2 修复版
//
// 修复说明（相对上一版本）：
//   Bug① 修复: u 来源从 PHR_F 改为 DTC_P_CTRL_10b − DTC_N_CTRL_10b
//              对应 MATLAB: u = (DCW_P − DCW_N) / max_diff_code
//   Bug② 修复: Chopping swap 时对 u 取反，与 PHE 翻转同步
//              对应 MATLAB: if EN_SWAP: u_diff = -u_diff
//   其他:      Chopping_EN 延迟链补全，INVKDTC/OFTDTC 校准改用 PHE_CORR
//
// 定点格式：
//   u_Q8      : u × 256，有符号整数，范围 [−256, +256] 对应 [−1.0, +1.0]
//   poly_Q8   : P₃(u) × 256，同上
//   alpha_odd : 32位有符号，高16位为整数部分（alpha_int），低16位为小数
//               PHE_CORR 补偿量 = alpha_int × poly_Q8 >> 8
//
// MATLAB 对应节次：
//   § 9  勒让德多项式补偿 (u 计算 + P₃ + PHE_CORR)
//   § 11 校准环路更新     (sign-sign LMS alpha_odd)
// =============================================================================

module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10,
    // MU_POLY: 32位累加器步长，高16位为整数部分
    // 等效步长 = MU_POLY / 2^16
    // 建议初始值 32'd64 (等效 ~0.001)，收敛后可减小
    parameter [31:0] MU_POLY = 32'd128
) (
    input  CKR,
    input  RST_N,

    // 相位误差（来自 phase_adder）
    input  signed [W_PHE-1:0]      PHE,

    // INVKDTC 增益校准
    input  [W_FCWF-1:0]            PHR_F,
    input                          CAL_INVKDTC_EN,
    input                          INVKDTC_OUT2SPI_FREEZE,
    output reg [11:0]              INVKDTC_OUT2SPI,
    output reg [11:0]              Tvd2_mis_OUT2SPI,
    output reg [W_INVKDTC-1:0]    INVKDTC,

    // OFFSET_DTC 校准
    input                          Chopping_EN,
    input                          CAL_OFTDTC_EN,
    input                          OFTDTC_OUT2SPI_FREEZE,
    output reg [11:0]              OFTDTC_OUT2SPI,
    output [9:0]                   OFFSET_DTC_CTRL_10b,

    // POLY Legendre 校准
    input  [W_DTC_CTRL-1:0]        DTC_P_CTRL_10b,   // 来自 phase_adder（已含 512 中心偏移）
    input  [W_DTC_CTRL-1:0]        DTC_N_CTRL_10b,
    input                          DIFF_EN,
    input                          CAL_POLY_EN,

    // 补偿后相位误差（输出给 Tracking 及其他校准环路）
    output signed [W_PHE-1:0]      PHE_CORR,

    // POLY 系数读出（调试用）
    output signed [15:0]           ALPHA_ODD
);

// =============================================================================
// 内部寄存器
// =============================================================================
reg  [W_FCWF-1:0]  PHR_F_Q;
reg                Chopping_EN_Q;       // 延迟 1 拍
reg                Chopping_EN_Q2;      // 延迟 2 拍，对齐 PHE 翻转时刻
reg  [11:0]        OFFSET_DTC_calib;

// POLY 寄存器
reg  signed [31:0] alpha_odd_reg;   // 32位：高16位整数，低16位小数
reg  signed [8:0]  poly_Q8_reg;     // P₃(u)×256，缓存供下拍 PHE_CORR 组合路径使用

// 行为级仿真用整数中间量
integer u_int;
integer u3_q8;
integer poly_int;

// =============================================================================
// PHE_CORR（组合逻辑）
//   correction = alpha_int × poly_Q8_reg >> 8
//   PHE_CORR   = saturate(PHE − correction)
// =============================================================================
wire signed [15:0] alpha_int    = alpha_odd_reg[31:16];
wire signed [24:0] corr_wire    = alpha_int * $signed(poly_Q8_reg);
wire signed [16:0] corr_shifted = corr_wire >>> 8;

// 饱和到 W_PHE 范围
wire signed [W_PHE-1:0] correction =
    (corr_shifted > $signed(12'sd2047))  ?  12'sd2047 :
    (corr_shifted < $signed(-12'sd2048)) ? -12'sd2048 :
    corr_shifted[W_PHE-1:0];

wire signed [W_PHE:0] phe_sub = $signed(PHE) - $signed(correction);

assign PHE_CORR =
    (phe_sub > $signed(13'sd2047))  ?  12'sd2047 :
    (phe_sub < $signed(-13'sd2048)) ? -12'sd2048 :
    phe_sub[W_PHE-1:0];

assign ALPHA_ODD           = alpha_int;
assign OFFSET_DTC_CTRL_10b = OFFSET_DTC_calib[11:2];

// =============================================================================
// 校准环路（时序逻辑）
// =============================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC          <= 12'd1881;
        PHR_F_Q          <= {W_FCWF{1'b0}};
        OFFSET_DTC_calib <= 12'd0;
        Chopping_EN_Q    <= 1'b0;
        Chopping_EN_Q2   <= 1'b0;
        alpha_odd_reg    <= 32'd0;
        poly_Q8_reg      <= 9'd0;
    end else begin

        // ------------------------------------------------------------------
        // 1. Chopping 延迟链
        //    Q1: 对齐 OFTDTC LMS 相关信号
        //    Q2: 对齐 PHE 翻转（phase_adder 里 Chopping_EN_Q2 控制 PHE 符号）
        // ------------------------------------------------------------------
        Chopping_EN_Q  <= Chopping_EN;
        Chopping_EN_Q2 <= Chopping_EN_Q;

        // ------------------------------------------------------------------
        // 2. INVKDTC 增益校准（sign-sign LMS，使用 PHE_CORR）
        //    sign(PHE_CORR) × sign(PHR_F) → INVKDTC ±1
        //    PHR_F_Q：延迟一拍与 PHE_CORR 对齐
        // ------------------------------------------------------------------
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
        PHR_F_Q <= PHR_F;

        // ------------------------------------------------------------------
        // 3. OFFSET_DTC 校准（sign-sign LMS，使用 PHE_CORR）
        //    sign(PHE_CORR) × sign(Chopping_EN_Q) → OFFSET ±1
        // ------------------------------------------------------------------
        OFFSET_DTC_calib <= OFFSET_DTC_calib + (CAL_OFTDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ Chopping_EN_Q) ? 12'h001 : 12'hFFF) : 1'b0);

        // ------------------------------------------------------------------
        // 4. POLY Legendre 校准
        //
        // Step A: 计算 u_diff（Q8 格式）
        //   DIFF 模式: u = (DCW_P − DCW_N) / (INVKDTC/4)
        //              max_diff_code = INVKDTC/4（对应 MATLAB 中 0.5×gain，
        //              因为 RTL 码字已含 ×2 的 Q8 因子）
        //   SE   模式: u = (DCW_P − 512)×2 / (INVKDTC/4)
        //              N路固定为512，差分退化为单端
        //
        // ★ Bug① 修复：u 来自真实 DTC 码差，不再用 PHR_F
        // ★ Bug② 修复：Chopping swap 时 u 取反，与 PHE 翻转同步
        // ------------------------------------------------------------------

        begin : poly_block
            // 有符号码差（11位，范围约 ±512）
            // DIFF: DCW_P − DCW_N，N 路也有效
            // SE:   (DCW_P − 512) × 2，等效为全幅差分
            reg signed [10:0] dcw_diff;
            reg        [9:0]  max_code;
            reg signed [19:0] u_shifted;

            if (DIFF_EN)
                dcw_diff = $signed({1'b0, DTC_P_CTRL_10b}) - $signed({1'b0, DTC_N_CTRL_10b});
            else
                dcw_diff = ($signed({1'b0, DTC_P_CTRL_10b}) - 11'sd512) * 2;

            // 归一化分母 = INVKDTC/4（取 bit[11:2]）
            max_code = INVKDTC[W_INVKDTC-1:2];

            // u_Q8 = dcw_diff × 256 / max_code
            // 为避免除法综合问题，这里保留行为级写法（仿真用）
            u_shifted = $signed(dcw_diff) * 20'sd256;
            if (max_code != 0)
                u_int = u_shifted / $signed({10'b0, max_code});
            else
                u_int = 0;

            // ★ Bug② 修复：Chopping Q2 对齐 PHE 翻转，u 同步取反
            if (Chopping_EN_Q2)
                u_int = -u_int;

            // 钳位到 Q8 范围 [−256, +256]
            if      (u_int >  256) u_int =  256;
            else if (u_int < -256) u_int = -256;
        end

        // Step B: P₃(u) × 256
        //   P₃(u) = 0.5 × (5u³ − 3u)
        //   u3_q8  = u³ / 256² = (u_Q8)³ / 2^16（Q8 格式）
        u3_q8    = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;

        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;

        poly_Q8_reg <= poly_int[8:0];

        // Step C: sign-sign LMS 更新
        //   alpha_odd += sign(PHE_CORR) × sign(P₃) × MU_POLY
        //   收敛条件：E[sign(PHE_CORR) × P₃(u)] = 0
        //   等价于 MATLAB: alpha_odd += mu_poly × sign(phie) × poly_val
        //
        //   XOR 真值表（注意：PHE_CORR MSB=0 → 正，poly_int>0 → 正）：
        //     ~MSB  ^  (poly<0)  →  同号=1→加，异号=0→减
        if (CAL_POLY_EN && poly_int != 0) begin
            if (~PHE_CORR[W_PHE-1] ^ (poly_int < 0))
                alpha_odd_reg <= alpha_odd_reg + MU_POLY;
            else
                alpha_odd_reg <= alpha_odd_reg - MU_POLY;
        end

    end
end

// =============================================================================
// SPI Readback
// =============================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC_OUT2SPI  <= 12'd0;
        OFTDTC_OUT2SPI   <= 12'd0;
        Tvd2_mis_OUT2SPI <= 12'd0;
    end else begin
        if (~INVKDTC_OUT2SPI_FREEZE) INVKDTC_OUT2SPI <= INVKDTC;
        if (~OFTDTC_OUT2SPI_FREEZE)  OFTDTC_OUT2SPI  <= OFFSET_DTC_calib;
    end
end

endmodule
*/
`timescale 1ps/1fs

// =============================================================================
// LMS_Calib.v  (严格对齐 MATLAB 的终极时序修复版)
//
// 核心修复：
// 1. 摒弃多级混乱延迟，建立统一的 1 拍延迟线（_Q），与抵达的 PHE 完美对齐。
// 2. 严格遵循 MATLAB 逻辑：u_diff = (DCW_P - DCW_N) / max_diff。
// 3. 修复了物理码字与 Chopping 符号还原时的时序错位问题。
// =============================================================================

module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10,
    parameter [31:0] MU_POLY = 32'd128 // 精细步长
) (
    input  CKR,
    input  RST_N,

    input  signed [W_PHE-1:0]      PHE,
    input  [W_FCWF-1:0]            PHR_F,
    input                          CAL_INVKDTC_EN,
    input                          INVKDTC_OUT2SPI_FREEZE,
    output reg [11:0]              INVKDTC_OUT2SPI,
    output reg [11:0]              Tvd2_mis_OUT2SPI, 
    output reg [W_INVKDTC-1:0]     INVKDTC,

    input                          Chopping_EN,
    input                          CAL_OFTDTC_EN,
    input                          OFTDTC_OUT2SPI_FREEZE,
    output reg [11:0]              OFTDTC_OUT2SPI,
    output [9:0]                   OFFSET_DTC_CTRL_10b,

    // 接收物理 DTC 控制码
    input  [W_DTC_CTRL-1:0]        DTC_P_CTRL_10b,    
    input  [W_DTC_CTRL-1:0]        DTC_N_CTRL_10b,
    input                          DIFF_EN,
    input                          CAL_POLY_EN,

    output signed [W_PHE-1:0]      PHE_CORR,
    output signed [15:0]           ALPHA_ODD
);

// =========================================================================
// 统一的 1 拍流水线寄存器 (Pipeline Registers)
// 保证所有计算用到的源数据，都与反馈回来的 PHE 属于同一个时钟周期
// =========================================================================
reg  [W_FCWF-1:0]       PHR_F_Q;
reg                     Chopping_EN_Q;
reg  [W_DTC_CTRL-1:0]   DTC_P_Q;  // 新增：DTC码字延迟线
reg  [W_DTC_CTRL-1:0]   DTC_N_Q;  // 新增：DTC码字延迟线

reg  [11:0]         OFFSET_DTC_calib;
reg  signed [31:0]  alpha_odd_reg;
reg  signed [8:0]   poly_Q8_reg;    

integer diff_c;
integer max_dc;
integer u_int;
integer u3_q8;
integer poly_int;

// =========================================================================
// PHE_CORR (相位补偿 - 组合逻辑)
// =========================================================================
wire signed [15:0] alpha_int = alpha_odd_reg[31:16];
wire signed [24:0] corr_wire = alpha_int * $signed(poly_Q8_reg);
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

assign ALPHA_ODD = alpha_int;
assign OFFSET_DTC_CTRL_10b = OFFSET_DTC_calib[11:2] + 10'd0;

// =========================================================================
// 校准环路更新
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC <= 12'd1881;
        OFFSET_DTC_calib <= 12'd0;
        
        PHR_F_Q <= 12'd0;
        Chopping_EN_Q <= 1'b0;
        DTC_P_Q <= 10'd0;
        DTC_N_Q <= 10'd0;

        alpha_odd_reg <= 32'd0;
        poly_Q8_reg   <= 9'd0;
    end else begin
        // -------------------------------------------------------------
        // 1. 推进延迟流水线 (此时_Q变量与当拍的PHE绝对对齐)
        // -------------------------------------------------------------
        PHR_F_Q       <= PHR_F;
        Chopping_EN_Q <= Chopping_EN;
        DTC_P_Q       <= DTC_P_CTRL_10b; // 关键修复：缓存物理码字
        DTC_N_Q       <= DTC_N_CTRL_10b; // 关键修复：缓存物理码字

        // -------------------------------------------------------------
        // 2. INVKDTC 和 OFTDTC 更新 (使用对齐后的 _Q 变量)
        // -------------------------------------------------------------
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ? 
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);

        OFFSET_DTC_calib <= OFFSET_DTC_calib + (CAL_OFTDTC_EN ? 
            ((~PHE_CORR[W_PHE-1] ^ Chopping_EN_Q) ? 12'h001 : 12'hFFF) : 1'b0);

        // -------------------------------------------------------------
        // 3. POLY 生成 (完全还原 MATLAB 逻辑)
        // -------------------------------------------------------------
        // 使用打了一拍的物理码字，确保与 Chopping_EN_Q 对齐
        if (DIFF_EN)
            diff_c = $signed({1'b0, DTC_P_Q}) - $signed({1'b0, DTC_N_Q});
        else
            diff_c = $signed({1'b0, DTC_P_Q}) - 512;

        max_dc = INVKDTC >> 3;
        if (max_dc == 0) max_dc = 1;

        u_int = diff_c * 256 / max_dc;

        // 还原 Chopping 符号 (MATLAB: if EN_SWAP: u_diff = -u_diff)
        if (Chopping_EN_Q) u_int = -u_int;

        // 钳位至 Q8 范围
        if      (u_int >  256) u_int =  256;
        else if (u_int < -256) u_int = -256;

        u3_q8 = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;

        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;

        poly_Q8_reg <= poly_int[8:0];

        // -------------------------------------------------------------
        // 4. POLY 系数更新
        // -------------------------------------------------------------
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
        INVKDTC_OUT2SPI <= 12'd0;
        OFTDTC_OUT2SPI  <= 12'd0;
    end else begin
        if (~INVKDTC_OUT2SPI_FREEZE) INVKDTC_OUT2SPI <= INVKDTC;
        if (~OFTDTC_OUT2SPI_FREEZE)  OFTDTC_OUT2SPI  <= OFFSET_DTC_calib;
    end
end

endmodule