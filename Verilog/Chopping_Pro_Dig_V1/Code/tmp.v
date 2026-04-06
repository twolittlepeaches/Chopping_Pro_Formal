`timescale 1ps/1fs

// =============================================================================
// LMS_Calib.v
//
// 校准环路：
//   1. INVKDTC Gain Calib      — sign-sign LMS, 相关信号: sign(PHE_CORR) × sign(PHR_F)
//   2. OFFSET_DTC Calib        — sign-sign LMS, 相关信号: sign(PHE_CORR) × sign(Chopping)
//   3. POLY Legendre Calib     — sign-sign LMS, 相关信号: sign(PHE_CORR) × sign(P3(u_diff))
//      消除DIFF模式残余的奇函数INL（三次项），与增益校准正交
//
// POLY算法（对应 Chopping_pro_td_v6.m 第9、11节）：
//   u_diff = (DCW_P - DCW_N) / (INVKDTC/8)     ← DIFF模式偏移自动抵消
//          = (DCW_P - 512)   / (INVKDTC/8)     ← SE模式（显式减中心）
//   P3(u)  = (5u³ - 3u) / 2                    ← Legendre三阶多项式，与u正交
//   correction = (ALPHA_ODD × P3_Q8) >> 8      ← PHE单位（TDC count）
//   PHE_CORR   = PHE - correction               ← 输出补偿后的相位误差
//   update: ALPHA_ODD ±= sign(PHE_CORR) × sign(P3)
//
// 定点格式：
//   u_Q8   : u × 256，有符号9位，范围 [-256, +256] = [-1.0, +1.0]
//   P3_Q8  : P3(u) × 256，同上
//   ALPHA_ODD: 16位有符号，±1/cycle步长，表示每单位P3对应的PHE补偿量（TDC counts/1）
// =============================================================================

module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10
) (
    input  CKR,
    input  RST_N,

    // 相位误差（来自phase_adder）
    input  signed [W_PHE-1:0]     PHE,

    // INVKDTC增益校准
    input  [W_FCWF-1:0]           PHR_F,
    input                          CAL_INVKDTC_EN,
    input                          INVKDTC_OUT2SPI_FREEZE,
    output reg [11:0]              INVKDTC_OUT2SPI,
    output reg [11:0]              Tvd2_mis_OUT2SPI,   // 保留
    output reg [W_INVKDTC-1:0]    INVKDTC,

    // OFFSET_DTC校准
    input                          Chopping_EN,
    input                          CAL_OFTDTC_EN,
    input                          OFTDTC_OUT2SPI_FREEZE,
    output reg [11:0]              OFTDTC_OUT2SPI,
    output [9:0]                   OFFSET_DTC_CTRL_10b,

    // POLY Legendre校准（新增）
    input  [W_DTC_CTRL-1:0]        DTC_P_CTRL_10b,    // 来自phase_adder（含512偏移）
    input  [W_DTC_CTRL-1:0]        DTC_N_CTRL_10b,
    input                          DIFF_EN,
    input                          CAL_POLY_EN,

    // 补偿后相位误差（输出给Tracking和其他校准环路）
    output signed [W_PHE-1:0]      PHE_CORR,

    // POLY系数读出（监测用，SPI后续再接）
    output signed [15:0]           ALPHA_ODD
);

// =========================================================================
// 内部信号
// =========================================================================
reg  [W_FCWF-1:0]  PHR_F_Q;
reg                 Chopping_EN_Q;
reg  [11:0]         OFFSET_DTC_calib;

// POLY内部寄存器
reg  signed [15:0]  alpha_odd_reg;   // 多项式系数（16位有符号）
reg  signed [8:0]   poly_Q8_reg;     // P3(u_diff)×256，缓存用于LMS更新

// 整数中间量（行为级仿真，用integer在always块内）
integer diff_c;     // DCW差分（有符号）
integer max_dc;     // 归一化分母 = INVKDTC/8
integer u_int;      // u_diff × 256（Q8），钳位前
integer u3_q8;      // u³ × 256（Q8）
integer poly_int;   // P3(u) × 256（Q8），钳位前
integer corr_int;   // 补偿量（PHE单位，TDC counts）

// =========================================================================
// PHE_CORR（组合逻辑：PHE - correction）
// =========================================================================
// correction = ALPHA_ODD × poly_Q8_reg / 256（Q8 → 整数）
// PHE_CORR   = PHE - correction（钳位到W_PHE范围）
wire signed [26:0] corr_wire    = $signed(alpha_odd_reg) * $signed(poly_Q8_reg);
wire signed [W_PHE-1:0] correction = corr_wire[W_PHE+7:8];  // >> 8（取Q8后的整数部分）
wire signed [W_PHE:0]   phe_sub    = $signed(PHE) - $signed(correction);

// 防溢出钳位
assign PHE_CORR = (phe_sub > $signed({1'b0, {(W_PHE-1){1'b1}}})) ? {1'b0, {(W_PHE-1){1'b1}}} :
                  (phe_sub < $signed({1'b1, {(W_PHE-1){1'b0}}})) ? {1'b1, {(W_PHE-1){1'b0}}} :
                  phe_sub[W_PHE-1:0];

assign ALPHA_ODD = alpha_odd_reg;

assign OFFSET_DTC_CTRL_10b = OFFSET_DTC_calib[11:2] + 10'd0;

// =========================================================================
// 1. INVKDTC 增益校准（sign-sign LMS，使用PHE_CORR）
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC <= 12'd1881;
        PHR_F_Q <= 12'd0;
    end else begin
        // sign(PHE_CORR) × sign(PHR_F_Q) → INVKDTC ±1
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
        PHR_F_Q <= PHR_F;
    end
end

// =========================================================================
// 2. OFFSET_DTC 校准（sign-sign LMS，使用PHE_CORR）
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        OFFSET_DTC_calib <= 12'd0;
        Chopping_EN_Q    <= 1'b0;
    end else begin
        OFFSET_DTC_calib <= OFFSET_DTC_calib + (CAL_OFTDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ Chopping_EN_Q) ? 12'h001 : 12'hFFF) : 1'b0);
        Chopping_EN_Q <= Chopping_EN;
    end
end

// =========================================================================
// 3. POLY Legendre 校准
//    对应MATLAB步骤9（P3计算+PHE补偿）和步骤11（alpha_odd更新）
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        alpha_odd_reg <= 16'd0;
        poly_Q8_reg   <= 9'd0;
    end else begin

        // ---------------------------------------------------------------
        // Step A：计算 u_diff（Q8格式，× 256）
        //   DIFF模式: DCW_P - DCW_N（两路都有+512偏移，差分时抵消）
        //   SE模式:   DCW_P - 512（显式减去中心）
        //   然后除以 INVKDTC/8（当前增益归一化分母）
        //   Chopping翻转：对应MATLAB中 if EN_SWAP: u_diff = -u_diff
        //   （因为phase_adder在Chopping时交换了P/N，差分符号翻转，
        //     需要还原以保证与PHE的正确相关性）
        // ---------------------------------------------------------------
        if (DIFF_EN)
            diff_c = $signed({1'b0, DTC_P_CTRL_10b}) - $signed({1'b0, DTC_N_CTRL_10b});
        else
            diff_c = $signed({1'b0, DTC_P_CTRL_10b}) - 512;

        max_dc = INVKDTC >> 3;
        if (max_dc == 0) max_dc = 1;

        u_int = diff_c * 256 / max_dc;

        // Chopping时还原符号（phase_adder交换P/N导致diff_c取反）
        if (Chopping_EN_Q) u_int = -u_int;

        // 钳位到 [-256, +256]
        if      (u_int >  256) u_int =  256;
        else if (u_int < -256) u_int = -256;

        // ---------------------------------------------------------------
        // Step B：计算 P3(u) × 256（Q8）
        //   P3(u) = (5u³ - 3u) / 2
        //   u_int = u × 256，所以：
        //   u³ × 256 = u_int³ / 256² = u_int³ / 65536
        //   P3_Q8 = (5×u³×256 - 3×u×256) / 2 = (5×u3_q8 - 3×u_int) / 2
        // ---------------------------------------------------------------
        u3_q8    = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;

        // 钳位
        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;

        // 注册poly，供下一拍组合路径（PHE_CORR）使用
        poly_Q8_reg <= poly_int[8:0];

        // ---------------------------------------------------------------
        // Step C：sign-sign LMS更新 ALPHA_ODD
        //   alpha_odd += sign(PHE_CORR) × sign(P3)
        //   PHE_CORR[MSB]=0 → positive → sign=+1
        //   P3[MSB]=0       → positive → sign=+1
        //   同号 → ALPHA_ODD +1，异号 → ALPHA_ODD -1
        // ---------------------------------------------------------------
        if (CAL_POLY_EN && poly_int != 0) begin
            // sign(PHE_CORR) XNOR sign(poly): 同号时为1（+1），异号时为0（-1）
            if (~PHE_CORR[W_PHE-1] ^ (poly_int < 0))
                alpha_odd_reg <= alpha_odd_reg + 16'h0001;  // +1
            else
                alpha_odd_reg <= alpha_odd_reg + 16'hFFFF;  // -1（二补数）
        end

    end
end

// =========================================================================
// SPI Readback（保持原接口，PHE_CORR版本更新INVKDTC_OUT2SPI）
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC_OUT2SPI <= 12'd0;
    end else begin
        if (~INVKDTC_OUT2SPI_FREEZE)
            INVKDTC_OUT2SPI <= INVKDTC;
    end
end

always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        OFTDTC_OUT2SPI <= 12'd0;
    end else begin
        if (~OFTDTC_OUT2SPI_FREEZE)
            OFTDTC_OUT2SPI <= OFFSET_DTC_calib;
    end
end

endmodule