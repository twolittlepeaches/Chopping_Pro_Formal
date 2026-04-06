/*
`timescale 1ps/1fs
              
module LMS_Calib #(
    parameter W_PHE = 12,
    parameter W_INVKDTC = 12,
    parameter W_FCWF = 12
) (
    input CKR,
    input RST_N,
    input [W_FCWF-1:0] PHR_F,
    input [W_PHE-1:0] PHE,
    input Chopping_EN,
    input CAL_INVKDTC_EN,
    input CAL_OFTDTC_EN,
    input INVKDTC_OUT2SPI_FREEZE,
    input OFTDTC_OUT2SPI_FREEZE,
    output reg [11:0] INVKDTC_OUT2SPI,
    output reg [11:0] Tvd2_mis_OUT2SPI,
    output reg [11:0] OFTDTC_OUT2SPI,
    output reg [W_INVKDTC-1:0] INVKDTC,
    output [9:0] OFFSET_DTC_CTRL_10b
);


reg [W_FCWF-1:0] PHR_F_Q;
reg Chopping_EN_Q;
reg [11:0] OFFSET_DTC_calib;

assign OFFSET_DTC_CTRL_10b = OFFSET_DTC_calib[11:2] +  10'd0;//10'd128


//----------------------------------------
// DTC Gain Calib
//----------------------------------------
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC <= 12'd1881;//1881
        PHR_F_Q <= 12'd0;
    end else begin
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ? ( (~PHE[W_PHE-1]^PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
        PHR_F_Q <= PHR_F;
    end
end


//----------------------------------------
// OFFSET DTC Calib
//----------------------------------------
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        OFFSET_DTC_calib <= 12'd0;//682，这个东西怎么算出来的
        Chopping_EN_Q <= 1'b0;
    end else begin
        OFFSET_DTC_calib <= OFFSET_DTC_calib + ( CAL_OFTDTC_EN ? ( (~PHE[W_PHE-1]^Chopping_EN_Q ) ? 12'h001 : 12'hFFF) : 1'b0);
        Chopping_EN_Q <= Chopping_EN;
    end
end


//-------------------------------OUT2SPI---------------------------------------//

always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC_OUT2SPI <= 12'd0;
    end else begin
        if (~INVKDTC_OUT2SPI_FREEZE) begin
            INVKDTC_OUT2SPI <= INVKDTC;
        end
    end
end

always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        OFTDTC_OUT2SPI <= 12'd0;
    end else begin
        if (~OFTDTC_OUT2SPI_FREEZE) begin
            OFTDTC_OUT2SPI <= OFFSET_DTC_calib;
        end
    end
end


endmodule
*/
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
/*
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
*/

/*
`timescale 1ps/1fs

module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10
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

    // POLY 端口保留，但内部直接使用纯净的 PHR_F_Q，彻底免疫 Chopping 时序干扰
    input  [W_DTC_CTRL-1:0]        DTC_P_CTRL_10b,    
    input  [W_DTC_CTRL-1:0]        DTC_N_CTRL_10b,
    input                          DIFF_EN,
    input                          CAL_POLY_EN,

    output signed [W_PHE-1:0]      PHE_CORR,
    output signed [15:0]           ALPHA_ODD
);

// =========================================================================
// 内部信号与寄存器
// =========================================================================
reg  [W_FCWF-1:0]   PHR_F_Q;
reg                 Chopping_EN_Q;
reg  [11:0]         OFFSET_DTC_calib;

// POLY内部寄存器 (32位小数寄存器，用于实现极小步长)
reg  signed [31:0]  alpha_odd_reg;
reg  signed [8:0]   poly_Q8_reg;    

// 直接从相位残差提取归一化变量 u (Q8格式)
wire signed [12:0] phr_centered = $signed({1'b0, PHR_F_Q}) - $signed(13'd2048);
wire signed [9:0]  u_int_comb   = phr_centered[12:3]; // [-256, 255]

integer u_int;
integer u3_q8;
integer poly_int;

// =========================================================================
// PHE_CORR (相位补偿)
// =========================================================================
// 提取 32 位寄存器的高 16 位参与实际补偿计算
wire signed [15:0] alpha_int = alpha_odd_reg[31:16];

// 16-bit * 9-bit = 25-bit
wire signed [24:0] corr_wire = alpha_int * $signed(poly_Q8_reg);
wire signed [16:0] corr_shifted = corr_wire >>> 8;

// 钳位保护
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
// 1. INVKDTC 增益校准
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC <= 12'd1881;
        PHR_F_Q <= 12'd0;
    end else begin
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ? 
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
        PHR_F_Q <= PHR_F;
    end
end

// =========================================================================
// 2. OFFSET_DTC 校准
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
// 3. POLY Legendre 校准 (重构后绝对免疫时序对齐问题)
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        alpha_odd_reg <= 32'd0;
        poly_Q8_reg   <= 9'd0;
    end else begin
        // Step A & B: 从纯净的中心化相位残差计算 P3(u)，完全避开 Chopping 的符号干扰
        u_int = u_int_comb;
        u3_q8 = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;

        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;

        poly_Q8_reg <= poly_int[8:0];

        // Step C: 32位小数寄存器带来 1fs 的平滑微步长 (32'd128)
        if (CAL_POLY_EN && poly_int != 0) begin
            if (~PHE_CORR[W_PHE-1] ^ (poly_int < 0))
                alpha_odd_reg <= alpha_odd_reg + 32'd128; // 可调步长：大步长可用 32'd1024
            else
                alpha_odd_reg <= alpha_odd_reg - 32'd128;
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

*/
`timescale 1ps/1fs

// =============================================================================
// LMS_Calib.v  (v3 — Chopping对齐修复 + MU_POLY参数化步长)
//
// 校准环路：
//   1. INVKDTC Gain Calib    — sign-sign LMS
//   2. OFFSET_DTC Calib      — sign-sign LMS
//   3. POLY Legendre Calib   — sign-sign LMS（奇函数INL三次项）
//
// Chopping时序修复：
//   phase_adder 中 PHE 符号基于 Chopping_EN_Q2（2拍延迟）翻转。
//   本模块内建 Chopping_EN_Q2_lms（同样2拍延迟），
//   用于 POLY 的 u 符号翻转，保证 sign(PHE)×sign(P3) 方向一致。
//
// u 计算基于 PHR_F_Q（非 DTC_P/N_CTRL），避免 DCW 流水线对齐问题：
//   u_int = (PHR_F_Q - 2048) >> 3  ∈ [-256, +256]
//   正比于 (acc1-0.5)，与 MATLAB u_diff 等价（差符号，alpha_odd收敛方向补偿）
//
// MU_POLY（32位步长参数）：
//   alpha_odd_reg 为32位小数累加器，alpha_int = alpha_odd_reg[31:16]
//   等效实数步长 = MU_POLY / 65536
//   64→0.001(精细)  512→0.008(默认)  4096→0.0625(快速)  65536→1(最快)
// =============================================================================
/*
module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10,
    parameter [31:0] MU_POLY = 32'd16384
) (
    input  CKR,
    input  RST_N,

    input  signed [W_PHE-1:0]      PHE,
    input  [W_FCWF-1:0]            PHR_F,
    input                           CAL_INVKDTC_EN,
    input                           INVKDTC_OUT2SPI_FREEZE,
    output reg [11:0]               INVKDTC_OUT2SPI,
    output reg [11:0]               Tvd2_mis_OUT2SPI,
    output reg [W_INVKDTC-1:0]     INVKDTC,

    input                           Chopping_EN,
    input                           CAL_OFTDTC_EN,
    input                           OFTDTC_OUT2SPI_FREEZE,
    output reg [11:0]               OFTDTC_OUT2SPI,
    output [9:0]                    OFFSET_DTC_CTRL_10b,

    input  [W_DTC_CTRL-1:0]         DTC_P_CTRL_10b,    // 保留接口，当前未使用
    input  [W_DTC_CTRL-1:0]         DTC_N_CTRL_10b,    // 保留接口，当前未使用
    input                           DIFF_EN,
    input                           CAL_POLY_EN,

    output signed [W_PHE-1:0]       PHE_CORR,
    output signed [15:0]            ALPHA_ODD
);

// =========================================================================
// 内部寄存器
// =========================================================================
reg  [W_FCWF-1:0]   PHR_F_Q;
reg                  Chopping_EN_Q;       // 1拍，用于OFFSET_DTC
reg                  Chopping_EN_Q2_lms;  // 2拍，用于POLY，与PHE翻转对齐
reg  [11:0]          OFFSET_DTC_calib;
reg  signed [31:0]   alpha_odd_reg;       // 32位小数累加器
reg  signed [8:0]    poly_Q8_reg;         // P3(u)×256，注册值

// =========================================================================
// u 组合逻辑（Q8格式）
//   PHR_F_Q = acc1 × 4096 ∈ [0, 4095]
//   phr_centered = (acc1 - 0.5) × 4096 ∈ [-2048, 2047]
//   u_int_comb = >> 3 = (acc1 - 0.5) × 512 ∈ [-256, 255]
// =========================================================================
wire signed [12:0] phr_centered = $signed({1'b0, PHR_F_Q}) - $signed(13'd2048);
wire signed [9:0]  u_int_comb   = phr_centered[12:3];

integer u_int, u3_q8, poly_int;

// =========================================================================
// PHE_CORR 组合逻辑
// =========================================================================
wire signed [15:0] alpha_int    = alpha_odd_reg[31:16];
wire signed [24:0] corr_wire    = alpha_int * $signed(poly_Q8_reg);  // 16b×9b=25b
wire signed [16:0] corr_shifted = corr_wire[24:8];                   // >>8

wire signed [W_PHE-1:0] correction =
    (corr_shifted > 17'sd2047)  ? 12'sd2047  :
    (corr_shifted < -17'sd2048) ? -12'sd2048 :
    corr_shifted[W_PHE-1:0];

wire signed [W_PHE:0] phe_sub = $signed(PHE) - $signed(correction);

assign PHE_CORR =
    (phe_sub > 13'sd2047)  ? 12'sd2047  :
    (phe_sub < -13'sd2048) ? -12'sd2048 :
    phe_sub[W_PHE-1:0];

assign ALPHA_ODD           = alpha_int;
assign OFFSET_DTC_CTRL_10b = OFFSET_DTC_calib[11:2] + 10'd0;

// =========================================================================
// 1. INVKDTC 增益校准
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC <= 12'd1881;
        PHR_F_Q <= 12'd0;
    end else begin
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ?
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
        PHR_F_Q <= PHR_F;
    end
end

// =========================================================================
// 2. OFFSET_DTC 校准 + Chopping 延迟链
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        OFFSET_DTC_calib   <= 12'd0;
        Chopping_EN_Q      <= 1'b0;
        Chopping_EN_Q2_lms <= 1'b0;
    end else begin
        OFFSET_DTC_calib <= OFFSET_DTC_calib + (CAL_OFTDTC_EN ?
            ((~PHE[W_PHE-1]^ Chopping_EN_Q) ? 12'h001 : 12'hFFF) : 1'b0);
        Chopping_EN_Q      <= Chopping_EN;       // 1拍
        Chopping_EN_Q2_lms <= Chopping_EN_Q;     // 2拍，对齐PHE翻转
    end
end

// =========================================================================
// 3. POLY Legendre 校准
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        alpha_odd_reg <= 32'd0;
        poly_Q8_reg   <= 9'd0;
    end else begin

        // Step A: u_int，Chopping Q2 对齐翻转
        u_int = u_int_comb;
        if (Chopping_EN_Q2_lms) u_int = -u_int;   // 与 PHE 翻转同步
        if      (u_int >  256) u_int =  256;
        else if (u_int < -256) u_int = -256;

        // Step B: P3(u) × 256
        u3_q8    = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;
        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;
        poly_Q8_reg <= poly_int[8:0];

        // Step C: sign-sign LMS，步长由 MU_POLY 参数控制
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
*/
`timescale 1ps/1fs

module LMS_Calib #(
    parameter W_PHE      = 12,
    parameter W_INVKDTC  = 12,
    parameter W_FCWF     = 12,
    parameter W_DTC_CTRL = 10,
    parameter [31:0] MU_POLY = 32'd512 // 精细步长
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

    // POLY 端口
    input  [W_DTC_CTRL-1:0]        DTC_P_CTRL_10b,    
    input  [W_DTC_CTRL-1:0]        DTC_N_CTRL_10b,
    input                          DIFF_EN,
    input                          CAL_POLY_EN,

    output signed [W_PHE-1:0]      PHE_CORR,
    output signed [15:0]           ALPHA_ODD
);

// =========================================================================
// 内部寄存器
// =========================================================================
reg  [W_FCWF-1:0]   PHR_F_Q;
reg                 Chopping_EN_Q;
reg  [11:0]         OFFSET_DTC_calib;

// POLY 内部寄存器 (32位小数寄存器)
reg  signed [31:0]  alpha_odd_reg;
reg  signed [8:0]   poly_Q8_reg;    

// 提取归一化变量 u (Q8)
wire signed [12:0] phr_centered = $signed({1'b0, PHR_F_Q}) - $signed(13'd2048);
// 注意：在差分模式下，DCW_P 随 PHR 增大而减小，此处加负号以统一收敛极性
wire signed [9:0]  u_int_comb   = -phr_centered[12:3]; 

integer u_int;
integer u3_q8;
integer poly_int;

// =========================================================================
// PHE_CORR (相位补偿)
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
// 校准环路更新 (恢复原版正确的收敛极性)
// =========================================================================
always @(posedge CKR, negedge RST_N) begin
    if (~RST_N) begin
        INVKDTC <= 12'd1881;
        PHR_F_Q <= 12'd0;
        OFFSET_DTC_calib <= 12'd0;
        Chopping_EN_Q    <= 1'b0;
        
        alpha_odd_reg <= 32'd0;
        poly_Q8_reg   <= 9'd0;
    end else begin
        // 1. INVKDTC (恢复 12'hFFF : 12'h001)
        INVKDTC <= INVKDTC + (CAL_INVKDTC_EN ? 
            ((~PHE_CORR[W_PHE-1] ^ PHR_F_Q[W_FCWF-1]) ? 12'hFFF : 12'h001) : 1'b0);
        PHR_F_Q <= PHR_F;

        // 2. OFTDTC (恢复 12'h001 : 12'hFFF)
        OFFSET_DTC_calib <= OFFSET_DTC_calib + (CAL_OFTDTC_EN ? 
            ((~PHE_CORR[W_PHE-1] ^ Chopping_EN_Q) ? 12'h001 : 12'hFFF) : 1'b0);
        Chopping_EN_Q <= Chopping_EN;

        // 3. POLY 生成 (免疫 Chopping，纯粹的数学映射)
        u_int = u_int_comb;
        u3_q8 = (u_int * u_int * u_int) / 65536;
        poly_int = (5 * u3_q8 - 3 * u_int) / 2;

        if      (poly_int >  256) poly_int =  256;
        else if (poly_int < -256) poly_int = -256;

        poly_Q8_reg <= poly_int[8:0];

        // 4. POLY 系数更新 (32位精细步长，消灭梯度噪声)
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

