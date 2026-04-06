/*
`timescale 1ps/1fs

module DTC_analog_10b (
    input vdd_ref,
    input vdd,
    input gnd,
    input [4:0] DTC_DL_cap_ctrl,
    input DTC_Iref,
    input DTC_vtop_tran,
    // Input signal
    input DTC_clk_ref,
    
    // Digital control inputs (from DTC_dig outputs)
    input [3:0] DTC_top_cap_ctrl_4bit_binary,
    input [62:0] DTC_top_cap_ctrl_6bit_unary,
    input [14:0] DTC_bot_cap_ctrl_4bit_unary,
    input DTC_pre_clk_test,        // unused, kept for interface matching
    input DTC_pre_clk_sel,         // unused, kept for interface matching
    
    // Output
    output reg OUT
);

// Parameters (adapted from csDTC.v)
parameter integer SEED = 13;
parameter real sigma_DNL = 0;      // DNL noise standard deviation (ps)
parameter real reso_binary = 1.0;  // Resolution per binary LSB (ps)
parameter real reso_unary = 8.0;   // Resolution per unary bit (ps)
parameter real base_delay = 600;   // Base delay (ps)

// Internal variables

integer ctrl_value_top_binary;
integer ctrl_value_top_unary;
integer ctrl_value_bot_unary;
integer total_ctrl_value;
integer i, a;
real DNL[1023:0];    // DNL array for 10-bit = 1024 codes
real INL[1023:0];    // INL array
real total_delay;

// Intermediate variable mapping
wire IN;
assign IN = DTC_clk_ref;


// Initialize DNL and INL (similar to csDTC.v)
initial begin
    for (i=0; i<1024; i=i+1) begin
        DNL[i] = 0.0*$dist_normal(($random(SEED)%10000), 0, sigma_DNL)*1e-3;
    end
    INL[0] = DNL[0];
    for (i=1; i<1024; i=i+1) begin
        INL[i] = INL[i-1] + DNL[i];
    end
end

// Convert binary code to integer (4-bit binary)
always @(DTC_top_cap_ctrl_4bit_binary) begin
    ctrl_value_top_binary = 0;
    a = 1;
    for (i=0; i<4; i=i+1) begin
        ctrl_value_top_binary = ctrl_value_top_binary + a*(DTC_top_cap_ctrl_4bit_binary[i]);
        a = a + a;
    end
end

// Count unary bits (6-bit unary -> 63 bits)
always @(DTC_top_cap_ctrl_6bit_unary) begin
    ctrl_value_top_unary = 0;
    for (i=0; i<63; i=i+1) begin
        ctrl_value_top_unary = ctrl_value_top_unary + DTC_top_cap_ctrl_6bit_unary[i];
    end
end

// Count bottom unary bits (4-bit unary -> 15 bits)
always @(DTC_bot_cap_ctrl_4bit_unary) begin
    ctrl_value_bot_unary = 0;
    for (i=0; i<15; i=i+1) begin
        ctrl_value_bot_unary = ctrl_value_bot_unary + DTC_bot_cap_ctrl_4bit_unary[i];
    end
end

// Calculate total control value (10-bit effective)
// Architecture: 4-bit binary (LSB) + 6-bit unary (MSB)
// Total range: 0 to 1023 (10 bits)
always @(*) begin
    // Combine: binary covers 0-15, each unary bit adds 16
    total_ctrl_value = ctrl_value_top_binary + (ctrl_value_top_unary * 16);
    
    // Clamp to 10-bit range
    if (total_ctrl_value > 1023)
        total_ctrl_value = 1023;
end

// Generate delay based on control value (similar to csDTC.v)
always @(IN) begin
    // Calculate total delay:
    // - Binary part: fine resolution
    // - Unary part: coarse resolution
    // - Add INL for non-ideality
    total_delay = base_delay + 
                  ctrl_value_top_binary * reso_binary + 
                  ctrl_value_top_unary * reso_unary + 
                  INL[total_ctrl_value];
    
    if (IN == 1'b1)
        OUT <= #(total_delay) IN;
    else
        OUT <= #140 IN;  
end

endmodule
*/


//For AMS
/*
`timescale 1ps/1fs

module DTC_analog_10b (
    input vdd_ref,
    input vdd,
    input gnd,
    input [4:0] DTC_DL_cap_ctrl,
    input DTC_Iref,
    input DTC_vtop_tran,
    // Input signal
    input DTC_clk_ref,
    // Digital control inputs (from DTC dig outputs)
    input [3:0] DTC_top_cap_ctrl_4bit_binary,
    input [62:0] DTC_top_cap_ctrl_6bit_unary,
    input [14:0] DTC_bot_cap_ctrl_4bit_unary,
    input DTC_pre_clk_test,    // unused, kept for interface matching
    input DTC_pre_clk_sel,     // unused, kept for interface matching
    // Output
    output reg OUT
);

// Parameters
parameter integer SEED = 13;
parameter real sigma_DNL = 0.0;
parameter real reso_binary = 0.5;
parameter real reso_unary = 8.0;
parameter real base_delay = 339.0;

// Internal
integer ctrl_value_top_binary;
integer ctrl_value_top_unary;
integer ctrl_value_bot_unary;
integer total_ctrl_value;
integer i, a;
real DNL [1023:0];
real INL [1023:0];
real total_delay;

// Map clock
wire IN = DTC_clk_ref;

//------------------------------------------------
// 1. Initialize DNL and INL (AMS-safe)
//------------------------------------------------
integer seed_tmp;
initial begin
    seed_tmp = SEED;
    for (i = 0; i < 1024; i = i + 1) begin
        // AMS-safe random: $dist_normal(seed, mean, sigma)
        DNL[i] = $dist_normal(seed_tmp, 0.0, sigma_DNL) * 1e-3;
    end
    INL[0] = DNL[0];
    for (i = 1; i < 1024; i = i + 1) begin
        INL[i] = INL[i-1] + DNL[i];
    end
end

//------------------------------------------------
// 2. Binary 4-bit decode
//------------------------------------------------
always @(DTC_top_cap_ctrl_4bit_binary) begin
    ctrl_value_top_binary = 0;
    a = 1;
    for (i=0; i<4; i=i+1) begin
        ctrl_value_top_binary = ctrl_value_top_binary + a * DTC_top_cap_ctrl_4bit_binary[i];
        a = a << 1;
    end
end

//------------------------------------------------
// 3. Unary (63 bits) count
//------------------------------------------------
always @(DTC_top_cap_ctrl_6bit_unary) begin
    ctrl_value_top_unary = 0;
    for (i=0; i<63; i=i+1)
        ctrl_value_top_unary = ctrl_value_top_unary + DTC_top_cap_ctrl_6bit_unary[i];
end

//------------------------------------------------
// 4. Bottom unary (15 bits)
//------------------------------------------------
always @(DTC_bot_cap_ctrl_4bit_unary) begin
    ctrl_value_bot_unary = 0;
    for (i=0; i<15; i=i+1)
        ctrl_value_bot_unary = ctrl_value_bot_unary + DTC_bot_cap_ctrl_4bit_unary[i];
end

//------------------------------------------------
// 5. Total control (10-bit effective)
//------------------------------------------------
always @(*) begin
    total_ctrl_value = ctrl_value_top_binary + ctrl_value_top_unary*16;
    if (total_ctrl_value > 1023)
        total_ctrl_value = 1023;
end

//------------------------------------------------
// 6. Generate delay
//------------------------------------------------
always @(IN) begin
    total_delay = base_delay +
                  ctrl_value_top_binary * reso_binary +
                  ctrl_value_top_unary * reso_unary +
                  INL[total_ctrl_value];

    if (IN)
        OUT <= #(total_delay) 1'b1;
    else
        OUT <= #140 1'b0;
end

endmodule
*/
/*
 * DTC_analog_10b.v — 10-bit DTC analog behavioral model
 *
 * INL建模说明（对应 Chopping_pro_td_v6.m）：
 * -----------------------------------------------
 * MATLAB 中对每个DTC实例的INL建模为：
 *
 *   u = (code - N_DTC/2) / (N_DTC/2)         % 归一化到 [-1, 1]
 *   INL_raw = A_INL_EVEN * u^2 + A_INL_ODD * u^3
 *   INL = detrend(INL_raw, 1)                 % 去除线性趋势（增益校准吸收一次项）
 *
 * 在RTL中：
 *   - total_ctrl_value 范围 0..1023（10-bit），中心点 = 512
 *   - u = (total_ctrl_value - 512) / 512.0
 *   - P路：系数为 A_INL_EVEN, A_INL_ODD（INL_MISMATCH=0）
 *   - N路：系数为 A_INL_EVEN*(1+INL_MISMATCH), A_INL_ODD*(1-INL_MISMATCH)
 *
 * 参数（单位 ps）：
 *   A_INL_EVEN   — 偶函数INL幅度（二次项系数，u=1时的峰值）默认 9.2 ps
 *   A_INL_ODD    — 奇函数INL幅度（三次项系数，u=1时的峰值）默认 0.8 ps
 *   INL_MISMATCH — P/N路失配比例，P路传0，N路传正值（如0.05表示5%）
 *   sigma_DNL    — 随机DNL噪声标准差（integer单位，实际ps = sigma_DNL * 1e-3）
 *
 * 注意：INL单位为ps，直接加入 total_delay（ps）。
 */

`timescale 1ps/1fs

module DTC_analog_10b (
    input vdd_ref,
    input vdd,
    input gnd,
    input [4:0] DTC_DL_cap_ctrl,
    input DTC_Iref,
    input DTC_vtop_tran,
    // Input signal
    input DTC_clk_ref,
    // Digital control inputs (from DTC dig outputs)
    input [3:0] DTC_top_cap_ctrl_4bit_binary,
    input [62:0] DTC_top_cap_ctrl_6bit_unary,
    input [14:0] DTC_bot_cap_ctrl_4bit_unary,
    input DTC_pre_clk_test,    // unused, kept for interface matching
    input DTC_pre_clk_sel,     // unused, kept for interface matching
    // Output
    output reg OUT
);

// -------------------------------------------------------
// Parameters
// -------------------------------------------------------
parameter integer SEED        = 13;
parameter real sigma_DNL      = 0.0;   // 随机DNL噪声，integer单位，ps = sigma_DNL*1e-3
parameter real reso_binary    = 0.5;   // ps per binary LSB
parameter real reso_unary     = 8.0;   // ps per unary bit
parameter real base_delay     = 339.0; // ps

// INL多项式建模参数（对应MATLAB INL_TYPE=3）
parameter real A_INL_EVEN     = 10;   // 偶函数INL系数（ps），u=1处峰值
parameter real A_INL_ODD      = 60.0;   // 奇函数INL系数（ps），u=1处峰值
parameter real INL_MISMATCH   = 0.0;   // P/N路失配，P路=0，N路=如0.05

// -------------------------------------------------------
// Internal variables
// -------------------------------------------------------
integer ctrl_value_top_binary;
integer ctrl_value_top_unary;
integer ctrl_value_bot_unary;
integer total_ctrl_value;
integer i, a;

real DNL     [1023:0];
real INL     [1023:0];
real INL_raw [1023:0];
real total_delay;

// 用于detrend的临时变量
real u_norm;
real a_even_eff, a_odd_eff;
real sum_i, sum_inl, sum_i_inl, sum_i2;
real lsq_slope, lsq_intercept;

// Map clock
wire IN = DTC_clk_ref;
integer seed_tmp;

// -------------------------------------------------------
// Initialize INL：多项式 + 去线性趋势 + 随机DNL
// -------------------------------------------------------
initial begin
    seed_tmp   = SEED;

    // 有效系数（考虑P/N路失配）
    a_even_eff = A_INL_EVEN * (1.0 + INL_MISMATCH);
    a_odd_eff  = A_INL_ODD  * (1.0 - INL_MISMATCH);

    // --- Step 1：计算原始多项式INL ---
    // u = (i - 512) / 512, 归一化到 [-1, 1]
    // INL_raw[i] = a_even * u^2 + a_odd * u^3    (单位 ps)
    for (i = 0; i < 1024; i = i + 1) begin
        u_norm     = (i - 512.0) / 512.0;
        INL_raw[i] = a_even_eff * u_norm * u_norm
                   + a_odd_eff  * u_norm * u_norm * u_norm;
    end

    // --- Step 2：detrend（最小二乘去线性趋势，对应MATLAB detrend(x,1)）---
    // 去掉一次线性分量，使增益校准环路能独立吸收一次项
    sum_i     = 0.0;
    sum_inl   = 0.0;
    sum_i_inl = 0.0;
    sum_i2    = 0.0;
    for (i = 0; i < 1024; i = i + 1) begin
        sum_i     = sum_i     + i;
        sum_inl   = sum_inl   + INL_raw[i];
        sum_i_inl = sum_i_inl + i * INL_raw[i];
        sum_i2    = sum_i2    + i * i;
    end
    lsq_slope     = (1024.0 * sum_i_inl - sum_i * sum_inl)
                  / (1024.0 * sum_i2    - sum_i * sum_i);
    lsq_intercept = (sum_inl - lsq_slope * sum_i) / 1024.0;

    for (i = 0; i < 1024; i = i + 1) begin
        INL_raw[i] = INL_raw[i] - lsq_intercept - lsq_slope * i;
    end

    // --- Step 3：随机DNL叠加（sigma_DNL=0时无随机分量）---
    // 先生成DNL，再累加成随机INL，最后叠加到多项式INL上
    for (i = 0; i < 1024; i = i + 1) begin
        DNL[i] = $dist_normal(seed_tmp, 0, sigma_DNL) * 1e-3; // ps
    end

    INL[0] = INL_raw[0] + DNL[0];
    for (i = 1; i < 1024; i = i + 1) begin
        // 多项式INL直接查表；随机DNL累积叠加
        INL[i] = INL_raw[i] + (INL[i-1] - INL_raw[i-1]) + DNL[i];
    end

    // --- Debug 输出：打印峰值和中心点INL供验证 ---
    $display("[DTC_INL] SEED=%0d  A_EVEN=%.2fps  A_ODD=%.2fps  MISMATCH=%.2f%%",
             SEED, a_even_eff, a_odd_eff, INL_MISMATCH*100);
    $display("[DTC_INL] INL@code0=%.3fps  INL@code512=%.3fps  INL@code1023=%.3fps",
             INL[0], INL[512], INL[1023]);
end

// -------------------------------------------------------
// Binary 4-bit decode
// -------------------------------------------------------
always @(DTC_top_cap_ctrl_4bit_binary) begin
    ctrl_value_top_binary = 0;
    a = 1;
    for (i = 0; i < 4; i = i + 1) begin
        ctrl_value_top_binary = ctrl_value_top_binary
                              + a * DTC_top_cap_ctrl_4bit_binary[i];
        a = a << 1;
    end
end

// -------------------------------------------------------
// Unary (63 bits) count
// -------------------------------------------------------
always @(DTC_top_cap_ctrl_6bit_unary) begin
    ctrl_value_top_unary = 0;
    for (i = 0; i < 63; i = i + 1)
        ctrl_value_top_unary = ctrl_value_top_unary
                             + DTC_top_cap_ctrl_6bit_unary[i];
end

// -------------------------------------------------------
// Bottom unary (15 bits)
// -------------------------------------------------------
always @(DTC_bot_cap_ctrl_4bit_unary) begin
    ctrl_value_bot_unary = 0;
    for (i = 0; i < 15; i = i + 1)
        ctrl_value_bot_unary = ctrl_value_bot_unary
                             + DTC_bot_cap_ctrl_4bit_unary[i];
end

// -------------------------------------------------------
// Total control (10-bit effective，clamp至合法范围)
// -------------------------------------------------------
always @(*) begin
    total_ctrl_value = ctrl_value_top_binary + ctrl_value_top_unary * 16;
    if (total_ctrl_value > 1023)
        total_ctrl_value = 1023;
end

// -------------------------------------------------------
// Generate delay（含INL）
// -------------------------------------------------------
always @(IN) begin
    total_delay = base_delay
                + ctrl_value_top_binary * reso_binary
                + ctrl_value_top_unary  * reso_unary
                + INL[total_ctrl_value];     // 多项式 + 随机DNL INL（ps）

    if (IN)
        OUT <= #(total_delay) 1'b1;
    else
        OUT <= #140 1'b0;
end

endmodule