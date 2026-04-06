%% ================================================================
%  DTC Chopping / Differential DTC / Dither PLL 行为级仿真  v2
%
%  v2 修改说明（相较 v1）：
%    1. CHOPPING_EN 与 DIFF_DTC_EN 可同时开启，互不冲突
%       - CHOPPING：控制 MAIN/OFFSET DTC 挂 ref/div 路径的随机交换
%       - DIFF_DTC：控制码计算方式（MAIN 和 OFFSET 互补，各承担一半相位）
%       - 两者同时开：差分 DTC + 随机路径交换，叠加抑制效果
%    2. INL_TYPE 新增 4 = 纯奇函数（线性 INL），用于单独验证奇函数 INL 的抑制效果
%       理论分析：Chopping 对奇函数 INL 无效（偶函数假设），
%                 Diff 理论上可抵消奇函数 INL（两路反向叠加）
% ================================================================
clc; clear; close all;

%% ============================================================
%  功能开关
%  CHOPPING_EN 和 DIFF_DTC_EN 现在可以独立控制，也可以同时开启
% ============================================================
CHOPPING_EN  = 0;   % 1=随机 chopping（MAIN DTC 随机交换 ref/div 路径）
DIFF_DTC_EN  = 0;   % 1=差分 DTC（MAIN+OFFSET 互补码，各承担半个相位）

DITHER_EN    = 0;   % 1=MAIN DTC 加 dither + TDC 端数字补偿
DITHER_AMP   = 512; % dither 幅度 ±LSB（可调）

%% ============================================================
%  INL 类型选择
%  1 = 纯理想，无INL
%  2 = 纯偶函数（抛物线，VS-DTC 主要非线性，Chopping 最有效）
%  3 = 偶函数（主）+ 奇函数（次）（混合型，更接近实际）
%  4 = 正弦形 INL（原始代码风格）
%  5 = 纯奇函数（线性斜坡，用于单独验证 Diff/Chopping 对奇函数的效果）
%      理论预期：Chopping 对奇 INL 无效，Diff 可以抵消奇 INL
% ============================================================
INL_TYPE         = 1;      % 选择 INL 类型
A_INL_EVEN       = 10e-12; % 偶函数 INL 幅度（ps）
A_INL_ODD        = 2e-12; % 奇函数 INL 幅度（type4 时建议与 EVEN 同量级）
INL_MISMATCH     = 0.05;   % 两路 DTC INL 幅度失配比例（5%）

%% ============================================================
%  基本 PLL 参数
% ============================================================
rng('default'); rng(23);
Sim_Time = 2^19;        % 仿真周期数（FREF 周期）
settle_sample = 10e6;   % 单位：CKV cycles

fref = 100e6;
Tref = 1 / fref;
f0   = 5.5e9;
T0   = 1 / f0;

% near-integer 信道，spur 在 fref/256 ≈ 390 kHz
fcw   = 55 + 1/256;
FCW_I = floor(fcw);
FCW_F = mod(fcw, 1);

fv = fcw * fref;
Tv = 1 / fv;
fv_min = 4.0e9;
Tv_min = 1 / fv_min;

% 分数相位锯齿序列
PHRF_full = mod(cumsum(FCW_F * ones(1, Sim_Time)), 1);
PHRF_full(PHRF_full >= 1) = 0;
OV = [0, (PHRF_full(2:end) - PHRF_full(1:end-1)) < 0];

fprintf('FCW = %g, FCW_F = 1/%.0f\n', fcw, 1/FCW_F);
fprintf('预期 spur 位置: %.2f kHz\n', FCW_F*fref/1e3);

%% ============================================================
%  DTC / TDC 基本参数
% ============================================================
delta_TDC = 0.5e-15;
DTC_reso  = 4e-15;% 4fs
KDCO      = 10e3;
NB_dco    = 5;
Mdcorun   = 2;

N_DTC = round(Tv_min / DTC_reso); % DTC range , fv_min = 4.0e9 -> N_DTC

%% ============================================================
%  噪声参数
% ============================================================
f_offset        = 1e6;
S_DCO_offset_dB = -126;
S_DCO_offset    = 10^(S_DCO_offset_dB/10);
sigma_w         = f_offset/fv * sqrt(Tv) * sqrt(S_DCO_offset);
delta_Tw        = sigma_w * randn(round(Sim_Time*fcw*Mdcorun), 1);

S_DCO_thermal_dB = -160;
S_DCO_thermal    = 10^(S_DCO_thermal_dB/10);
sigma_j          = 1/(2*pi*fv)*sqrt(S_DCO_thermal*fv);
delta_Tj         = sigma_j*randn(round(Sim_Time*fcw*Mdcorun),1);

S_ref_dB   = -170;
S_ref      = 10^(S_ref_dB/10);
sigma_ref  = 1/(2*pi*fref)*sqrt(S_ref*fref);
delta_Tref = sigma_ref*randn(Sim_Time,1);

%% ============================================================
%  环路滤波器参数
% ============================================================
alpha = 2^(-5);
rho   = 2^(-12);
I_path = 0;
yiir1=0; yiir2=0; yiir3=0; yiir4=0;
lambda1=1; lambda2=1; lambda3=1; lambda4=1;

%% ============================================================
%  变量初始化
% ============================================================
otw         = zeros(Sim_Time+1, 1);
tR          = zeros(Sim_Time, 1);
tR_norm     = zeros(Sim_Time, 1);
trise       = zeros(Sim_Time, 1);
phie        = zeros(Sim_Time, 1);
Delta_TDEV  = zeros(Sim_Time, 1);
t_main      = 0;
t_offset    = 0;

Rv          = zeros(round(Sim_Time*fcw*Mdcorun), 1);
TDEV        = zeros(size(Rv));
TDEVpn      = zeros(size(Rv));
tckv        = zeros(size(Rv));
tckv_div    = zeros(size(Rv));
tckv_period = zeros(size(Rv));
tckv_norm   = zeros(size(Rv));

dither_actual      = zeros(Sim_Time, 1);
PHRF_cur_log       = zeros(Sim_Time, 1);
DCW_main_log       = zeros(Sim_Time, 1);
DCW_offset_log     = zeros(Sim_Time, 1);
DCW_delta_log      = zeros(Sim_Time, 1);
koft_log      = zeros(Sim_Time, 1);
kdtc_log      = zeros(Sim_Time, 1);
DTC_Delay_time_error_log = zeros(Sim_Time, 1);

DCW_main_ideal    = 0;
DCW_main_actual   = 0;
DCW_offset_ideal  = 0;
DCW_offset_actual = 0;

%% ============================================================
%  Chopping 序列
%  CHOPPING_EN=1：随机 0/1，决定 MAIN DTC 挂 ref 还是 div
%  CHOPPING_EN=0：固定 SWAP=0（MAIN 始终在 ref 侧）
% ============================================================
if CHOPPING_EN
    EN_SWAP = randi([0,1], 1, Sim_Time);
else
    EN_SWAP = zeros(1, Sim_Time);
end

%% ============================================================
%  Dither 序列
% ============================================================
if DITHER_EN
    dither_seq = randi([-DITHER_AMP, DITHER_AMP], 1, Sim_Time);
else
    dither_seq = zeros(1, Sim_Time);
end

%% ============================================================
%  增益和偏置校准初始值
% ============================================================
gain_DTC_est = 0.9 * Tv / DTC_reso;
Koft         = 300e-12;
step_DCO     = 0;

%% ============================================================
%  INL 建模
%  code_vec: [0, N_DTC-1]，u 归一化到 [-1, 1]（以中点为原点）
% ============================================================
code_vec = (0 : N_DTC-1);
u = (code_vec - N_DTC/2) / (N_DTC/2);

switch INL_TYPE
    case 1
        % 理想，无INL
        INL_main_raw   = 1 * u;
        INL_offset_raw = 1 * u;

    case 2
        % 纯偶函数：抛物线
        % Chopping 最有效，Diff 对偶函数也有抑制（两路相同，相减为0）
        INL_main_raw   = A_INL_EVEN * u.^2;
        INL_offset_raw = A_INL_EVEN * (1 + INL_MISMATCH) * u.^2;

    case 3
        % 偶函数（主）+ 奇函数（次）：更接近实际 VS-DTC
        INL_main_raw   = A_INL_EVEN * u.^2 + A_INL_ODD * u.^3;
        INL_offset_raw = A_INL_EVEN*(1+INL_MISMATCH)* u.^2 ...
                       + A_INL_ODD*(1-INL_MISMATCH)* u.^3;

    case 4
        % 正弦形 INL（原始代码风格）
        INL_main_raw   = A_INL_EVEN * sin(3*pi*code_vec/N_DTC);
        INL_offset_raw = A_INL_EVEN*(1+INL_MISMATCH) * sin(3*pi*code_vec/N_DTC);

    case 5
        % ★ 纯奇函数：线性斜坡（INL ∝ u，关于原点反对称）
        %
        % 物理意义：增益线性误差（DTC 步长随输入码单调变化）
        %
        % 理论分析：
        %   - Chopping: MAIN 从 ref 移到 div，INL_main(x) → INL_main(-x)
        %               奇函数：INL(-x) = -INL(x)，误差方向反了！
        %               → 两个 SWAP 状态的 INL 误差符号相反，
        %                 随机 Chopping 并不会抵消（反而来回跳），无法抑制 spur
        %   - Diff DTC: MAIN 码 = +x/2，OFFSET 码 = -x/2（互补）
        %               trise 中出现 INL(+x/2) - INL(-x/2)
        %               奇函数：INL(+x/2) - INL(-x/2) = 2*INL(x/2) ≠ 0
        %               → 差分结构对纯奇函数 INL 无法完全抵消！
        %               但如果两路 INL 完全相同，差分可以抵消偶函数分量；
        %               对奇函数，差分反而会"相加"而非相减
        %
        % 结论：纯奇函数 INL 需要数字预失真或 LMS 校准来处理
        % INL_main_raw   = A_INL_ODD * u;
        % INL_offset_raw = A_INL_ODD * (1 + INL_MISMATCH) * u;
        INL_main_raw   = A_INL_ODD * (u.^3);
        INL_offset_raw = A_INL_ODD * (1 + INL_MISMATCH) * (u.^3);

    otherwise
        error('INL_TYPE 必须为 1~5');
end

% 去线性趋势：排除增益误差，只保留非线性部分
INL_main   = detrend(INL_main_raw,   1);
INL_offset = detrend(INL_offset_raw, 1);

%% ============================================================
%  INL 查表函数（带边界保护）
% ============================================================
get_INL_main   = @(c) INL_main(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));
get_INL_offset = @(c) INL_offset(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));

%% ============================================================
%  ★★★ 主仿真循环 ★★★
% ============================================================
fprintf('开始仿真，Sim_Time = 2^%.0f = %d FREF cycles...\n', log2(Sim_Time), Sim_Time);
fprintf('CHOPPING_EN=%d, DIFF_DTC_EN=%d, DITHER_EN=%d, INL_TYPE=%d\n', ...
    CHOPPING_EN, DIFF_DTC_EN, DITHER_EN, INL_TYPE);

for step = 1:Sim_Time

    %% -- 1. FREF 时间戳 --
    tR(step)      = step * Tref + delta_Tref(step);
    tR_norm(step) = step * Tref;

    %% -- 2. DCO 计数 --
    if step == 1
        Delta_TDEV(1) = T0 - 1/(f0 + otw(1)*KDCO);
    else
        Delta_TDEV(step) = T0 - 1/(f0 + otw(step)*KDCO);
    end

    div_cnt = 0;
    while div_cnt < (FCW_I + OV(step))
        step_DCO = step_DCO + 1;
        if step_DCO == 1
            TDEV(1)        = Delta_TDEV(1);
            TDEVpn(1)      = delta_Tj(1) + delta_Tw(1);
            tckv(1)        = T0 - TDEV(1) + TDEVpn(1);
            tckv_norm(1)   = Tv;
            tckv_div(1)    = tckv(1) - step_DCO*Tv;
            tckv_period(1) = tckv(1);
            Rv(1)          = 1;
        else
            TDEV(step_DCO)   = TDEV(step_DCO-1) + Delta_TDEV(step);
            TDEVpn(step_DCO) = TDEVpn(step_DCO-1) ...
                             + delta_Tj(step_DCO) - delta_Tj(step_DCO-1) ...
                             + delta_Tw(step_DCO);
            tckv(step_DCO)        = step_DCO*T0 - TDEV(step_DCO) + TDEVpn(step_DCO);
            tckv_norm(step_DCO)   = step_DCO*Tv;
            tckv_div(step_DCO)    = tckv(step_DCO) - step_DCO*Tv;
            tckv_period(step_DCO) = tckv(step_DCO) - tckv(step_DCO-1);
            Rv(step_DCO)          = Rv(step_DCO-1) + 1;
        end
        div_cnt = div_cnt + 1;
    end

    %% -- 3. 分数相位（中心化到 [-0.5, 0.5)）--
    PHRF_cur = PHRF_full(step) - 0.5;
    PHRF_cur_log(step) = PHRF_cur;

    %% -- 4. DTC 码计算 --
    %
    %  逻辑说明：
    %
    %  [单独 Chopping，DIFF_DTC_EN=0]
    %    SWAP=0: MAIN 在 ref 侧，code = -(PHRF_cur)（延时用于对齐 ref）
    %    SWAP=1: MAIN 在 div 侧，code = +(PHRF_cur)（延时用于对齐 div）
    %    OFFSET DTC 提供固定偏置 Koft，随 SWAP 反向
    %
    %  [单独 Diff，CHOPPING_EN=0]
    %    MAIN   code = -(PHRF_cur+0.5)/2 * gain（负向，承担一半相位）
    %    OFFSET code = +(PHRF_cur+0.5)/2 * gain（正向，互补）
    %    SWAP 固定为 0（MAIN 在 ref 侧，OFFSET 在 div 侧）
    %
    %  [Chopping + Diff 同时开启]
    %    Diff 负责码字计算（互补分担相位）
    %    Chopping 负责随机交换路径（随机化 INL 序列）
    %    两者叠加：随机 SWAP 下，MAIN/OFFSET 互补码的 INL 误差被随机化
    %    期望：对偶函数 INL 的抑制更强（Chopping 的贡献）
    %         对奇函数 INL 的抑制仍有限（需校准）
    %
    %  trise 计算：
    %    SWAP=0: trise = (tckv + t_offset) - (tR + t_main)  [MAIN 在 ref 侧]
    %    SWAP=1: trise = (tckv + t_main)   - (tR + t_offset) [MAIN 在 div 侧]

   % if DIFF_DTC_EN 
   %      if EN_SWAP(step) > 0.5 % Chopping = 1. DIFF = 1
   %          % SWAP=1: MAIN 接 div，OFFSET 接 ref
   %          DCW_main_ideal   = +((PHRF_cur+0.5)/2) * gain_DTC_est;
   %          DCW_offset_ideal = -((PHRF_cur+0.5)/2) * gain_DTC_est;
   %      else                   % Only DIFF = 1
   %          % SWAP=0: MAIN 接 ref，OFFSET 接 div
   %          % 【关键修复】路径换了，码字必须跟着反转，才能保证整体延时方向不变！
   %          DCW_main_ideal   = -((PHRF_cur+0.5)/2) * gain_DTC_est;
   %          DCW_offset_ideal = +((PHRF_cur+0.5)/2) * gain_DTC_est;
   %      end
   % else                        % Only Chop = 1
   %      if EN_SWAP(step) > 0.5
   %          DCW_main_ideal = +PHRF_cur * gain_DTC_est;
   %      else
   %          DCW_main_ideal = -PHRF_cur * gain_DTC_est;
   %      end
   %      DCW_offset_ideal = 0; 
   %  end
    
if DIFF_DTC_EN 
    % 此时码字是绕 0 对称的
    DCW_main_ideal   = -(PHRF_cur / 2) * gain_DTC_est; 
    DCW_offset_ideal = +(PHRF_cur / 2) * gain_DTC_est;
    
    if EN_SWAP(step) > 0.5
        % 交换路径
        tmp = DCW_main_ideal;
        DCW_main_ideal = DCW_offset_ideal;
        DCW_offset_ideal = tmp;
    end
 else                        % Only Chop = 1
    if EN_SWAP(step) > 0.5
            DCW_main_ideal = +PHRF_cur * gain_DTC_est;
    else
            DCW_main_ideal = -PHRF_cur * gain_DTC_est;
    end
    DCW_offset_ideal = 0;
end

    DCW_main_actual = DCW_main_ideal + dither_seq(step);
    DCW_main_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_main_actual));

    dither_actual(step) = DCW_main_actual - DCW_main_ideal;

    DCW_offset_actual = DCW_offset_ideal;
    DCW_offset_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_offset_actual));


    t_main   = DCW_main_actual   * DTC_reso + 250e-12 + get_INL_main(DCW_main_actual);
    % if EN_SWAP(step) > 0.5
    %          t_offset = DCW_offset_actual * DTC_reso + 200e-12 + Koft + get_INL_offset(DCW_offset_actual);
    % else
    %          t_offset = DCW_offset_actual * DTC_reso + 200e-12 + get_INL_offset(DCW_offset_actual);
    % end
    t_offset = DCW_offset_actual * DTC_reso + 200e-12 + Koft + get_INL_offset(DCW_offset_actual);


    % 记录调试信息
    DCW_main_log(step)   = DCW_main_actual;
    DCW_offset_log(step) = DCW_offset_actual;

    %% -- 5. TDC 测量（trise）--
    %  SWAP=0: MAIN 在 ref 侧，OFFSET 在 div 侧
    %  SWAP=1: MAIN 在 div 侧，OFFSET 在 ref 侧
    if EN_SWAP(step) > 0.5   % MAIN 在 div 侧
        trise(step) = (tckv(step_DCO) + t_main) - (tR(step) + t_offset);
        DCW_delta_log(step) = (t_main - t_offset) / DTC_reso;
        DTC_Delay_time_error_log(step) = get_INL_main(DCW_main_actual) ...
                                       - get_INL_offset(DCW_offset_actual);
    else                      % MAIN 在 ref 侧
        trise(step) = (tckv(step_DCO) + t_offset) - (tR(step) + t_main);
        DCW_delta_log(step) = (t_offset - t_main) / DTC_reso;
        DTC_Delay_time_error_log(step) = get_INL_offset(DCW_offset_actual) ...
                                       - get_INL_main(DCW_main_actual);
    end

    %% -- 6. Dither 数字补偿 --
    if DITHER_EN
        dither_delay = dither_actual(step) * DTC_reso;
        if EN_SWAP(step) > 0.5
            trise(step) = trise(step) - dither_delay;  % MAIN 在 div 侧
        else
            trise(step) = trise(step) + dither_delay;  % MAIN 在 ref 侧
        end
    end

    %% -- 7. TDC 量化 --
    phie(step) = (floor(trise(step)/delta_TDC) - 0.5) * delta_TDC / Tv;

    %% -- 8. 校准环路（Sign-LMS）--

    % OFFSET DTC 偏置校准（仅单路 Chopping 模式）：
    % 如果 phie 和 SWAP 相关，说明 Koft 不对，调整之
    %if CHOPPING_EN && ~DIFF_DTC_EN
    if CHOPPING_EN 
        Koft = Koft + 2e-14 * sign(EN_SWAP(step) - 0.5) * sign(phie(step));
    end

    % DTC 增益校准：让 phie 和 PHRF_cur 去相关
    gain_DTC_est = gain_DTC_est + (-1) * 1e-1 * sign(phie(step)) * sign(PHRF_cur);

    koft_log(step) = Koft;
    kdtc_log(step) = gain_DTC_est;

    %% -- 9. 环路滤波器 --
    I_path = I_path + phie(step);
    yiir1  = (1-lambda1)*yiir1 + lambda1*phie(step);
    yiir2  = (1-lambda2)*yiir2 + lambda2*yiir1;
    yiir3  = (1-lambda3)*yiir3 + lambda3*yiir2;
    yiir4  = (1-lambda4)*yiir4 + lambda4*yiir3;

    P_path = alpha * yiir4;
    LFout  = P_path + rho * I_path;

    otw(step+1) = floor(LFout * fref/KDCO * 2^NB_dco) / 2^NB_dco;

end

fprintf('仿真完成，step_DCO = %d\n', step_DCO);

%% ============================================================
%  稳态区间截取
% ============================================================
settle_start_fref = round(Sim_Time * 0.4);
settle_start_dco  = round(step_DCO * 0.4);

%% ============================================================
%  Plot 1: CKV 瞬时频率偏差
% ============================================================
if 1 % Determine plot or not.
    fckv_div = 1./tckv_period(1:settle_sample) - fv;% 瞬时频率误差
    fckv_div_avg = zeros(size(fckv_div));
    N_fckv_div_avg = 25;
    for num = 1: settle_sample
        if num < N_fckv_div_avg
            fckv_div_avg(num) = mean(fckv_div(1:num+N_fckv_div_avg));
        elseif (num > (settle_sample-N_fckv_div_avg))
            fckv_div_avg(num) = mean(fckv_div(num-N_fckv_div_avg+1:settle_sample));
        else
            fckv_div_avg(num) = mean(fckv_div(num-N_fckv_div_avg+1:num+N_fckv_div_avg));
        end
    end
    figure('Name', 'Freq Deviation');
    plot(1e6*tckv(1:settle_sample),fckv_div/1e6);
    hold on
    plot(1e6*tckv(1:settle_sample),fckv_div_avg/1e6,'r-');
    grid on;
    xlabel('Time (us)','fontsize',16);
    ylabel('Freq Deviation (MHz)','fontsize',16);
    %xlim([0*Tv*1e6 settle_sample*Tv*1e6])
    set(gca,'YTick',-100:5:100);%This is just to tune the y-axis step, rather than the range.
    set(gca,'fontsize',16)
    set(findobj(gca,'Type','line'),'LineWidth',1)
    %set(gcf,'color','w');
end

%% ============================================================
%  Plot 2: 瞬时相位误差 phie
% ============================================================
if 1
    figure('Name', 'Phase Deviation');
    plot((1:Sim_Time)*Tref*1e6, phie, 'b', 'LineWidth', 0.5);
    grid on;
    xlabel('Time (us)', 'fontsize', 16);
    ylabel('Phase Deviation \phi_e', 'fontsize', 16);
    set(gca, 'fontsize', 16);
    set(findobj(gca,'Type','line'), 'LineWidth', 1);
    %set(gcf, 'color', 'w');
end

%% ============================================================
%  Plot 3: CKV 周期偏差
% ============================================================
if 1
    figure('Name', 'Period Deviation');
    period_div = (tckv_div(1:step_DCO)-mean(tckv_div(settle_sample:step_DCO)));
    plot(tckv_norm(1:step_DCO)*1e6,period_div*1e12);
    hold on;grid on;
    xlabel('Time (us)','fontsize',20)
    ylabel('Period Deviation (ps)','fontsize',20);
    %xlim([0*Tv settle_sample*Tv*1e6]);
    set(gca,'YTick',-2500:500:2500);
    set(gca,'fontsize',16);
    set(findobj(gca,'Type','line'),'LineWidth',1);
    %set(gcf,'color','w');
end

%% ============================================================
%  S 域噪声模型
% ============================================================
figure('Name', 'Phase Noise');
rbw   = 1e3;
fstep = rbw;
f     = 0:rbw:fv-rbw;
PN_tdc = (2*pi)^2/12*(delta_TDC/Tv)^2/fref * ones(size(f));
PN_vco = S_DCO_offset * (f_offset./f).^2;
Hol    = (alpha + rho*fref./(1 - exp(-2*pi*1j*f/fref))/fref) ...
         .* fref ./ (1 - exp(-2*pi*1j*f/fv)) / fv .* sinc(f*Tref);
Hcl_ref = fcw * Hol ./ (1+Hol);
Hcl_tdc = Hol ./ (1+Hol);
Hcl_dco = 1 ./ (1+Hol);
Scl_ref = S_ref * abs(Hcl_ref.^2);
Scl_tdc = PN_tdc .* abs(Hcl_tdc.^2);
Scl_dco = PN_vco .* abs(Hcl_dco.^2);
Scl_tot = Scl_ref + Scl_tdc + Scl_dco;
semilogx(f, 10*log10(Scl_ref), 'b--'); grid on; hold on;
semilogx(f, 10*log10(Scl_tdc), 'b-');
semilogx(f, 10*log10(Scl_dco), 'b-');
semilogx(f, 10*log10(Scl_tot), 'r--', 'LineWidth', 1.5, 'DisplayName', 's domain');
PN_dcoQ = 1/12*(KDCO/2^NB_dco./f).^2/fref.*sinc(f/fref).^2;
semilogx(f, 10*log10(PN_dcoQ.*abs(Hcl_dco.^2)), 'b-');

PHE    = tckv_div(settle_sample:step_DCO) / Tv * 2*pi;
PHE    = PHE - mean(PHE);

fprintf('\nReal td:\n');
[Px_sine, f_psd] = fun_calc_psd_dbs(PHE, fv, rbw);
f_psd  = f_psd(1:floor(length(f_psd)/2));
Px_sine= Px_sine(1:floor(length(Px_sine)/2));
semilogx(f_psd, Px_sine, 'g-','LineWidth', 1, 'DisplayName', 'time domain');
Px_R   = 10.^(Px_sine/10);
fstep_psd = f_psd(2) - f_psd(1);
sum_Y  = sum(Px_R(f_psd > 1e1)) * fstep_psd;
jitter = sqrt(2*sum_Y) / (2*pi*fv);
xlim([1e3, fv/2]);
ylim([-180, -40]);
title(['fv=', num2str(fv/1e9), 'GHz; rms jitter=', num2str(floor(jitter*1e15)), ' fs']);

%% ============================================================
%  Plot 4: ADPLL 输出频谱
% ============================================================
if 1
    Out_sine = sin(2*pi*tckv(1+10000:round(Sim_Time*fcw-2e4)+10000)/Tv);

    fprintf('Real sd:\n');
    [Px_sine2, f2] = fun_calc_psd_dbs(Out_sine, fv, rbw, fstep_psd);
    Px_sine2 = fftshift(Px_sine2);
    figure('Name', 'Spectrum');
    plot(f2+fv/2, Px_sine2 - 10*log10(2));
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Phase noise Spectrum (dBc/Hz)');
    xlim([fv-3e6, fv+3e6]);
    ylim([-180, 0]);

end

%% ============================================================
%  Plot 5: DTC 调试图（稳态段放大）
% ============================================================
if 1
    figure('Name', 'DTC 与 PHRF 逻辑关系对照');
    time_axis = (1:Sim_Time) * Tref * 1e6;

    ax1 = subplot(4,1,1);
    plot(time_axis, DCW_main_log,   'r', 'LineWidth', 1, 'DisplayName', 'Main DTC');
    hold on;
    plot(time_axis, DCW_offset_log, 'y', 'LineWidth', 1, 'DisplayName', 'Offset DTC');
    plot(time_axis, DCW_delta_log,  'g', 'LineWidth', 1, 'DisplayName', 'Delta');
    grid on; ylabel('DTC Code (LSB)');
    title('DTC 控制码序列');
    legend('Location', 'northeast');

    ax2 = subplot(4,1,2);
    plot(time_axis, PHRF_cur_log, 'g', 'LineWidth', 1);
    grid on; ylabel('Phase (Norm)');
    title('输入分数相位 PHRF\_cur');

    ax3 = subplot(4,1,3);
    plot(time_axis, phie, 'y', 'LineWidth', 1);
    grid on; ylabel('phie');
    title('TDC 相位误差 phie');

    ax4 = subplot(4,1,4);
    plot(time_axis, DTC_Delay_time_error_log, 'm', 'LineWidth', 1);
    grid on; xlabel('Time (\mu s)'); ylabel('Time (s)');
    title('DTC INL 误差（MAIN - OFFSET）');

    t_start = settle_start_fref * Tref * 1e6;
    t_end   = (settle_start_fref + 2000) * Tref * 1e6;
    linkaxes([ax1, ax2, ax3, ax4], 'x');
    xlim([t_start, t_end]);
end

%% ============================================================
%  Plot 6: Calib
% ============================================================
if 1
    figure('Name', 'Calib');
    time_axis = (1:Sim_Time) * Tref * 1e6;

    ax1 = subplot(2,1,1);
    plot(time_axis, koft_log,   'r', 'LineWidth', 1);
    hold on;
    grid on; 
    title('koft');
   

    ax2 = subplot(2,1,2);
    plot(time_axis, kdtc_log, 'g', 'LineWidth', 1);
    grid on; 
    title('kdtc');



    t_start = settle_start_fref * Tref * 1e6;
    t_end   = (settle_start_fref + 2000) * Tref * 1e6;
    linkaxes([ax1, ax2], 'x');
    xlim([t_start, t_end]);
end

%% ============================================================
%  终端输出汇总
% ============================================================
spur_f = FCW_F * fref;

fprintf('\n===== 仿真结果汇总 =====\n');
fprintf('===== Chopping pro V2 =====\n');
fprintf('CHOPPING_EN  = %d\n', CHOPPING_EN);
fprintf('DIFF_DTC_EN  = %d\n', DIFF_DTC_EN);
fprintf('DITHER_EN    = %d，DITHER_AMP = %d\n', DITHER_EN, DITHER_AMP);

inl_name = {'理想，无INL','纯偶函数(抛物线)', '偶+奇混合', '正弦形', '纯奇函数(线性斜坡)'};
fprintf('INL_TYPE     = %d  [%s] ,', INL_TYPE, inl_name{INL_TYPE});
fprintf('EVEN=%.0fps, ODD=%.0fps, Mismatch=%.0f%%\n', ...
    A_INL_EVEN*1e12, A_INL_ODD*1e12, INL_MISMATCH*100);
fprintf('FCW_F        = 1/%.0f,  预期 spur @ %.2f kHz\n', 1/FCW_F, spur_f/1e3);
fprintf('RMS Jitter   = %.1f fs  (积分 1kHz ~ fv/2)\n', jitter * 1e15);
fprintf('========================\n');

