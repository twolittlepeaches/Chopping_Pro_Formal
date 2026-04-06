%% ================================================================
%  DTC Chopping / Differential DTC / Dither PLL 行为级仿真  v3
%
%  v3 相较 v2 修改说明：
%    1. MAIN/OFFSET DTC 全部更名为 P/N DTC
%       - P DTC: 正向路径（原 MAIN）
%       - N DTC: 负向路径（原 OFFSET）
%    2. 新增 LMS2_EN: 二阶LMS预失真
%       - 针对差分后残余的奇函数INL（差分会使奇INL加倍）
%       - 数字域补偿：在TDC量化前从trise中减去估计的奇INL贡献
%       - LMS更新：用PHRF²加权，与增益校准（线性项）区分
%    3.DCT
% ================================================================
clc; clear; close all;

%% ============================================================
%  功能开关（独立，可任意组合）
% ============================================================
CHOPPING_EN       = 1;   % 随机chopping：P/N DTC随机交换ref/div路径
DIFF_DTC_EN       = 1;   % 差分DTC：P/N互补码各承担一半相位
DITHER_EN         = 1;   % Dither：P DTC加随机扰动 + TDC数字补偿
DITHER_AMP        = 256; % Dither幅度 ±LSB

LMS2_EN           = 0;   % 二阶LMS预失真（针对奇函数INL残余）

% ============================================================
%  【新增】INL 对称中心偏移控制
% ============================================================
INL_OFFSET_EN     = 0;   % 开关：引入真实的INL偶函数对称中心偏移
INL_OFFSET_VAL    = 0.15; % 偏移量（归一化单位，0.15代表偏移了15%的量程）

%% ============================================================
%  INL 类型选择
%  1 = 理想，无INL（验证基础功能）
%  2 = 纯偶函数（抛物线，VS-DTC主要非线性，Chopping最有效）
%  3 = 偶+奇混合（更接近实际VS-DTC，ODD<<EVEN）
%  4 = 正弦形INL（原始代码风格）
%  5 = 纯奇函数（三次方，验证LMS2和随机中点对奇INL的效果）
%      理论：差分后奇INL加倍；Chopping无效；LMS2/随机中点可抑制
% ============================================================
INL_TYPE         = 3;
A_INL_EVEN       = 10e-12; % 偶函数INL幅度（ps）
A_INL_ODD        = 2e-12;  % 奇函数INL幅度（ps）
INL_MISMATCH     = 0.2;   % 两路INL幅度失配比例（5%）

%% ============================================================
%  基本PLL参数
% ============================================================
rng('default'); rng(23);
Sim_Time     = 2^19;
settle_sample= 10e6;

fref = 100e6;   Tref = 1/fref;
f0   = 5.5e9;   T0   = 1/f0;

fcw   = 55 + 1/256;   % near-integer，spur @ fref/256 ≈ 390kHz
FCW_I = floor(fcw);
FCW_F = mod(fcw, 1);

fv    = fcw * fref;   Tv    = 1/fv;
fv_min= 4.0e9;        Tv_min= 1/fv_min;

PHRF_full = mod(cumsum(FCW_F * ones(1, Sim_Time)), 1);
PHRF_full(PHRF_full >= 1) = 0;
OV = [0, (PHRF_full(2:end) - PHRF_full(1:end-1)) < 0];

fprintf('FCW = %g, FCW_F = 1/%.0f\n', fcw, 1/FCW_F);
fprintf('预期spur: %.2f kHz\n', FCW_F*fref/1e3);

%% ============================================================
%  DTC / TDC 基本参数
% ============================================================
delta_TDC = 0.5e-15;
DTC_reso  = 4e-15;
KDCO      = 10e3;
NB_dco    = 5;
Mdcorun   = 2;
N_DTC     = round(Tv_min / DTC_reso);

%% ============================================================
%  噪声参数
% ============================================================
f_offset        = 1e6;
S_DCO_offset    = 10^(-126/10);
sigma_w         = f_offset/fv * sqrt(Tv) * sqrt(S_DCO_offset);
delta_Tw        = sigma_w * randn(round(Sim_Time*fcw*Mdcorun), 1);

S_DCO_thermal   = 10^(-160/10);
sigma_j         = 1/(2*pi*fv)*sqrt(S_DCO_thermal*fv);
delta_Tj        = sigma_j * randn(round(Sim_Time*fcw*Mdcorun), 1);

S_ref           = 10^(-170/10);
sigma_ref       = 1/(2*pi*fref)*sqrt(S_ref*fref);
delta_Tref      = sigma_ref * randn(Sim_Time, 1);

%% ============================================================
%  环路滤波器参数
% ============================================================
alpha  = 2^(-5);   rho = 2^(-12);
I_path = 0;
yiir1=0; yiir2=0; yiir3=0; yiir4=0;
lambda1=1; lambda2=1; lambda3=1; lambda4=1;

%% ============================================================
%  变量初始化
% ============================================================
otw          = zeros(Sim_Time+1, 1);
tR           = zeros(Sim_Time, 1);
trise        = zeros(Sim_Time, 1);
phie         = zeros(Sim_Time, 1);
Delta_TDEV   = zeros(Sim_Time, 1);
t_P = 0;  t_N = 0;

Rv           = zeros(round(Sim_Time*fcw*Mdcorun), 1);
TDEV         = zeros(size(Rv));
TDEVpn       = zeros(size(Rv));
tckv         = zeros(size(Rv));
tckv_div     = zeros(size(Rv));
tckv_period  = zeros(size(Rv));
tckv_norm    = zeros(size(Rv));

INL_est_P   = zeros(1, N_DTC);
INL_est_N = zeros(1, N_DTC);

% 调试记录
dither_actual         = zeros(Sim_Time, 1);
PHRF_cur_log          = zeros(Sim_Time, 1);
DCW_P_log             = zeros(Sim_Time, 1);
DCW_N_log             = zeros(Sim_Time, 1);
DCW_delta_log         = zeros(Sim_Time, 1);
INL_error_log         = zeros(Sim_Time, 1);
koft_log              = zeros(Sim_Time, 1);
kdtc_log              = zeros(Sim_Time, 1);
C_total_log           = zeros(Sim_Time, 1);
beta2_log             = zeros(Sim_Time, 1);
C_dc_log              = zeros(Sim_Time, 1);

DCW_P_ideal   = 0;  DCW_P_actual  = 0;
DCW_N_ideal   = 0;  DCW_N_actual  = 0;

%% ============================================================
%  Chopping序列
% ============================================================
if CHOPPING_EN
    EN_SWAP = randi([0,1], 1, Sim_Time);
else
    EN_SWAP = zeros(1, Sim_Time);
end

%% ============================================================
%  Dither序列（仅加在P路）
% ============================================================
if DITHER_EN
    dither_seq = randi([-DITHER_AMP, DITHER_AMP], 1, Sim_Time);
else
    dither_seq = zeros(1, Sim_Time);
end

%% ============================================================
%  校准初始值
% ============================================================
gain_DTC_est = 0.9 * Tv / DTC_reso;
Koft         = 300e-12;
step_DCO     = 0;

% 二阶LMS预失真系数
% 物理含义：补偿差分后残余的奇函数INL，单位：Tv（归一化相位）
mu_lut = 1e-14;  % 步长，根据收敛速度调整



%% ============================================================
%  INL建模
%  u ∈ [-1, 1]，以码字中点为原点归一化
% ============================================================
code_vec = (0 : N_DTC-1);
u = (code_vec - N_DTC/2) / (N_DTC/2);

% 【核心修改】：引入对称中心偏移
% 如果存在偏移，偶函数的抛物线顶点不再是 u=0，而是 u=INL_OFFSET_VAL
if INL_OFFSET_EN
    u_even = u - INL_OFFSET_VAL; 
else
    u_even = u;
end

INL_EVEN = A_INL_EVEN * u_even.^2;
INL_ODD  = A_INL_ODD * u.^3;

switch INL_TYPE
    case 1
        INL_P_raw = zeros(size(u));
        INL_N_raw = zeros(size(u));
    case 2  % 纯偶函数
        INL_P_raw = A_INL_EVEN * u_even.^2;
        INL_N_raw = A_INL_EVEN * (1 + INL_MISMATCH) * u_even.^2;
    case 3  % 偶+奇混合
        INL_P_raw = A_INL_EVEN * u_even.^2 + A_INL_ODD * u.^3;
        INL_N_raw = A_INL_EVEN*(1+INL_MISMATCH)*u_even.^2 + A_INL_ODD*(1-INL_MISMATCH)*u.^3;
    case 4  % 正弦形 (也加上相位偏移)
        INL_P_raw = A_INL_EVEN * sin(3*pi*(code_vec - INL_OFFSET_VAL*N_DTC/2)/N_DTC);
        INL_N_raw = A_INL_EVEN*(1+INL_MISMATCH) * sin(3*pi*(code_vec - INL_OFFSET_VAL*N_DTC/2)/N_DTC);
    case 5  % 纯奇函数（三次方）
        INL_P_raw = A_INL_ODD * u.^3;
        INL_N_raw = A_INL_ODD * (1 + INL_MISMATCH) * u.^3;
    otherwise
        error('INL_TYPE 必须为 1~5');
end

INL_P = detrend(INL_P_raw, 1);
INL_N = detrend(INL_N_raw, 1);

% 【同步修改 log 计算，防止画图出错】
INL_P_odd_log = INL_P - A_INL_EVEN * u_even.^2;
INL_N_odd_log = INL_N - A_INL_EVEN * (1 + INL_MISMATCH) * u_even.^2;

get_INL_P = @(c) INL_P(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));
get_INL_N = @(c) INL_N(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));

%% ============================================================
%  主仿真循环 
% ============================================================
fprintf('开始仿真 2^%.0f FREF cycles...\n', log2(Sim_Time));


for step = 1:Sim_Time

    %% 1. FREF时间戳
    tR(step)      = step * Tref + delta_Tref(step);

    %% 2. DCO计数
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

    %% 3. 分数相位
    PHRF_cur = PHRF_full(step) - 0.5;  % 中心化到[-0.5, 0.5)
    PHRF_cur_log(step) = PHRF_cur;

    %% 4. DTC理想码字计算
    %
    %  差分模式（DIFF=1）：
    %    P/N互补，各承担一半相位。SWAP决定路径，码字随之反向。
    %    SWAP=0: P接ref，code=-x/2；N接div，code=+x/2
    %    SWAP=1: P接div，code=+x/2；N接ref，code=-x/2
    %
    %  单路模式（DIFF=0）：
    %    P承担全部相位，N提供固定偏置Koft
    %    SWAP=0: P接ref，code=-PHRF
    %    SWAP=1: P接div，code=+PHRF

    % if DIFF_DTC_EN % Old Move Type
    %     if EN_SWAP(step) > 0.5   % SWAP=1: P在div侧
    %         DCW_P_ideal = +((PHRF_cur + 0.5) / 2) * gain_DTC_est;
    %         DCW_N_ideal = -((PHRF_cur + 0.5) / 2) * gain_DTC_est;
    %     else                      % SWAP=0: P在ref侧
    %         DCW_P_ideal = -((PHRF_cur + 0.5) / 2) * gain_DTC_est;
    %         DCW_N_ideal = +((PHRF_cur + 0.5) / 2) * gain_DTC_est;
    %     end
    % else
    %     if EN_SWAP(step) > 0.5
    %         DCW_P_ideal = +PHRF_cur * gain_DTC_est;
    %     else
    %         DCW_P_ideal = -PHRF_cur * gain_DTC_est;
    %     end
    %     DCW_N_ideal = 0;
    % end


    if DIFF_DTC_EN % New Move Type
        % 此时码字是绕 0 对称的
        DCW_P_ideal   = -(PHRF_cur / 2) * gain_DTC_est; 
        DCW_N_ideal = +(PHRF_cur / 2) * gain_DTC_est;

        if EN_SWAP(step) > 0.5
            % 交换路径
            tmp = DCW_P_ideal;
            DCW_P_ideal = DCW_N_ideal;
            DCW_N_ideal = tmp;
        end
    else                        % Only Chop = 1
        if EN_SWAP(step) > 0.5
            DCW_P_ideal = +PHRF_cur * gain_DTC_est;
        else
            DCW_P_ideal = -PHRF_cur * gain_DTC_est;
        end
        DCW_N_ideal = 0;
    end



    %% 6. 实际码字 + Dither（仅P路）
    DCW_P_actual = DCW_P_ideal + dither_seq(step);
    DCW_P_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_P_actual));

    DCW_N_actual = DCW_N_ideal ;
    DCW_N_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_N_actual));

    dither_actual(step) = DCW_P_actual - DCW_P_ideal;

    %% 7. DTC实际延时
    t_P = DCW_P_actual * DTC_reso + 250e-12 + get_INL_P(DCW_P_actual);
    t_N = DCW_N_actual * DTC_reso + 200e-12 + Koft + get_INL_N(DCW_N_actual);

    DCW_P_log(step) = DCW_P_actual;
    DCW_N_log(step) = DCW_N_actual;

    %% 8. TDC测量（trise）
    %  SWAP=0: P在ref侧 → trise = (tckv + t_N) - (tR + t_P)
    %  SWAP=1: P在div侧 → trise = (tckv + t_P) - (tR + t_N)

    if EN_SWAP(step) > 0.5   % P在div侧
        trise(step) = (tckv(step_DCO) + t_P) - (tR(step) + t_N);
        DCW_delta_log(step) = (t_P - t_N) / DTC_reso;
       
    else                      % P在ref侧
        trise(step) = (tckv(step_DCO) + t_N) - (tR(step) + t_P);
        DCW_delta_log(step) = (t_N - t_P) / DTC_reso;

    end

    %% 9. Dither数字补偿（P路dither的理想延时还原）
    if DITHER_EN
        dither_delay = dither_actual(step) * DTC_reso;
        if EN_SWAP(step) > 0.5
            trise(step) = trise(step) - dither_delay;
        else
            trise(step) = trise(step) + dither_delay;
        end
    end

%% 10. 二阶LMS预失真数字补偿
if LMS2_EN


     idx_P = max(1, min(N_DTC, round(DCW_P_actual)   + round(N_DTC/2) + 1));
     idx_N = max(1, min(N_DTC, round(DCW_N_actual)   + round(N_DTC/2) + 1));
    % % 主循环内，TDC量化之前：


    if EN_SWAP(step) > 0.5
            % P 在 div 侧（正），N 在 ref 侧（负）
            comp_val = INL_est_P(idx_P) - INL_est_N(idx_N);
            INL_error_log(step) = get_INL_P(DCW_P_actual) - get_INL_N(DCW_N_actual) - (INL_est_P(idx_P) - INL_est_N(idx_N));
    else
            % N 在 div 侧（正），P 在 ref 侧（负）
            comp_val = INL_est_N(idx_N) - INL_est_P(idx_P);
            INL_error_log(step) = get_INL_N(DCW_N_actual) - get_INL_P(DCW_P_actual)- (INL_est_N(idx_N) - INL_est_P(idx_P));
    end
        % 从 trise 中减去查表得到的估计误差
    trise(step) = trise(step) - comp_val;

end



    %% 11. TDC量化
    phie(step) = (floor(trise(step)/delta_TDC) - 0.5) * delta_TDC / Tv;

    %% 12. 校准环路（Sign-LMS）
    %
    %  [Koft校准]
    %  目标：消除P/N路固定延时差（250ps vs 200ps）对SWAP切换的影响
    %  与INL形状无关，校准的是纯DC偏置
    if CHOPPING_EN
        Koft = Koft + 2e-14 * sign(EN_SWAP(step) - 0.5) * sign(phie(step));
    end

    %  [增益校准]
    %  目标：让phie与PHRF_cur线性分量去相关
    %  步长最大（1e-1），带宽最高，最先收敛
    gain_DTC_est = gain_DTC_est + (-1) * 1e-1 * sign(phie(step)) * sign(PHRF_cur);

    %  [二阶LMS预失真校准]

    if LMS2_EN

        if EN_SWAP(step) > 0.5
            % phie>0 说明 trise 偏大，意味着 t_P 偏大或 t_N 偏小
            % 因此 P 表应该增加（下次多减点），N 表应该减小
            INL_est_P(idx_P) = INL_est_P(idx_P) + mu_lut * sign(phie(step));
            INL_est_N(idx_N) = INL_est_N(idx_N) - mu_lut * sign(phie(step));
        else
            % phie>0 说明 trise 偏大，意味着 t_N 偏大或 t_P 偏小
            % 因此 P 表应该减小，N 表应该增加
            INL_est_P(idx_P) = INL_est_P(idx_P) - mu_lut * sign(phie(step));
            INL_est_N(idx_N) = INL_est_N(idx_N) + mu_lut * sign(phie(step));
        end

    end

    koft_log(step) = Koft;
    kdtc_log(step) = gain_DTC_est;

    %% 13. 环路滤波器
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
%  稳态区间
% ============================================================
settle_start_fref = round(Sim_Time * 0.4);
settle_start_dco  = round(step_DCO * 0.4);

%% ============================================================
%  Plot 1: CKV 瞬时频率偏差
% ============================================================
if 1
    fckv_div = 1./tckv_period(1:settle_sample) - fv;
    fckv_div_avg = zeros(size(fckv_div));
    N_avg = 25;
    for num = 1:settle_sample
        if num < N_avg
            fckv_div_avg(num) = mean(fckv_div(1:num+N_avg));
        elseif num > (settle_sample-N_avg)
            fckv_div_avg(num) = mean(fckv_div(num-N_avg+1:settle_sample));
        else
            fckv_div_avg(num) = mean(fckv_div(num-N_avg+1:num+N_avg));
        end
    end
    figure('Name', 'Freq Deviation');
    plot(1e6*tckv(1:settle_sample), fckv_div/1e6); hold on;
    plot(1e6*tckv(1:settle_sample), fckv_div_avg/1e6, 'r-');
    grid on;
    xlabel('Time (us)', 'fontsize', 16);
    ylabel('Freq Deviation (MHz)', 'fontsize', 16);
    set(gca, 'YTick', -100:5:100, 'fontsize', 16);
    set(findobj(gca,'Type','line'), 'LineWidth', 1);
end

%% ============================================================
%  Plot 2: 瞬时相位误差
% ============================================================
if 1
    figure('Name', 'Phase Error phie');
    plot((1:Sim_Time)*Tref*1e6, phie, 'b', 'LineWidth', 0.5);
    grid on;
    xlabel('Time (us)', 'fontsize', 16);
    ylabel('Phase Deviation \phi_e', 'fontsize', 16);
    set(gca, 'fontsize', 16);
end

%% ============================================================
%  Plot 3: CKV 周期偏差
% ============================================================
if 1
    figure('Name', 'Period Deviation');
    period_div = tckv_div(1:step_DCO) - mean(tckv_div(settle_sample:step_DCO));
    plot(tckv_norm(1:step_DCO)*1e6, period_div*1e12);
    grid on;
    xlabel('Time (us)', 'fontsize', 20);
    ylabel('Period Deviation (ps)', 'fontsize', 20);
    set(gca, 'YTick', -2500:500:2500, 'fontsize', 16);
    set(findobj(gca,'Type','line'), 'LineWidth', 1);
end

%% ============================================================
%  Plot 4: 相位噪声（S域模型 + 时域仿真叠加）
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
%  Plot 5: ADPLL输出频谱（spur可视化）
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
%  Plot 6: DTC调试图（P/N码字 + PHRF + phie + INL误差）
% ============================================================
if 1
    figure('Name', 'DTC Debug');
    time_axis = (1:Sim_Time) * Tref * 1e6;

    ax1 = subplot(4,1,1);
    plot(time_axis, DCW_P_log, 'r', 'LineWidth', 1, 'DisplayName', 'P DTC');
    hold on;
    plot(time_axis, DCW_N_log, 'y', 'LineWidth', 1, 'DisplayName', 'N DTC');
    plot(time_axis, DCW_delta_log, 'g', 'LineWidth', 1, 'DisplayName', '\Delta');
    grid on; ylabel('Code (LSB)'); title('P/N DTC 控制码');
    legend('Location', 'northeast');

    ax2 = subplot(4,1,2);
    plot(time_axis, PHRF_cur_log, 'g', 'LineWidth', 1);
    grid on; ylabel('Phase (Norm)'); title('分数相位 PHRF\_cur');

    ax3 = subplot(4,1,3);
    plot(time_axis, phie, 'y', 'LineWidth', 1);
    grid on; ylabel('phie'); title('TDC 相位误差');

    ax4 = subplot(4,1,4);
    plot(time_axis, INL_error_log*1e12, 'm', 'LineWidth', 1);
    grid on; xlabel('Time (\mu s)'); ylabel('(ps)');
    title('INL误差（P路 - N路）');

    t_start = settle_start_fref * Tref * 1e6;
    t_end   = (settle_start_fref + 2000) * Tref * 1e6;
    linkaxes([ax1, ax2, ax3, ax4], 'x');
    xlim([t_start, t_end]);
end

%% ============================================================
%  Plot 7: 校准收敛曲线
% ============================================================
if 1
    figure('Name', 'Calibration Convergence');
    time_axis = (1:Sim_Time) * Tref * 1e6;

    n_sub = 2;
    sp_idx = 0;

    sp_idx = sp_idx + 1;
    subplot(n_sub, 1, sp_idx);
    plot(time_axis, koft_log*1e12, 'r', 'LineWidth', 1);
    grid on; ylabel('Koft (ps)'); title('Koft校准（固定路径偏置）');

    sp_idx = sp_idx + 1;
    subplot(n_sub, 1, sp_idx);
    plot(time_axis, kdtc_log, 'y', 'LineWidth', 1);
    grid on; ylabel('gain'); title('DTC增益校准');



    t_start = settle_start_fref * Tref * 1e6;
    t_end   = (settle_start_fref + 2000) * Tref * 1e6;
    ax_list = findobj(gcf, 'Type', 'axes');
    linkaxes(ax_list, 'x');
    xlim([t_start, t_end]);
end


%% ============================================================
%  Plot 10: LMS2 Calibration 效果 (真实 vs 估计)
% ============================================================
if 1
    figure('Name', sprintf('LMS2 Calib Effect (EN=%d)', LMS2_EN));
    

    % 子图1：P 路对比
    subplot(3,1,1);
    plot(code_vec - N_DTC/2, INL_P_odd_log* Tv * 1e12, 'r--', 'LineWidth', 1, 'DisplayName', 'Real INL_P');
    hold on;
    % 注意：TDC量化处是 phie(step) = ... / Tv，所以估计值要乘回 Tv 换算成时间
    plot(code_vec - N_DTC/2, INL_est_P * Tv * 1e12, 'g', 'LineWidth', 1.5, 'DisplayName', 'LMS2 Est P');
    grid on;
    ylabel('Time (ps)');
    title('P-DTC: 真实 INL vs LMS2 估计值');
    legend;

    % 子图2：N 路对比
    subplot(3,1,2);
    plot(code_vec - N_DTC/2, INL_N_odd_log* Tv * 1e12, 'r--', 'LineWidth', 1, 'DisplayName', 'Real INL_N');
    hold on;
    plot(code_vec - N_DTC/2, INL_est_N * Tv * 1e12, 'g', 'LineWidth', 1.5, 'DisplayName', 'LMS2 Est N');
    grid on;
    xlabel('DTC Control Code');
    ylabel('Time (ps)');
    title('N-DTC: 真实 INL vs LMS2 估计值');
    legend;


    % 子图3 DTC INL Mis Match
    subplot(3,1,3);
    plot(code_vec - N_DTC/2, INL_N*1e12, 'y--', 'LineWidth', 1, 'DisplayName', 'Real INL_N');
    hold on;
    plot(code_vec - N_DTC/2, INL_P*1e12, 'g--', 'LineWidth', 1, 'DisplayName', 'Real INL_p');
    grid on;
    xlabel('DTC Control Code');
    ylabel('Time (ps)');
    title('DTC INL Mis Match');
    legend;
end

%% ============================================================
%  Plot 11: 归一化 CKV 输出频谱 (标准 FFT 观察 Spur)
% ============================================================
%% Spectrum for the ADPLL output (CKV)
if 1
    % 用相位时间戳构造等效基带复信号，再做FFT
    % settle之后取整数段
    idx_start = settle_sample + 1;
    idx_end   = step_DCO - 1;
    
    % CKV时间戳（绝对时间），去掉理想线性部分得到相位偏差
    t_use  = tckv(idx_start:idx_end);
    N_use  = length(t_use);
    
    % 瞬时相位偏差（rad）
    phi_ckv = 2*pi*(t_use - (idx_start:idx_end)' * Tv) / Tv;
    phi_ckv = phi_ckv - mean(phi_ckv);
    
    % 构造等效复基带信号
    ckv_bb = exp(1j * phi_ckv);
    
    % FFT
    NFFT   = 2^nextpow2(N_use);
    CKV_F  = fftshift(fft(ckv_bb, NFFT)) / NFFT;
    f_bb   = (-NFFT/2 : NFFT/2-1) * (fv / NFFT);  % 基带频率轴，中心=0
    f_ckv  = f_bb + fv;                             % 搬移到载波频率
    
    % 转换为dBc（相对于载波功率）
    psd_ckv = 20*log10(abs(CKV_F) + 1e-30);
    carrier_bin = NFFT/2 + 1;  % DC bin（对应载波）
    psd_dbc = psd_ckv - psd_ckv(carrier_bin);
    
    figure('Name', 'FFT');
    plot(f_ckv/1e9, psd_dbc, 'g', 'LineWidth', 1);
    grid on;
    xlabel('Frequency (GHz)');
    ylabel('Power (dBc)');
    title(['CKV Spectrum  fv = ', num2str(fv/1e9), ' GHz'], 'fontsize', 14);
    xlim([(fv-0.8e6)/1e9, (fv+0.8e6)/1e9]);
    ylim([-90, 5]);

end

%% ============================================================
%  终端输出汇总
% ============================================================
spur_f = FCW_F * fref;
inl_names = {'理想无INL', '纯偶函数', '偶+奇混合', '正弦形', '纯奇函数(三次方)'};

fprintf('\n===== 仿真结果汇总  v3 =====\n');

fprintf('CHOPPING_EN  = %d\n', CHOPPING_EN);
fprintf('DIFF_DTC_EN  = %d\n', DIFF_DTC_EN);
fprintf('DITHER_EN    = %d，DITHER_AMP = %d\n', DITHER_EN, DITHER_AMP);

fprintf('LMS2         =%d (mu   =%.0e)\n', LMS2_EN, mu_lut);

fprintf('INL: Type%d [%s]  EVEN=%.0fps  ODD=%.0fps  Mismatch=%.0f%%\n', ...
    INL_TYPE, inl_names{INL_TYPE}, A_INL_EVEN*1e12, A_INL_ODD*1e12, INL_MISMATCH*100);
fprintf('FCW_F = 1/%.0f  →  spur @ %.2f kHz\n', 1/FCW_F, spur_f/1e3);
fprintf('RMS Jitter = %.1f fs\n', jitter*1e15);

fprintf('===========================\n');
