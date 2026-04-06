%%  DTC Chopping / Differential DTC / Dither PLL 行为级仿真  v6
%
%  v6 修改说明（相对 v5）：
%    1.删除 NCC_EN 开关：
% ================================================================

clc; clear; close all;

% ============================================================
%%  功能开关
% ============================================================
CHOPPING_EN   = 1;
DIFF_DTC_EN   = 1;
DITHER_EN     = 0;
DITHER_AMP    = 2048;

MASH_ORDER    = 1;

LMS_POLY_EN   = 1;

INL_OFFSET_EN  = 0;
INL_OFFSET_VAL = 0.15;

% ============================================================
%%  DTC INL 参数
% ============================================================
INL_TYPE     = 3;
% A_INL_EVEN   = 9.2e-12;
% A_INL_ODD    = 0.8e-12;
A_INL_EVEN   = 10e-12;
A_INL_ODD    = 2e-12;
INL_MISMATCH = 0.05;


% ============================================================
%%  基本 PLL 参数
% ============================================================
rng('default'); rng(23);
Sim_Time      = 2^19;
settle_sample = 10e6;

fref  = 100e6;   Tref = 1/fref;
f0    = 5.5e9;   T0   = 1/f0;
fcw   = 55 + 1/256;
FCW_I = floor(fcw);
FCW_F = mod(fcw, 1);
fv    = fcw * fref;   Tv = 1/fv;
fv_min = 4.0e9;       Tv_min = 1/fv_min;

fprintf('FCW = %g, 预期spur: %.2f kHz\n', fcw, FCW_F*fref/1e3);

% ============================================================
%%  DSM 序列生成
% ============================================================
if MASH_ORDER == 2
    acc1 = zeros(1,Sim_Time); acc2 = zeros(1,Sim_Time);
    ov1  = zeros(1,Sim_Time); ov2  = zeros(1,Sim_Time);
    for i = 2:Sim_Time
        acc1_val = acc1(i-1) + FCW_F;
        ov1(i) = floor(acc1_val); acc1(i) = acc1_val - ov1(i);
        acc2_val = acc2(i-1) + acc1(i);
        ov2(i) = floor(acc2_val); acc2(i) = acc2_val - ov2(i);
    end
    OV = ov1 + [0, diff(ov2)];
    PHRF_cur_array = acc1 - ov2;
else %一阶MASH
    PHRF_full = mod(cumsum(FCW_F * ones(1,Sim_Time)), 1);
    OV = [0, (PHRF_full(2:end) - PHRF_full(1:end-1)) < 0];
    PHRF_cur_array = PHRF_full - 0.5;
end

% ============================================================
%%  DTC / TDC 基本参数
% ============================================================
delta_TDC = 0.5e-15;
TDC_RANGE = 150e-12;
DTC_reso  = 4e-15;
KDCO      = 10e3;
NB_dco    = 5;
Mdcorun   = 2;
N_DTC     = round(Tv_min / DTC_reso);

% ============================================================
%%  噪声参数
% ============================================================
f_offset     = 1e6;
S_DCO_offset = 10^(-126/10);
sigma_w      = f_offset/fv * sqrt(Tv) * sqrt(S_DCO_offset);
delta_Tw     = sigma_w * randn(round(Sim_Time*fcw*Mdcorun), 1);

S_DCO_thermal = 10^(-160/10);
sigma_j       = 1/(2*pi*fv)*sqrt(S_DCO_thermal*fv);
delta_Tj      = sigma_j * randn(round(Sim_Time*fcw*Mdcorun), 1);

S_ref      = 10^(-170/10);
sigma_ref  = 1/(2*pi*fref)*sqrt(S_ref*fref);
delta_Tref = sigma_ref * randn(Sim_Time, 1);

% ============================================================
%%  环路滤波器参数
% ============================================================
alpha = 2^(-5);   rho = 2^(-12);
I_path = 0;
yiir1=0; yiir2=0; yiir3=0; yiir4=0;
lambda1=1; lambda2=1; lambda3=1; lambda4=1;

% ============================================================
%%  NCC 参数（仅 NCC_EN=1 时生效）
%
%  NCC_M         每次二分决策累积的样本数（抗噪，M越大越稳但越慢）
%  NCC_BITS_*    各路二分位数（精度 = step_init / 2^bits）
%  NCC_WARMUP    热身周期，PLL 粗锁后再开始校准
%
%  时间预算（NCC_EN=1）：
%    总计 = WARMUP + KOFT_bits*M + KDTC_bits*M + POLY_bits*M
%    对比 v4 sign-LMS：通常 >50000 周期，FCW_F小时更慢
% ============================================================
NCC_M         = 64;
NCC_BITS_KOFT = 14;
NCC_BITS_KDTC = 14;
NCC_BITS_POLY = 14;
NCC_WARMUP    = 2000;

T_end_warmup = NCC_WARMUP;
T_end_koft   = T_end_warmup + NCC_BITS_KOFT * NCC_M;
T_end_kdtc   = T_end_koft   + NCC_BITS_KDTC * NCC_M;
T_end_poly   = T_end_kdtc   + LMS_POLY_EN * NCC_BITS_POLY * NCC_M;





% ============================================================
%%  变量初始化
% ============================================================
otw          = zeros(Sim_Time+1, 1);
tR           = zeros(Sim_Time, 1);
trise        = zeros(Sim_Time, 1);
trise_quant  = zeros(Sim_Time, 1);
phie         = zeros(Sim_Time, 1);
Delta_TDEV   = zeros(Sim_Time, 1);
t_P = 0;  t_N = 0;

Rv          = zeros(round(Sim_Time*fcw*Mdcorun), 1);
TDEV        = zeros(size(Rv));
TDEVpn      = zeros(size(Rv));
tckv        = zeros(size(Rv));
tckv_div    = zeros(size(Rv));
tckv_period = zeros(size(Rv));

PHRF_cur_log   = zeros(Sim_Time, 1);
DCW_P_log      = zeros(Sim_Time, 1);
DCW_N_log      = zeros(Sim_Time, 1);
DCW_delta_log  = zeros(Sim_Time, 1);
Phie_error_log = zeros(Sim_Time, 1);
koft_log       = zeros(Sim_Time, 1);
kdtc_log       = zeros(Sim_Time, 1);
alpha_odd_log  = zeros(Sim_Time, 1);

% ============================================================
%%  校准初始值
% ============================================================
gain_DTC_est  = 0.9 * Tv / DTC_reso;
Koft          = 300e-12;
alpha_odd     = 0;
step_DCO      = 0;

% 步长（NCC_EN=0 时使用）

mu_koft = 2e-13;
mu_kdtc = 1e-1;
mu_poly = 2e-16;

% 跟踪阶段步长（极小，仅补偿温漂）
MU_TRACK_KOFT = 2e-13;
MU_TRACK_KDTC = 4e-1;
MU_TRACK_POLY = 2e-15;

% NCC 初始步长（覆盖最大预期误差的一半即可）
ncc_step_koft = 300e-12;               % 初始偏差约 250ps，300ps 覆盖有余
ncc_step_kdtc = 0.05 * gain_DTC_est;   % 初始偏差约 10%，20% 覆盖有余
ncc_step_poly = 2e-12;                % 保守估计

% NCC 内部状态
ncc_acc = 0;
ncc_cnt = 0;

% ============================================================
%%  Chopping / Dither 序列
% ============================================================
if CHOPPING_EN
    EN_SWAP = randi([0,1], 1, Sim_Time);
else
    EN_SWAP = zeros(1, Sim_Time);
end

if DITHER_EN
    dither_seq = randi([-DITHER_AMP, DITHER_AMP], 1, Sim_Time);
else
    dither_seq = zeros(1, Sim_Time);
end

% ============================================================
%%  INL 建模
% ============================================================
code_vec = (0 : N_DTC-1);
u = (code_vec - N_DTC/2) / (N_DTC/2);
if INL_OFFSET_EN
    u_even = u - INL_OFFSET_VAL;
else
    u_even = u;
end

switch INL_TYPE
    case 1
        INL_P_raw = zeros(size(u)); INL_N_raw = zeros(size(u));
    case 2
        INL_P_raw = A_INL_EVEN * u_even.^2;
        INL_N_raw = A_INL_EVEN * (1+INL_MISMATCH) * u_even.^2;
    case 3
        INL_P_raw = A_INL_EVEN * u_even.^2 + A_INL_ODD * u.^3;
        INL_N_raw = A_INL_EVEN*(1+INL_MISMATCH)*u_even.^2 + A_INL_ODD*(1-INL_MISMATCH)*u.^3;
    case 4
        INL_P_raw = A_INL_EVEN * sin(3*pi*(code_vec - INL_OFFSET_VAL*N_DTC/2)/N_DTC);
        INL_N_raw = A_INL_EVEN*(1+INL_MISMATCH)*sin(3*pi*(code_vec - INL_OFFSET_VAL*N_DTC/2)/N_DTC);
    case 5
        INL_P_raw = A_INL_ODD * u.^3;
        INL_N_raw = A_INL_ODD * (1+INL_MISMATCH) * u.^3;
    otherwise; error('INL_TYPE 必须为 1~5');
end

INL_P = detrend(INL_P_raw, 1);
INL_N = detrend(INL_N_raw, 1);
INL_P_odd_log = INL_P - A_INL_EVEN * u_even.^2;
INL_N_odd_log = INL_N - A_INL_EVEN * (1+INL_MISMATCH) * u_even.^2;

get_INL_P = @(c) INL_P(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));
get_INL_N = @(c) INL_N(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));

% ============================================================
%% 主仿真循环
% ============================================================
fprintf('开始仿真 2^%.0f FREF cycles...\n', log2(Sim_Time));

for step = 1:Sim_Time

    %% 1. FREF 时间戳
    tR(step) = step * Tref + delta_Tref(step);
    if step == 1
        Delta_TDEV(1) = T0 - 1/(f0 + otw(1)*KDCO);
    else
        Delta_TDEV(step) = T0 - 1/(f0 + otw(step)*KDCO);
    end

    %% 2. DCO 计数
    div_cnt = 0;
    while div_cnt < (FCW_I + OV(step))
        step_DCO = step_DCO + 1;
        if step_DCO == 1
            TDEV(1)        = Delta_TDEV(1);
            TDEVpn(1)      = delta_Tj(1) + delta_Tw(1);
            tckv(1)        = T0 - TDEV(1) + TDEVpn(1);
            tckv_div(1)    = tckv(1) - step_DCO*Tv;
            tckv_period(1) = tckv(1);
            Rv(1)          = 1;
        else
            TDEV(step_DCO)   = TDEV(step_DCO-1) + Delta_TDEV(step);
            TDEVpn(step_DCO) = TDEVpn(step_DCO-1) ...
                + delta_Tj(step_DCO) - delta_Tj(step_DCO-1) + delta_Tw(step_DCO);
            tckv(step_DCO)        = step_DCO*T0 - TDEV(step_DCO) + TDEVpn(step_DCO);
            tckv_div(step_DCO)    = tckv(step_DCO) - step_DCO*Tv;
            tckv_period(step_DCO) = tckv(step_DCO) - tckv(step_DCO-1);
            Rv(step_DCO)          = Rv(step_DCO-1) + 1;
        end
        div_cnt = div_cnt + 1;
    end

    %% 3. 分数相位
    PHRF_cur = PHRF_cur_array(step);
    PHRF_cur_log(step) = PHRF_cur;

    %% 4. DTC 理想码字
    if DIFF_DTC_EN
        DCW_P_ideal = -(PHRF_cur / 2) * gain_DTC_est;
        DCW_N_ideal = +(PHRF_cur / 2) * gain_DTC_est;
        if EN_SWAP(step) > 0.5
            tmp = DCW_P_ideal; DCW_P_ideal = DCW_N_ideal; DCW_N_ideal = tmp;
        end
    else
        if EN_SWAP(step) > 0.5
            DCW_P_ideal = +PHRF_cur * gain_DTC_est;
        else
            DCW_P_ideal = -PHRF_cur * gain_DTC_est;
        end
        DCW_N_ideal = 0;
    end

    %% 5. 实际码字 + Dither
    DCW_P_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_P_ideal + dither_seq(step)));
    DCW_N_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_N_ideal));
    dither_actual = DCW_P_actual - DCW_P_ideal;
    DCW_P_log(step) = DCW_P_actual;
    DCW_N_log(step) = DCW_N_actual;

    %% 6. DTC 实际延时
    t_P = DCW_P_actual * DTC_reso + 250e-12 + get_INL_P(DCW_P_actual);
    t_N = DCW_N_actual * DTC_reso + 200e-12 + Koft + get_INL_N(DCW_N_actual);
    % t_P = DCW_P_actual * DTC_reso +  get_INL_P(DCW_P_actual);
    % t_N = DCW_N_actual * DTC_reso +  get_INL_N(DCW_N_actual);
    %% 7. TDC 测量
    if EN_SWAP(step) > 0.5
        trise(step) = (tckv(step_DCO) + t_P) - (tR(step) + t_N);
        DCW_delta_log(step) = (t_P - t_N) / DTC_reso;
    else
        trise(step) = (tckv(step_DCO) + t_N) - (tR(step) + t_P);
        DCW_delta_log(step) = (t_N - t_P) / DTC_reso;
    end

    %% 8. TDC 量化 

    trise_quant(step) = trise(step);

    trise_quant(step) = max(-TDC_RANGE, min(TDC_RANGE, trise_quant(step)));

    phie_raw   = (floor(trise_quant(step)/delta_TDC) - 0.5) * delta_TDC / Tv;
    phie_noinl = (floor(trise(step)/delta_TDC) - 0.5) * delta_TDC / Tv;

    %% 9. 勒让德多项式补偿
    poly_val = 0;
    if LMS_POLY_EN
        if MASH_ORDER == 1
            max_diff_code = 0.5 * gain_DTC_est;
        else
            max_diff_code = 1.0 * gain_DTC_est;
        end
        u_diff = (DCW_P_actual - DCW_N_actual) / (max_diff_code + eps);
        u_diff = max(-1, min(1, u_diff));
        if EN_SWAP(step) > 0.5; u_diff = -u_diff; end
        poly_val = 0.5 * (5 * u_diff^3 - 3 * u_diff);
        phie_raw = phie_raw - (floor(alpha_odd * poly_val / delta_TDC) - 0.5) * delta_TDC / Tv;
    end

    %% 10. Dither 数字补偿
    if DITHER_EN
        dither_phi = dither_actual / gain_DTC_est;
        if EN_SWAP(step) > 0.5
            phie(step) = phie_raw - dither_phi;
        else
            phie(step) = phie_raw + dither_phi;
        end
    else
        phie(step) = phie_raw;
    end

    Phie_error_log(step) = phie(step) - phie_noinl;

    % ============================================================
    %% 11. 校准环路更新
    % ============================================================

    if CHOPPING_EN
        Koft = Koft + mu_koft * sign(EN_SWAP(step) - 0.5) * sign(phie(step));
    end

    gain_DTC_est = gain_DTC_est + (-1) * mu_kdtc * sign(phie(step)) * sign(PHRF_cur);

    if LMS_POLY_EN
        alpha_odd = alpha_odd + mu_poly * sign(phie(step)) * poly_val;
    end



    %% 日志
    koft_log(step)      = Koft;
    kdtc_log(step)      = gain_DTC_est;
    alpha_odd_log(step) = alpha_odd;

    %% 12. 环路滤波器
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

% ============================================================
%%  稳态起始点
% ============================================================

settle_start_fref = round(Sim_Time * 0.6);


% ============================================================
%%  Plot 1: CKV 瞬时频率偏差
% ============================================================
if 1
    fckv_div = 1./tckv_period(1:settle_sample) - fv;
    fckv_div_avg = zeros(size(fckv_div));
    N_avg = 25;
    for num = 1:settle_sample
        lo = max(1, num-N_avg+1); hi = min(settle_sample, num+N_avg);
        fckv_div_avg(num) = mean(fckv_div(lo:hi));
    end
    figure('Name','Freq Deviation');
    plot(1e6*tckv(1:settle_sample), fckv_div/1e6); hold on;
    plot(1e6*tckv(1:settle_sample), fckv_div_avg/1e6, 'r-');
    grid on;
    xlabel('Time (us)', 'fontsize', 16);
    ylabel('Freq Deviation (MHz)', 'fontsize', 16);
    set(gca, 'YTick', -100:5:100, 'fontsize', 16);
    set(findobj(gca,'Type','line'), 'LineWidth', 1);
end

% ============================================================
%%  Plot 2: 瞬时相位误差
% ============================================================
if 1
    figure('Name','Phase Error phie');
    plot((1:Sim_Time)*Tref*1e6, phie, 'b', 'LineWidth', 0.5);
    grid on;
    xlabel('Time (us)', 'fontsize', 16);
    ylabel('Phase Deviation \phi_e', 'fontsize', 16);
    set(gca, 'fontsize', 16);
end

% ============================================================
%%  Plot 3: 相位噪声
% ============================================================
figure('Name','Phase Noise');
rbw  = 1e3;
f    = 0:rbw:fv-rbw;
PN_tdc = (2*pi)^2/12*(delta_TDC/Tv)^2/fref * ones(size(f));
PN_vco = S_DCO_offset * (f_offset./f).^2;
Hol    = (alpha + rho*fref./(1 - exp(-2*pi*1j*f/fref))/fref) ...
       .* fref ./ (1 - exp(-2*pi*1j*f/fv)) / fv .* sinc(f*Tref);
Hcl_ref = fcw * Hol ./ (1+Hol);
Hcl_tdc = Hol ./ (1+Hol);
Hcl_dco = 1 ./ (1+Hol);
Scl_ref = S_ref   * abs(Hcl_ref.^2);
Scl_tdc = PN_tdc  .* abs(Hcl_tdc.^2);
Scl_dco = PN_vco  .* abs(Hcl_dco.^2);
Scl_tot = Scl_ref + Scl_tdc + Scl_dco;
semilogx(f, 10*log10(Scl_ref), 'b--'); grid on; hold on;
semilogx(f, 10*log10(Scl_tdc), 'b-');
semilogx(f, 10*log10(Scl_dco), 'b-');
semilogx(f, 10*log10(Scl_tot), 'r--', 'LineWidth', 1.5, 'DisplayName', 's domain');
PN_dcoQ = 1/12*(KDCO/2^NB_dco./f).^2/fref.*sinc(f/fref).^2;
semilogx(f, 10*log10(PN_dcoQ.*abs(Hcl_dco.^2)), 'b-');

PHE = tckv_div(settle_sample:step_DCO) / Tv * 2*pi;
PHE = PHE - mean(PHE);
fprintf('\nReal td:\n');
[Px_sine, f_psd] = fun_calc_psd_dbs(PHE, fv, rbw);
n_half  = floor(length(f_psd)/2);
f_psd   = f_psd(1:n_half);
Px_sine = Px_sine(1:n_half);
semilogx(f_psd, Px_sine, 'g-', 'LineWidth', 1, 'DisplayName', 'time domain');
Px_R = 10.^(Px_sine/10);
fstep_psd = f_psd(2) - f_psd(1);
sum_Y = sum(Px_R(f_psd > 1e1)) * fstep_psd;
jitter = sqrt(2*sum_Y) / (2*pi*fv);
xlim([1e3, fv/2]); ylim([-180, -40]);
title(['fv=', num2str(fv/1e9), 'GHz; rms jitter=', num2str(floor(jitter*1e15)), ' fs']);

% ============================================================
%%  Plot 4: ADPLL 输出频谱
% ============================================================
if 1
    Out_sine = sin(2*pi*tckv(1+10000:round(Sim_Time*fcw-2e4)+10000)/Tv);
    fprintf('Real sd:\n');
    [Px_sine2, f2] = fun_calc_psd_dbs(Out_sine, fv, rbw, fstep_psd);
    Px_sine2 = fftshift(Px_sine2);
    figure('Name','Spectrum');
    plot(f2+fv/2, Px_sine2 - 10*log10(2));
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Phase noise Spectrum (dBc/Hz)');
    xlim([fv-3e6, fv+3e6]);
    ylim([-180, 0]);
end

% ============================================================
%%  Plot 5: FFT Spur 精细观察
% ============================================================
if 1
    idx_start = settle_sample + 1;
    idx_end   = step_DCO - 1;
    t_use     = tckv(idx_start:idx_end);
    phi_ckv   = 2*pi*(t_use - (idx_start:idx_end)'*Tv)/Tv;
    phi_ckv   = phi_ckv - mean(phi_ckv);
    ckv_bb    = exp(1j*phi_ckv);
    NFFT      = 2^nextpow2(length(t_use));
    CKV_F     = fftshift(fft(ckv_bb, NFFT)) / NFFT;
    f_bb      = (-NFFT/2 : NFFT/2-1) * (fv/NFFT);
    f_ckv     = f_bb + fv;
    psd_ckv   = 20*log10(abs(CKV_F)+1e-30);
    psd_dbc   = psd_ckv - psd_ckv(NFFT/2+1);

    figure('Name','FFT');
    plot(f_ckv/1e9, psd_dbc, 'g', 'LineWidth', 1);
    grid on;
    xlabel('Frequency (GHz)');
    ylabel('Power (dBc)');
    title(['CKV Spectrum  fv = ', num2str(fv/1e9), ' GHz'], 'fontsize', 14);
    xlim([(fv-0.8e6)/1e9, (fv+0.8e6)/1e9]);
    ylim([-90, 5]);

    f_spur_theory = mod(FCW_F,1) * fref;
    if f_spur_theory > fref/2; f_spur_theory = fref - f_spur_theory; end
    [~, idx_s] = min(abs(f_bb - f_spur_theory));
    sr = max(1,idx_s-5) : min(NFFT,idx_s+5);
    [spur_val, ~] = max(psd_dbc(sr));
    noise_floor   = mean(psd_dbc(min(NFFT,idx_s+10) : min(NFFT,idx_s+50)));
end

% ============================================================
%%  Plot 6: DTC 调试图
% ============================================================
if 1
    figure('Name','DTC Debug');
    time_axis = (1:Sim_Time) * Tref * 1e6;

    ax1 = subplot(4,1,1);
    plot(time_axis, DCW_P_log, 'r', 'LineWidth', 1, 'DisplayName', 'P DTC'); hold on;
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
    plot(time_axis, Phie_error_log, 'y', 'LineWidth', 1);
    grid on; ylabel('phie'); title('TDC Phie error');

    t_start = settle_start_fref * Tref * 1e6;
    t_end   = min((settle_start_fref + 2000) * Tref * 1e6, Sim_Time*Tref*1e6);
    linkaxes([ax1, ax2, ax3, ax4], 'x');
    xlim([t_start, t_end]);
end

% ============================================================
%%  Plot 7: 校准收敛曲线
% ============================================================
if 1
    figure('Name','Calibration Convergence');
    time_axis = (1:Sim_Time) * Tref * 1e6;

    if LMS_POLY_EN
        n_sub = 3;
    else
        n_sub = 2;
    end


    mode_str = 'sign-LMS';


    sp_idx = 0;

    sp_idx = sp_idx + 1;
    ax1=subplot(n_sub, 1, sp_idx);
    plot(time_axis, koft_log*1e12, 'r', 'LineWidth', 1);
    grid on; ylabel('Koft (ps)');
    title(['Koft 校准（', mode_str, '）']);

    sp_idx = sp_idx + 1;
    ax2=subplot(n_sub, 1, sp_idx);
    plot(time_axis, kdtc_log, 'y', 'LineWidth', 1);
    grid on; ylabel('gain');
    title(['DTC 增益校准（', mode_str, '）']);

    if LMS_POLY_EN
        sp_idx = sp_idx + 1;
        ax3=subplot(n_sub, 1, sp_idx);
        plot(time_axis, alpha_odd_log, 'm', 'LineWidth', 1.5);
        grid on; ylabel('\alpha_{odd} Coeff');
        title(['勒让德系数 \alpha_{odd}（', mode_str, '）']);
        linkaxes([ax1, ax2, ax3], 'x');
    else
        linkaxes([ax1, ax2], 'x');
    end

end

% ============================================================
%%  终端汇总输出
% ============================================================
inl_names = {'理想无INL', '纯偶函数', '偶+奇混合', '正弦形', '纯奇函数(三次方)'};
fprintf('\n===== 仿真结果汇总  v6 =====\n');
fprintf('CHOPPING_EN  = %d\n', CHOPPING_EN);
fprintf('DIFF_DTC_EN  = %d\n', DIFF_DTC_EN);
fprintf('DITHER_EN    = %d，DITHER_AMP = %d\n', DITHER_EN, DITHER_AMP);
fprintf('LMS_POLY     = %d\n', LMS_POLY_EN);
fprintf('\nMASH Order   = %d\n', MASH_ORDER);
fprintf('INL: Type%d [%s]  EVEN=%6.2fps  ODD=%6.2fps  Mismatch=%6.2f%%\n', ...
    INL_TYPE, inl_names{INL_TYPE}, A_INL_EVEN*1e12, A_INL_ODD*1e12, INL_MISMATCH*100);


fprintf('\nFCW_F = 1/%.0f  →  spur @ %.2f kHz\n', 1/FCW_F, FCW_F*fref/1e3);
fprintf('RMS Jitter          = %.1f fs\n', jitter*1e15);
fprintf('Fractional Spur     = %.2f dBc @ %.2f kHz\n', spur_val, f_spur_theory/1e3);
fprintf('Noise Floor         = %.2f dBc/bin\n', noise_floor);

fprintf('\nKoft           = %d\n', Koft);
fprintf('gain_DTC_est   = %d\n', gain_DTC_est);
fprintf('alpha_odd      = %d\n', alpha_odd);
fprintf('===========================\n');