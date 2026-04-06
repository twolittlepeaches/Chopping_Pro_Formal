%% ================================================================
%  DTC Chopping / Differential DTC / Dither PLL 行为级仿真
%  基于 TIPM_Archi_SwapTDCinput_v2.m 修改 的 Chopping_pro_td_v1
%
%  修改说明：
%    1. 增加 CHOPPING_EN / DIFF_DTC_EN / DITHER_EN 开关
%    2. 重建 INL 模型（偶函数+奇函数可分别控制）
%    3. 差分 DTC
%    4. Dither：MAIN DTC 码加随机量，TDC 输出数字补偿
% ================================================================
clc; clear; close all;

%% ============================================================
%  功能开关 该版本中，chopping与DIFF只能选一个开启，dither可以一起开
% ============================================================
CHOPPING_EN  = 1;   % 1=随机 chopping，0=固定 SWAP=0（看 spur 先关掉）

DIFF_DTC_EN  = 0;   % 1=差分 DTC（OFFSET 跟随 MAIN 互补码）

DITHER_EN    = 0;   % 1=MAIN DTC 加 dither + TDC 端数字补偿
DITHER_AMP   = 128;   % ±4 个 LSB（可调）

%% ============================================================
%  INL 类型选择 
%  1 = 纯偶函数（抛物线，VS-DTC 主要非线性）
%  2 = 偶函数为主 + 少量奇函数
%  3 = 原始正弦 INL
% ============================================================
INL_TYPE         = 2;      % 建议先用 1，spur 最明显
A_INL_EVEN       = 10e-12; % 偶函数 INL 幅度（ps 量级，10ps 典型值）
A_INL_ODD        = 2e-12;  % 奇函数 INL 幅度（比偶函数小一个量级）
%A_INL_EVEN       = 0; % 偶函数 INL 幅度（ps 量级，10ps 典型值）
%A_INL_ODD        = 0;  % 奇函数 INL 幅度（比偶函数小一个量级）
INL_MISMATCH     = 0.05;   % 两路 DTC INL 幅度失配比例（5%）

%% ============================================================
%  基本 PLL 参数
% ============================================================
rng('default'); rng(23);
Sim_Time = 2^19;   % 仿真周期数（FREF 周期），越大频率分辨率越高
settle_sample = 10e6; % The unit is in CKV cycles.

fref = 100e6;
Tref = 1 / fref;
f0   = 5.5e9;
T0   = 1 / f0;

%    FCW 选择：near-integer 信道，spur 在低频，PLL 不滤，最容易看到
%    FCW_F = 1/256 → spur 在 fref/256 ≈ 390 kHz（near-integer 最严苛）
%    如果要看 mid-fractional，改成 fcw = 55 + 127/256
fcw   = 55 + 255/256;
FCW_I = floor(fcw);
FCW_F = mod(fcw, 1);

fv = fcw * fref;
Tv = 1 / fv;
fv_min = 4.0e9;
Tv_min = 1 / fv_min;

% 分数相位锯齿序列
%理想的小数相位累加轨迹,模拟数字累加器的行为
%若FCW_F 0.25,生成一个全是 0.25 的数组,cumsum（累加和）计算出：0.25, 0.50, 0.75, 1.00, 1.25, ...
%代表在每一个参考时钟上升沿，理想的分数相位应该增加多少
PHRF_full = mod(cumsum(FCW_F * ones(1, Sim_Time)), 1);
%相位取模，也就是生成锯齿波,把无限增长的累加值变成了[0, 1)区间的锯齿波
PHRF_full(PHRF_full >= 1) = 0;

%溢出标志位,检测PHRF是否溢出,控制MMD分频比
OV = [0, (PHRF_full(2:end) - PHRF_full(1:end-1)) < 0];

fprintf('FCW = %g, FCW_F = 1/%.0f\n', fcw, 1/FCW_F);
fprintf('预期 spur 位置: %.2f kHz\n', FCW_F*fref/1e3);

%% ============================================================
%  DTC / TDC 基本参数
% ============================================================
delta_TDC = 0.5e-15;    % TDC 分辨率 0.5 fs
DTC_reso  = 4e-15;      % DTC 最小步长 4 fs/LSB
KDCO      = 10e3;       % DCO 增益 Hz/code
NB_dco    = 5;          % DCO 量化位数
Mdcorun   = 2;          % DCO 数组冗余系数

% DTC 满量程码字数（约覆盖一个 最大 DCO 周期）
N_DTC = round(Tv_min / DTC_reso);   
% 实际工作范围约 N_DTC/2（PHRF 在 ±0.5 之间）


%% ============================================================
%  噪声参数
% ============================================================
f_offset        = 1e6;
S_DCO_offset_dB = -126;
S_DCO_offset = 10^(S_DCO_offset_dB/10);

% time domain
sigma_w         = f_offset/fv * sqrt(Tv) * sqrt(S_DCO_offset);
delta_Tw        = sigma_w * randn(round(Sim_Time*fcw*Mdcorun), 1);

%%% DCO thermal noise
S_DCO_thermal_dB = -160;
S_DCO_thermal = 10^(S_DCO_thermal_dB/10);

sigma_j = 1/(2*pi*fv)*sqrt(S_DCO_thermal*fv);
delta_Tj = sigma_j*randn(round(Sim_Time*fcw*Mdcorun),1);

%%% Reference noise
S_ref_dB = -170;
S_ref = 10^(S_ref_dB/10);
sigma_ref = 1/(2*pi*fref)*sqrt(S_ref*fref);
delta_Tref = sigma_ref*randn(Sim_Time,1);

%% ============================================================
%  环路滤波器参数
% ============================================================
alpha = 2^(-5);
rho   = 2^(-12);
I_path = 0;
yiir1=0; yiir2=0; yiir3=0; yiir4=0;
lambda1=1; lambda2=1; lambda3=1; lambda4=1; % =1 相当于关闭 IIR

%% ============================================================
%  变量初始化
% ============================================================
otw         = zeros(Sim_Time+1, 1);
tR          = zeros(Sim_Time, 1);
tR_norm     = zeros(Sim_Time, 1);
trise       = zeros(Sim_Time, 1);
phie        = zeros(Sim_Time, 1);  % phase error
DCW_log     = zeros(Sim_Time, 1);  % 记录 MAIN DTC 码（调试用）
INL_err_log = zeros(Sim_Time, 1);  % 记录 INL 误差序列（调试用）
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

% 在循环开始前添加
dither_actual = zeros(Sim_Time, 1);
PHRF_cur_log = zeros(Sim_Time, 1);
DCW_main_log = zeros(Sim_Time, 1);
DCW_offset_log = zeros(Sim_Time, 1);
DCW_delta_log = zeros(Sim_Time, 1);
DTC_Delay_time_error_log = zeros(Sim_Time, 1);
DCW_main_ideal = 0;
DCW_main_actual = 0;
DCW_offset_ideal = 0;
DCW_offset_actual = 0;


%  ---- Chopping 序列 ----
if CHOPPING_EN
    EN_SWAP = randi([0,1], 1, Sim_Time);   % 随机 0/1
else
    EN_SWAP = zeros(1, Sim_Time);          % 固定 0
end

%  ---- Dither 参数 ----

if DITHER_EN
    % 均匀分布随机整数，范围 [-DITHER_AMP, +DITHER_AMP]
    dither_seq = randi([-DITHER_AMP, DITHER_AMP], 1, Sim_Time);
else
    dither_seq = zeros(1, Sim_Time);
end

%  ---- 差分 DTC 参数 ----
%  C_diff：差分对称中心（理想情况 = DTC 工作范围中点）
%  MAIN 工作范围约 [0, N_DTC/2]，中点约 N_DTC/4  /2？
C_diff = 0;

%  ---- 增益和偏置校准初始值 ----
gain_DTC_est = 0.9 * Tv / DTC_reso;   % 初始增益估计
%Koft         = C_diff * DTC_reso;     % OFFSET DTC 偏置初始值（对应差分中心延时）
Koft = 300e-12;                      %offset dtc delay init
step_DCO = 0;

%% ============================================================
%  INL 建模 
% ============================================================
code_vec  = (0 : N_DTC-1);
% 归一化坐标：以中点为原点，范围 [-1, 1]
u = (code_vec - N_DTC/2) / (N_DTC/2);

switch INL_TYPE
    case 1  % 纯偶函数：抛物线
        % 物理依据：VS-DTC buffer 斜率效应，关于工作中点对称
        INL_main_raw   = A_INL_EVEN * u.^2;
        INL_offset_raw = A_INL_EVEN * (1 + INL_MISMATCH) * u.^2;

    case 2  % 偶函数（主）+ 奇函数（次）
        INL_main_raw   = A_INL_EVEN * u.^2 + A_INL_ODD * u;
        INL_offset_raw = A_INL_EVEN*(1+INL_MISMATCH)*u.^2 ...
                       + A_INL_ODD*(1-INL_MISMATCH)*u;

    case 3  % 正弦形（原始代码风格）
        INL_main_raw   = A_INL_EVEN * sin(3*pi*code_vec/N_DTC);
        INL_offset_raw = A_INL_EVEN*(1+INL_MISMATCH) * sin(3*pi*code_vec/N_DTC);
end

% 去线性趋势（排除增益误差，只保留非线性）
INL_main   = detrend(INL_main_raw,   1);
INL_offset = detrend(INL_offset_raw, 1);


%% ============================================================
%  INL 快速查表函数
% ============================================================

get_INL_main = @(c) INL_main(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));
get_INL_offset = @(c) INL_offset(max(1, min(N_DTC, round(c) + round(N_DTC/2) + 1)));

%% ============================================================
%  ★★★ 主仿真循环 ★★★
% ============================================================
fprintf('开始仿真，Sim_Time = 2^%.0f = %d FREF cycles...\n', log2(Sim_Time), Sim_Time);

for step = 1:Sim_Time

    %% -- 1 FREF 时间戳 --
    tR(step)     = step * Tref + delta_Tref(step);
    tR_norm(step)= step * Tref;

    %% -- 2 DCO 计数（整数 N 个 DCO 周期）--
    if step == 1
        Delta_TDEV(1) = T0 - 1/(f0 + otw(1)*KDCO);
    else
        Delta_TDEV(step) = T0 - 1/(f0 + otw(step)*KDCO);
    end

    div_cnt = 0;
    while div_cnt < (FCW_I + OV(step))% 一个FREF周期内,DCO COUNT(/56/55...)   
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
            TDEV(step_DCO)        = TDEV(step_DCO-1) + Delta_TDEV(step);% 累积DCO相位误差(DCO 的相位 = 周期误差的积分)
            % 叠加噪声
            TDEVpn(step_DCO)      = TDEVpn(step_DCO-1) ...
                                  + delta_Tj(step_DCO) - delta_Tj(step_DCO-1) ...
                                  + delta_Tw(step_DCO);
            % DCO时间戳 (游走+热噪声)
            tckv(step_DCO)        = step_DCO*T0 - TDEV(step_DCO) + TDEVpn(step_DCO);
            tckv_norm(step_DCO)   = step_DCO*Tv;
            tckv_div(step_DCO)    = tckv(step_DCO) - step_DCO*Tv;
            tckv_period(step_DCO) = tckv(step_DCO) - tckv(step_DCO-1);
            Rv(step_DCO)          = Rv(step_DCO-1) + 1;
        end
        div_cnt = div_cnt + 1;
    end

    %%% 整数部分DCO计数完毕,剩余分数DCO周期,由DTC/TDC处理
    %理论上,DTC,TDC随CKV变，DTC DELAY为锯齿状
    %为简化程序,DTC/TDC直接在最后一个CKV变,不会改变spur频率,只改变幅度,不影响系统对spur的抑制作用
    
    %%% DTC的引入是为了解决小数分频带来的大spur,但是由于INL,带来了新的,小的spur
    % 无Chopping:INL_sequence = [INL(x1), INL(x2), ..., INL(x256)] 256个周期
    % Chopping:EN_SWAP = [0, 1, 0, 1, 1, 0, ...],随机,INL_chopped = [+INL(x1), -INL(x2), +INL(x3), -INL(x4), ...],序列不再周期性！
    % 结果是近白噪声,spur抑制,但是噪底抬高

    %%% 在理想情况下,chopping 应与 DCO 周期对齐以获得最强抑制
    % 行为级模型中在 FREF 速率 chopping 是一种保守近似,能正确反映 spur 抑制机制,但会低估抑制深度并高估相关性
   
    %fractional phase 拉到[-0.5, +0.5),对称性,chopping抵消offset / INL
    
    
    %% -- 3 分数相位（中心化到 [-0.5, 0.5)）--
    PHRF_cur = PHRF_full(step) - 0.5;

    %% -- 4.1 Chopping
    %    SWAP=0: MAIN 接 FREF → MAIN 延时加在 tR 一侧（ref）
    %            OFFSET 接 MMD → OFFSET 延时加在 tckv 一侧（div）
    %    SWAP=1: MAIN 接 MMD  → MAIN 延时加在 tckv 一侧（div）
    %            OFFSET 接 FREF→ OFFSET 延时加在 tR 一侧（ref）
    %
    %  TDC 测量：trise = (DCO 侧时刻) - (REF 侧时刻)
    if EN_SWAP(step)>0.5        %chopping state 1,正相位 → MAIN DTC，OFFSET DTC 加正偏置
        % MAIN DTC处理分数部分
        DCW_main_ideal = (PHRF_cur-0.0) * gain_DTC_est;% DTC控制字
        
        % 加 dither
        DCW_main_actual = DCW_main_ideal + dither_seq(step);
        DCW_main_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_main_actual));

        % 记录实际施加的 dither（限幅后）
        dither_actual(step) = DCW_main_actual - DCW_main_ideal;

        % MAIN DTC 实际延时
        t_main = floor(DCW_main_actual)*DTC_reso + 250e-12 ...
               + get_INL_main(DCW_main_actual);% Q:为什么chop 0/1 的MAIN DTC的oft不一样
        % OFFSET DTC提供固定偏置(专门用来校准 offset)
        t_offset = Koft*(+1);

        DCW_offset_actual = t_offset/DTC_reso;

    else                       %chopping state 0,负相位 → MAIN DTC 反向，OFFSET DTC 反向
        % MAIN DTC处理分数部分
        DCW_main_ideal = (0-PHRF_cur) * gain_DTC_est;% !!! DTC控制字反转
        
        % 加 dither
        DCW_main_actual = DCW_main_ideal + dither_seq(step);
        DCW_main_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_main_actual));

        % 记录实际施加的 dither（限幅后）
        dither_actual(step) = DCW_main_actual - DCW_main_ideal;

        % MAIN DTC 实际延时
        t_main = floor(DCW_main_ideal)*DTC_reso +200e-12 ...
                + get_INL_main(DCW_main_actual);
        % OFFSET DTC反向(专门用来校准 offset)
        t_offset = -Koft*(-1);% !!! 偏置反转
        t_offset = Koft;% !!! 偏置反转
        DCW_offset_actual = t_offset/DTC_reso;
       
    end


    %% -- 4.2 Diff
    %  MAIN DTC 控制码计算 --
    %
    %  chopper.v:
    %    SWAP=0: MAIN 接 FREF（ref 路径），code = 1 - PHR_F → 互补方向
    %    SWAP=1: MAIN 接 MMD（div 路径），code = PHR_F     → 正向
    %
    %  以 C_diff 为工作中心，PHRF_cur 在 [-0.5, 0.5) 范围内偏移
    %
    if DIFF_DTC_EN   
        DCW_main_ideal = 0 - ((PHRF_cur+0.5)/2) * gain_DTC_est;
        DCW_offset_ideal = 0 + ((PHRF_cur+0.5)/2) * gain_DTC_est;
    

        % 加 dither
        DCW_main_actual = DCW_main_ideal + dither_seq(step);
        DCW_main_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_main_actual));

        % 记录实际施加的 dither（限幅后）
        dither_actual(step) = DCW_main_actual - DCW_main_ideal;


        DCW_offset_actual = DCW_offset_ideal ;
        DCW_offset_actual = max(-N_DTC/2+1, min(N_DTC/2-1, DCW_offset_actual));


        % DTC 实际延时
        t_main = DCW_main_actual * DTC_reso + get_INL_main(DCW_main_actual);
        t_offset =  0e-12-Koft*(-1) + DCW_offset_actual * DTC_reso + get_INL_offset(DCW_offset_actual);
        
       
    end
    % 记录 INL 误差（用于调试和 plot）
    % INL_err_log(step) = get_INL_main(DCW_main_actual);
    DCW_main_log(step)     = DCW_main_actual;
    DCW_offset_log(step)     = DCW_offset_actual;

    PHRF_cur_log(step)=PHRF_cur;


    %% -- (6) TDC 输入时刻差（trise）--
    %
    %  根据 chopper.v：
    %    SWAP=0: MAIN 接 FREF → MAIN 延时加在 tR 一侧（ref）
    %            OFFSET 接 MMD → OFFSET 延时加在 tckv 一侧（div）
    %    SWAP=1: MAIN 接 MMD  → MAIN 延时加在 tckv 一侧（div）
    %            OFFSET 接 FREF→ OFFSET 延时加在 tR 一侧（ref）
    %
    %  TDC 测量：trise = (DCO 侧时刻) - (REF 侧时刻)
    %
    if EN_SWAP(step)  % SWAP=1
        trise(step) = (tckv(step_DCO) + t_main) - (tR(step) + t_offset);
        DCW_delta_log(step) = -(t_offset - t_main)/DTC_reso;
        DTC_Delay_time_error_log(step) = get_INL_main(DCW_main_actual)-get_INL_offset(DCW_offset_actual);
    else              % SWAP=0
        trise(step) = (tckv(step_DCO) + t_offset) - (tR(step) + t_main);
        DCW_delta_log(step) = (t_offset - t_main)/DTC_reso;
        DTC_Delay_time_error_log(step) = get_INL_offset(DCW_offset_actual)-get_INL_main(DCW_main_actual);
    end

    %DCW_delta_log(step) = DCW_offset_actual - DCW_main_actual;
    %DCW_delta_log(step) = (t_offset - t_main)/DTC_reso;

    % Dither 数字补偿：把 dither 引入的理想延时减掉
    % （注意补偿方向和 MAIN DTC 所在路径一致）
    if DITHER_EN
        dither_delay = dither_actual(step) * DTC_reso;
        if EN_SWAP(step)   % MAIN 在 tckv 侧
            trise(step) = trise(step) - dither_delay;
        else               % MAIN 在 tR 侧，符号相反
            trise(step) = trise(step) + dither_delay;
        end
    end

    %% -- (7) TDC 量化 --
    phie(step) = (floor(trise(step)/delta_TDC) - 0.5) * delta_TDC / Tv;

    %% -- (8) 校准环路（Sign-LMS）--

    % % INVKDTC 增益校准：使 phie 和 PHRF_cur 不相关
    % % 误差方向：phie 和 PHRF_cur 同号 → 增益偏小 → 增大 gain
    % gain_update_sign = sign(phie(step)) * sign(PHRF_cur);
    % gain_DTC_est = gain_DTC_est - 1e-1 * gain_update_sign;
    % 
    % 
    % % OFTDTC 偏置校准（非差分模式）：使 phie 和 SWAP_EN 不相关
    % if ~DIFF_DTC_EN
    %     swap_sign = 2*EN_SWAP(step) - 1;   % 0→-1, 1→+1
    %     Koft = Koft + 2e-14 * swap_sign * sign(phie(step));
    % end

    if CHOPPING_EN
        % OFFSET DTC校准,符号LMS,如果chopping后误差还有DC,就说明offset不对,继续推Koft
        Koft = Koft + 2*1e-14*(sign(EN_SWAP(step)-0.5))*sign(phie(step));

    end


    % DTC增益校准,逼近,让 fractional phase 的正负，对应相位误差对称
    gain_DTC_est = gain_DTC_est  + (-1)*1e-1*sign(phie(step))*sign(PHRF_cur);
    %gain_DTC_est = 1.*Tv/DTC_reso;


    %% -- (9) 环路滤波器 --
    % 积分通道
	I_path = I_path + phie(step);
    % 4阶IIR,时域递归方程 (可关闭，这里lambda=1相当于关闭)
    yiir1 = (1-lambda1)*yiir1 + lambda1*(phie(step));
    yiir2 = (1-lambda2)*yiir2 + lambda2*yiir1;
    yiir3 = (1-lambda3)*yiir3 + lambda3*yiir2;
    yiir4 = (1-lambda4)*yiir4 + lambda4*yiir3;
    
    P_path = alpha*yiir4;
    % Type-II输出
    LFout = P_path+rho*I_path;

    %%  DCO quantazation,会产生DCO量化噪声,也会被S域模型拿去算PN
    otw(step+1) = floor(LFout * fref/KDCO * 2^NB_dco) / 2^NB_dco;

end

fprintf('仿真完成，step_DCO = %d\n', step_DCO);

%% ============================================================
%  稳态区间截取
% ============================================================
% 截取后半段（前半段 PLL 还未完全锁定）
settle_start_fref = round(Sim_Time * 0.4);   % 取后 60% 的 FREF 周期
settle_start_dco  = round(step_DCO * 0.4);   % 对应 DCO 周期



%% Plot the Inst frequency deviation of CKV
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
    Fig = figure('Color',[1 1 1]);
    plot(1e6*tckv(1:settle_sample),fckv_div/1e6);
    hold on
    plot(1e6*tckv(1:settle_sample),fckv_div_avg/1e6,'r-');
    grid on;
    xlabel('Time (us)','fontsize',16);
    ylabel('Freq Deviation (MHz)','fontsize',16);
    %xlim([0*Tv*1e6 settle_sample*Tv*1e6])
    set(gca,'YTick',-100:5:100);%This is just to tune the y-axis step, rather than the range.
    set(gca,'fontsize',16)
    set(findobj(gca,'Type','line'),'LineWidth',2)
    set(gcf,'color','w');
end

%% Plot the Inst phase error phie
if 1
    Fig = figure('Color',[1 1 1]);
    %phie = (tckv_div(1:Sim_Time)-mean(tckv_div(1:Sim_Time)))./Tv*2*pi;
    plot((1:Sim_Time)*Tref*1e6, phie, 'b', 'LineWidth', 0.5);
    hold on;grid on;
    xlabel('Time (us)','fontsize',16)
    ylabel('Phase Deviation \phi_e','fontsize',16);
    %xlim([0*Tv settle_sample*Tv*1e6])
    set(gca,'YTick',0:0.5:5);
    set(gca,'fontsize',16)
    set(findobj(gca,'Type','line'),'LineWidth',2)
    set(gcf,'color','w');
end

%% Plot the Inst Period deviation of CKV
if 1
    Fig = figure('Color',[1 1 1]);
    period_div = (tckv_div(1:step_DCO)-mean(tckv_div(settle_sample:step_DCO)));
    plot(tckv_norm(1:step_DCO)*1e6,period_div*1e12);
    hold on;grid on;
    xlabel('Time (us)','fontsize',20)
    ylabel('Period Deviation (ps)','fontsize',20);
    %xlim([0*Tv settle_sample*Tv*1e6]);
    set(gca,'YTick',-2500:500:2500);
    set(gca,'fontsize',16);
    set(findobj(gca,'Type','line'),'LineWidth',2);
    set(gcf,'color','w');
end

%% S-domain model
figure;
rbw = 1e3;
fstep = rbw;
f = 0:rbw:fv-rbw;
PN_tdc = (2*pi)^2/12*(delta_TDC/Tv)^2/fref * ones(size(f));
PN_vco = S_DCO_offset * (f_offset ./f).^2;
%Hol = (alpha + rho*fref./(j*2*pi*f)).*fref./(j*2*pi*f);
Hol = (alpha + rho * fref./(1 - exp(-2*pi*j*f/fref)) /fref ) .* fref ./ (1 - exp(-2*pi*j*f/fv))/fv.*sinc(f*Tref);
Hcl_ref = fcw * Hol./(1+Hol);
Hcl_tdc = Hol./(1+Hol);
Hcl_dco = 1./(1+Hol);
Scl_ref = S_ref * abs(Hcl_ref.^2);
Scl_tdc =  PN_tdc .* abs(Hcl_tdc.^2);
Scl_dco = PN_vco .* abs(Hcl_dco.^2);
Scl_tot = Scl_ref + Scl_tdc + Scl_dco;
semilogx(f,10*log10(Scl_ref),'b--');grid on;hold on;
semilogx(f,10*log10(Scl_tdc),'b-');
semilogx(f,10*log10(Scl_dco),'b-');
semilogx(f,10*log10(Scl_tot),'r--');
PN_dcoQ = 1/12.*(KDCO/2^NB_dco./f).^2*1/fref.*sinc(f/fref).^2;
semilogx(f,10*log10(PN_dcoQ.*abs(Hcl_dco.^2)),'b-')

PHE = tckv_div(settle_sample:step_DCO)/Tv*2*pi;
PHE = PHE - mean(PHE);
rbw = 1e3;
[Px_sine,f] = fun_calc_psd_dbs(PHE, fv, rbw);
fstep = f(2)-f(1);
f = f(1:floor(length(f)/2));
Px_sine = Px_sine(1:floor(length(Px_sine)/2));
semilogx(f,Px_sine,'k-');
Px_sine_R = 10.^(Px_sine/10);
sum_Y = sum(Px_sine_R(find(f>1e1)))*fstep;
jitter = sqrt(2*sum_Y) / (2*pi*fv);

xlim([1e3,fv/2]);
ylim([-180,-40]);
title(['fv= ',num2str(fv/1e9),'GHz; rms jitter= ',num2str(floor(jitter*1e15)),' fs']);


%% Spectrum for the ADPLL output
if 1
    Out_sine = sin(2*pi*tckv(1+10000:round(Sim_Time*fcw-2e4)+10000)/Tv);
    figure;plot(Out_sine);
    [Px_sine,f] = fun_calc_psd_dbs(Out_sine, fv, rbw, fstep);
    Px_sine = fftshift(Px_sine);
    Fig = figure('Color',[1 1 1]);
    plot(f+fv/2,Px_sine-10*log10(2));
    grid on;
    xlabel('Frequency (Hz)','fontsize',16);
    ylabel('Phase noise Spectrum (dBc/Hz)','fontsize',16);
    xlim([fv-3*1e6 fv+3*1e6]);
    ylim([-180,0]);
    set(gca,'fontsize',16);
    set(findobj(gca,'Type','line'),'LineWidth',2);
    set(gcf,'color','w');
end

%% ============================================================
%  Plot 5: DTC 调试图（稳态段放大）
% ============================================================
if 1
    fig_debug = figure('Color',[1 1 1], 'Name', 'DTC 与 PHRF 逻辑关系对照');
    time_axis = (1:Sim_Time) * Tref * 1e6; % 时间轴 (us)
    
    % --- 上半部分: DTC 控制码 ---
    ax1 = subplot(4,1,1);
    plot(time_axis, DCW_main_log, 'r', 'LineWidth', 1, 'DisplayName', 'Main DTC');
    hold on;
    plot(time_axis, DCW_offset_log, 'b', 'LineWidth', 1, 'DisplayName', 'Offset DTC');
    hold on;
    plot(time_axis, DCW_delta_log, 'g', 'LineWidth', 1, 'DisplayName', 'delta');
    grid on;
    ylabel('DTC Code (LSB)');
    title('DTC 控制码序列 (Red: Main, Blue: Offset)');
    legend('Location', 'northeast');
    
    % --- 下半部分: PHRF 分数相位 ---
    ax2 = subplot(4,1,2);
    plot(time_axis, PHRF_cur_log, 'g', 'LineWidth', 1, 'DisplayName', 'PHRF\_cur');
    grid on;
    xlabel('Time (\mu s)');
    ylabel('Phase (Normalized)');
    title('输入分数相位 (PHRF\_cur)');

    ax3 = subplot(4,1,3);
    plot(time_axis, phie, 'y', 'LineWidth', 1, 'DisplayName', 'phie');
    grid on;
    xlabel('Time (\mu s)');
    ylabel('Phase (Normalized)');
    title('phie');
    
    ax4 = subplot(4,1,4);
    plot(time_axis , DTC_Delay_time_error_log , 'y', 'LineWidth', 1, 'DisplayName', 'DTC\_Delay\_time\_error');
    grid on;
    xlabel('Time (\mu s)');
    ylabel('time s');
    title('DTC\_Delay\_time\_error');
    % 设置显示范围 (稳态后的 500 个周期)
    t_start = settle_start_fref * Tref * 1e6;
    t_end   = (settle_start_fref + 2000) * Tref * 1e6;
    linkaxes([ax1, ax2, ax3, ax4], 'x'); % 关键：联动两个子图的 X 轴
    xlim([t_start, t_end]);
    
end


%% ============================================================
%  终端输出汇总
% ============================================================

% 标注预期 spur 频率
spur_f = FCW_F * fref;

fprintf('\n===== 仿真结果汇总 =====\n');
fprintf('\n===== Chopping pro V1 =====\n');
fprintf('CHOPPING_EN  = %d\n', CHOPPING_EN);
fprintf('DIFF_DTC_EN  = %d\n', DIFF_DTC_EN);
fprintf('DITHER_EN    = %d，DITHER_AMP = %d\n', DITHER_EN , DITHER_AMP);
fprintf('INL_TYPE     = %d  (EVEN=%.0fps, ODD=%.0fps, Mismatch=%.0f%%)\n', ...
    INL_TYPE, A_INL_EVEN*1e12, A_INL_ODD*1e12, INL_MISMATCH*100);
fprintf('FCW_F        = 1/%.0f,  预期spur @ %.2f kHz\n', 1/FCW_F, spur_f/1e3);
fprintf('RMS Jitter   = %.1f fs  (积分 1kHz ~ fv/2)\n', jitter * 1e15);
fprintf('========================\n');


