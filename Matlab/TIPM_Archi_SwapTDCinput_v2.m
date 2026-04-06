%% General Description
% This is the model for J Choi ISSCC 2021, TIPM architecture
% Note that DCWR stands different compared with ISSCC 2020 version. 
% This version is to chop TDC input. One with k*x+b, the other is with
% k*(1-x)+b.@2022.05.23


% Version 1.0, Peng CHEN, 2021.12.08
% Version 1.1, Peng CHEN, swap TDC input @ 2022.05.23
% Version 1.2, Peng CHEN, dig into the gain calibration direction.
% @2022.05.24
% It seems can work now. But cannot cover near-int-N channels. The benefit
% is suppressing spurs.

%% Initial Parameter Settings
clc;clear;close all;
Sim_Time = 2^18; % The number is in FREF cycles.
settle_sample = 10e6; % The unit is in CKV cycles.

rng('default');
rng(23);

%% Defining parameters

%fref
fref = 100e6;   
Tref = 1/fref;                 %~10ns 

% DCO free running
f0 = 5.5e9;
T0 = 1/f0;                     %~181ps

%FCW
fcw = 55+1/2^8;
FCW_I = floor(fcw); 
FCW_F = mod(fcw,1);

%理想的小数相位累加轨迹,模拟数字累加器的行为
%若FCW_F 0.25,生成一个全是 0.25 的数组,cumsum（累加和）计算出：0.25, 0.50, 0.75, 1.00, 1.25, ...
%代表在每一个参考时钟上升沿，理想的分数相位应该增加多少
PHRF = cumsum(FCW_F*ones(1,Sim_Time));
%相位取模，也就是生成锯齿波,把无限增长的累加值变成了[0, 1)区间的锯齿波
PHRF = mod(PHRF,1);
PHRF((PHRF>=1)) = 0;

%溢出标志位,检测PHRF是否溢出,控制MMD分频比
OV = [0,(PHRF(2:end)-PHRF(1:end-1))<0];

fv = fcw*fref;
Tv = 1/fv;

delta_TDC = 0.5e-15;
DTC_reso = 0.4e-14;
N_TDC_avg = 128;    %not used
KDCO = 10e3;

Mdcorun = 2;% 数组长度的安全冗余系数,防止仿真过程中数组越界

%% 环路滤波器设置
alpha = 2^(-5);% 比例通道增益
rho = 2^(-12);% 积分通道增益
I_path = 0;
yiir1 = 0; 
yiir2 = 0; 
yiir3 = 0; 
yiir4 = 0;
% IIR滤波器极点位置
lambda1 = 1/2^0;           % lambda = 1/2^0 = 1,相当于关闭IIR
lambda2 = 1/2^0;
lambda3=1/2^0;
lambda4=1/2^0;

%%% DCO 1/f^2 noise
f_offset = 1e6;
S_DCO_offset_dB = -126;
S_DCO_offset = 10^(S_DCO_offset_dB/10);
% time domain
sigma_w = f_offset/fv*sqrt(Tv)*sqrt(S_DCO_offset);
delta_Tw = sigma_w*randn(round(Sim_Time*fcw*Mdcorun),1);

%%% DCO thermal noise
S_DCO_thermal_dB = -160;
S_DCO_thermal = 10^(S_DCO_thermal_dB/10);
% time domain
sigma_j = 1/(2*pi*fv)*sqrt(S_DCO_thermal*fv);
delta_Tj = sigma_j*randn(round(Sim_Time*fcw*Mdcorun),1);

%%% Reference noise
S_ref_dB = -170;
S_ref = 10^(S_ref_dB/10);
sigma_ref = 1/(2*pi*fref)*sqrt(S_ref*fref);
delta_Tref = sigma_ref*randn(Sim_Time,1);

%% Variables initialization

% Variables changing at fref rate
Rr = zeros(round(Sim_Time),1);          %fref count
otw = zeros(size(Rr));                  %DCO control word
tR = zeros(size(Rr));
tR_norm = zeros(size(Rr));
Delta_TDEV_fast = zeros(size(Rr));
Delta_TDEV = zeros(size(Rr));
trise = zeros(size(Rr));
Tv_current = zeros(size(Rr));
Tv_est = zeros(size(Rr));
epsilon = zeros(size(Rr));
phie = zeros(size(Rr));                 %phase error
%EN_SWAP = 1*randi(2,1,Sim_Time)-1;      %chopping enable,random 0/1
EN_SWAP = zeros(size(Rr));      %chopping enable,random 0/1
PHF_R1 = zeros(size(Rr));
PHF_R2 = zeros(size(Rr));
PHF_R3 = zeros(size(Rr));
t_DTCr = zeros(size(Rr));               %main dtc delay
t_DTCd = zeros(size(Rr));               %offset dtc delay
DCWR1 = zeros(size(Rr));
DCWR2 = zeros(size(Rr));
DCWR3 = zeros(size(Rr));
DCW = zeros(size(Rr));

DAQ = 0;
URNG = unifrnd(0,1,1,length(Rr));


% Variables changing at fv rate
Rv = zeros(round(Sim_Time*fcw*Mdcorun),1);
TDEV = zeros(size(Rv));
TDEVpn = zeros(size(Rv));
tckv = zeros(size(Rv));
tckv_norm = zeros(size(Rv));
tckv_div = zeros(size(Rv));
tckv_period = zeros(size(Rv));
% number of bits for the key parameters;Here it is the frac bits 
NB_dco =  5; % We need <= 10KHz DCO resolution; But this value reduces when the BW reduces.
step_DCO = 0;


gain_DTC_est = 0.9*Tv/DTC_reso;
gain_DTC2_est = 0;                   % not used
gain_DTC3_est = 0;                   % not used
DTC_DNL = 1e-13*randn(1,floor(6.7*floor(Tv/DTC_reso)));
DTC_INL = cumsum(DTC_DNL);
DTC_INL = 5e-12*sin(3*pi*[1:length(DTC_DNL)]/length(DTC_DNL));
tmp = zeros(size(Rr));
tmp2 = zeros(size(Rr));
tmp3 = zeros(size(Rr));
local_cnt = 0;

Koft = 300e-12;                      %offset dtc delay init

%% Main Loop
for step = 1:Sim_Time    
    if step == 1  
        Rr(1) = 0 + fcw;        
        Delta_TDEV(1) = T0 - 1/(f0 + otw(1)*KDCO);     % DCO周期误差(由于频率控制字变化导致的周期偏差)   
        tR(1) = step * Tref + delta_Tref(1);           % 真实参考上升沿（带抖动）
        tR_norm(step) = step * Tref;                   % 理想参考（干净时间轴）
    else        
        Rr(step) = Rr(step-1) + fcw;                   % 相位累加器,整数部分由ref里DCO跑几拍定，小数部分由DTC+TDC补
        Delta_TDEV_fast(step) = T0 - 1/(f0 + 1*phie(step-1)*fref/KDCO*KDCO);      %T0/f0:DCO free running
        Delta_TDEV(step) = T0 - 1/(f0 + 1*otw(step)*KDCO);       %T0 DCO理想周期+控制导致的周期偏差    
        tR(step) = step * Tref + delta_Tref(step);        
        tR_norm(step) = step * Tref;        
    end  
    div_cnt = 0;
    while div_cnt < (FCW_I + OV(step))% 一个FREF周期内,DCO COUNT(/56/55...)   
        step_DCO = step_DCO + 1;
        if step_DCO == 1           
            TDEV(1) = 0 + Delta_TDEV(1);
            TDEVpn(1) = 0 + delta_Tj(1) - 0 + delta_Tw(1); 
            tckv(1) = 1*T0 - TDEV(1) + TDEVpn(1);            
            tckv_norm(1) = 1*Tv;            
            tckv_div(1) = tckv(1) - step_DCO*Tv;       
            tckv_period(1) =  tckv(1);            
            Rv(step_DCO) = 0 + 1;            
        else
            if (div_cnt <= -1)% div_cnt<=-1暂时没用到，不知道什么意思
                TDEV(step_DCO) = TDEV(step_DCO - 1) + Delta_TDEV_fast(step);% 累积DCO相位误差(DCO 的相位 = 周期误差的积分)
            else
                TDEV(step_DCO) = TDEV(step_DCO - 1) + Delta_TDEV(step);% 累积DCO相位误差(DCO 的相位 = 周期误差的积分)
            end
            % 叠加噪声
            TDEVpn(step_DCO) = TDEVpn(step_DCO-1) + delta_Tj(step_DCO)-delta_Tj(step_DCO-1) + delta_Tw(step_DCO);
            % DCO时间戳 (游走+热噪声)
            tckv(step_DCO) = step_DCO*T0 -(TDEV(step_DCO)) + TDEVpn(step_DCO);%(真实的 DCO 上升沿时间戳)
            tckv_norm(step_DCO) = step_DCO*Tv;  
            tckv_div(step_DCO) = tckv(step_DCO) - step_DCO*Tv;
            tckv_period(step_DCO) =  tckv(step_DCO) - tckv(step_DCO - 1);
            Rv(step_DCO) = Rv(step_DCO-1) + 1;       
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
    PHRF(step) = PHRF(step)-0.5;

    if EN_SWAP(step)>0.5        %chopping state 1,正相位 → MAIN DTC，OFFSET DTC 加正偏置
        % MAIN DTC处理分数部分
        DCW(step) = (PHRF(step)-0.0) * gain_DTC_est;% DTC控制字
        t_DTCr(step) = floor(DCW(step))*DTC_reso + 250e-12 + DTC_INL(1+floor(length(DTC_INL)/2+DCW(step)));% Q:为什么chop 0/1 的MAIN DTC的oft不一样
        % OFFSET DTC提供固定偏置(专门用来校准 offset)
        t_DTCd(step) = Koft*(+1);
        % 计算上升沿时刻差
        trise(step) = 1* (-(tR(step)+t_DTCd(step)) + (tckv(step_DCO)+t_DTCr(step))) ;
    else                       %chopping state 0,负相位 → MAIN DTC 反向，OFFSET DTC 反向
        % MAIN DTC处理分数部分
        DCW(step) = (0-PHRF(step)-0.0) * gain_DTC_est;% !!! DTC控制字反转
        t_DTCr(step) = floor(DCW(step))*DTC_reso +200e-12 + DTC_INL(1+floor(length(DTC_INL)/2+DCW(step)));
        % OFFSET DTC反向(专门用来校准 offset)
        t_DTCd(step) = 0e-12-Koft*(-1);% !!! 偏置反转
        % 计算上升沿时刻差
        trise(step) =  -(tR(step)+t_DTCr(step)) + (tckv(step_DCO)+t_DTCd(step)) ; 
    end
	% tmp(step) = DTC_INL(1+floor(length(DTC_INL)/2+DCW(step)));
    % tmp2(step) = Koft;
    % tmp3(step) = gain_DTC_est;
    
    
    % trise(step) = trise(step) + (trise(step)/2e-10)^2;
    % TDC量化: 两个上升沿的时间差,相位误差归一化
    Tv_est = Tv;
    epsilon(step) = (floor(trise(step)/delta_TDC)-0.5)*delta_TDC/Tv_est;
    phie(step) =  epsilon(step);
    
    % OFFSET DTC校准,符号LMS,如果chopping后误差还有DC,就说明offset不对,继续推Koft
    Koft = Koft + 2*1e-14*(sign(EN_SWAP(step)-0.5))*sign(phie(step));
    %Koft = 355.9e-12;
    %Koft = 223e-12;

    % DTC增益校准,逼近,让 fractional phase 的正负，对应相位误差对称
    gain_DTC_est = gain_DTC_est  + (-1)*1e-1*sign(phie(step))*sign(PHRF(step));
    %gain_DTC_est = 1.*Tv/DTC_reso;
    
    
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
   
    % DCO quantazation,会产生DCO量化噪声,也会被S域模型拿去算PN
    otw(step+1) = LFout * fref/KDCO;
    otw(step+1) = floor(otw(step+1)*2^NB_dco)/2^NB_dco;
end

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
    xlim([0*Tv*1e6 settle_sample*Tv*1e6])
    set(gca,'YTick',-100:5:100);%This is just to tune the y-axis step, rather than the range.
    set(gca,'fontsize',16)
    set(findobj(gca,'Type','line'),'LineWidth',2)
    set(gcf,'color','w');
end

%% Plot the Inst phase error phie
if 1
    Fig = figure('Color',[1 1 1]);
    %phie = (tckv_div(1:Sim_Time)-mean(tckv_div(1:Sim_Time)))./Tv*2*pi;
    plot(tR_norm(1:Sim_Time)*1e6,phie);
    hold on;grid on;
    xlabel('Time (us)','fontsize',16)
    ylabel('Phase Deviation \phi_e','fontsize',16);
    xlim([0*Tv settle_sample*Tv*1e6])
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
    xlim([0*Tv settle_sample*Tv*1e6]);
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
semilogx(f,10*log10(Scl_tot),'k-');
PN_dcoQ = 1/12.*(KDCO/2^NB_dco./f).^2*1/fref.*sinc(f/fref).^2;
semilogx(f,10*log10(PN_dcoQ.*abs(Hcl_dco.^2)),'b-')

PHE = tckv_div(settle_sample:step_DCO)/Tv*2*pi;
PHE = PHE - mean(PHE);
rbw = 1e3;
fprintf('s-domain:\n');
[Px_sine,f] = fun_calc_psd_dbs(PHE, fv, rbw);
fstep = f(2)-f(1);
f = f(1:floor(length(f)/2));
Px_sine = Px_sine(1:floor(length(Px_sine)/2));
semilogx(f,Px_sine,'g-');
Px_sine_R = 10.^(Px_sine/10);
sum_Y = sum(Px_sine_R(find(f>1e1)))*fstep;
jitter = sqrt(2*sum_Y) / (2*pi*fv);
xlim([1e3,fv/2]);
ylim([-180,-80]);
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

% if 1
%     plot((1:Sim_Time)*Tref*1e6, DCW, 'r', 'LineWidth', 0.5);
% grid on;
% xlabel('Time (us)'); ylabel('DTC Code');
% title('MAIN DTC 控制码序列（时域）');
% xlim([settle_sample*Tv*1e6, (settle_sample+500000)*Tv*1e6]);
% 
% end