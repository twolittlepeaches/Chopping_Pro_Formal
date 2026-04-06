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
rng(23);

%% Defining parameters
fref = 100e6;
Tref = 1/fref;
f0 = 5.5e9;
T0 = 1/f0;
fcw = 55+1/2^8;
FCW_I = floor(fcw); FCW_F = mod(fcw,1);
PHRF = cumsum(FCW_F*ones(1,Sim_Time));
PHRF = mod(PHRF,1);PHRF((PHRF>=1)) = 0;
OV = [0,(PHRF(2:end)-PHRF(1:end-1))<0];
fv = fcw*fref;
Tv = 1/fv;
delta_TDC = 0.5e-15;
DTC_reso = 0.4e-14;
N_TDC_avg = 128;
KDCO = 10e3;
Mdcorun = 2;
alpha = 2^(-5);
rho = 2^(-12);
%%% DCO 1/f^2 noise
f_offset = 1e6;
S_DCO_offset_dB = -126;
S_DCO_offset = 10^(S_DCO_offset_dB/10);
sigma_w = f_offset/fv*sqrt(Tv)*sqrt(S_DCO_offset);
delta_Tw = sigma_w*randn(round(Sim_Time*fcw*Mdcorun),1);
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

%% Variables initialization
% Variables changing at fref rate
Rr = zeros(round(Sim_Time),1);
otw = zeros(size(Rr));
tR = zeros(size(Rr));
tR_norm = zeros(size(Rr));
Delta_TDEV_fast = zeros(size(Rr));
Delta_TDEV = zeros(size(Rr));
trise = zeros(size(Rr));
Tv_current = zeros(size(Rr));
Tv_est = zeros(size(Rr));
epsilon = zeros(size(Rr));
phie = zeros(size(Rr));
EN_SWAP = 1*randi(2,1,Sim_Time)-1;
PHF_R1 = zeros(size(Rr));
PHF_R2 = zeros(size(Rr));
PHF_R3 = zeros(size(Rr));
t_DTCr = zeros(size(Rr));
t_DTCd = zeros(size(Rr));
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
I_path = 0;
yiir1 = 0; yiir2 = 0; yiir3 = 0; yiir4 = 0;lambda1 = 1/2^0;lambda2 = 1/2^0;lambda3=1/2^0;lambda4=1/2^0;
gain_DTC_est = 0.9*Tv/DTC_reso;
gain_DTC2_est = 0;
gain_DTC3_est = 0;
DTC_DNL = 1e-13*randn(1,floor(6.7*floor(Tv/DTC_reso)));
DTC_INL = cumsum(DTC_DNL);
DTC_INL = 5e-12*sin(3*pi*[1:length(DTC_DNL)]/length(DTC_DNL));
tmp = zeros(size(Rr));
tmp2 = zeros(size(Rr));
tmp3 = zeros(size(Rr));
local_cnt = 0;

Koft = 300e-12;

%% Main Loop
for step = 1:Sim_Time    
    if step == 1  
        Rr(1) = 0 + fcw;        
        Delta_TDEV(1) = T0 - 1/(f0 + otw(1)*KDCO);        
        tR(1) = step * Tref + delta_Tref(1);        
        tR_norm(step) = step * Tref;        
    else        
        Rr(step) = Rr(step-1) + fcw;        
        Delta_TDEV_fast(step) = T0 - 1/(f0 + 1*phie(step-1)*fref/KDCO*KDCO);
        Delta_TDEV(step) = T0 - 1/(f0 + 1*otw(step)*KDCO);    
        tR(step) = step * Tref + delta_Tref(step);        
        tR_norm(step) = step * Tref;        
    end  
    div_cnt = 0;
    while div_cnt < (FCW_I + OV(step))   
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
            if (div_cnt <= -1)
                TDEV(step_DCO) = TDEV(step_DCO - 1) + Delta_TDEV_fast(step);
            else
                TDEV(step_DCO) = TDEV(step_DCO - 1) + Delta_TDEV(step);
            end
            TDEVpn(step_DCO) = TDEVpn(step_DCO-1) + delta_Tj(step_DCO)-delta_Tj(step_DCO-1) + delta_Tw(step_DCO);
            tckv(step_DCO) = step_DCO*T0 -(TDEV(step_DCO)) + TDEVpn(step_DCO);
            tckv_norm(step_DCO) = step_DCO*Tv;  
            tckv_div(step_DCO) = tckv(step_DCO) - step_DCO*Tv;
            tckv_period(step_DCO) =  tckv(step_DCO) - tckv(step_DCO - 1);
            Rv(step_DCO) = Rv(step_DCO-1) + 1;       
        end
        div_cnt = div_cnt + 1;
    end
    
   

    PHRF(step) = PHRF(step)-0.5;
    if EN_SWAP(step)>0.5
        DCW(step) = (PHRF(step)-0.0) * gain_DTC_est;
        t_DTCr(step) = floor(DCW(step))*DTC_reso + 250e-12 + DTC_INL(1+floor(length(DTC_INL)/2+DCW(step)));
        t_DTCd(step) = Koft*(+1);
        trise(step) = 1* (-(tR(step)+t_DTCd(step)) + (tckv(step_DCO)+t_DTCr(step))) ;
    else
        DCW(step) = (0-PHRF(step)-0.0) * gain_DTC_est;
        t_DTCr(step) = floor(DCW(step))*DTC_reso +200e-12 + DTC_INL(1+floor(length(DTC_INL)/2+DCW(step)));
        t_DTCd(step) = 0e-12-Koft*(-1);
        trise(step) =  -(tR(step)+t_DTCr(step)) + (tckv(step_DCO)+t_DTCd(step)) ; 
    end
	tmp(step) = DTC_INL(1+floor(length(DTC_INL)/2+DCW(step)));
    tmp2(step) = Koft;
    tmp3(step) = gain_DTC_est;
    
    
   % trise(step) = trise(step) + (trise(step)/2e-10)^2;
    Tv_est = Tv;
    epsilon(step) = (floor(trise(step)/delta_TDC)-0.5)*delta_TDC/Tv_est;
    phie(step) =  epsilon(step);
    
    Koft = Koft + 2*1e-14*(sign(EN_SWAP(step)-0.5))*sign(phie(step));
    %Koft = 355.9e-12;
    %Koft = 223e-12;
    gain_DTC_est = gain_DTC_est  + (-1)*1e-1*sign(phie(step))*sign(PHRF(step));
    %gain_DTC_est = 1.*Tv/DTC_reso;
    
    
    
	I_path = I_path + phie(step);
    yiir1 = (1-lambda1)*yiir1 + lambda1*(phie(step));
    yiir2 = (1-lambda2)*yiir2 + lambda2*yiir1;
    yiir3 = (1-lambda3)*yiir3 + lambda3*yiir2;
    yiir4 = (1-lambda4)*yiir4 + lambda4*yiir3;
    P_path = alpha*yiir4;
    LFout = P_path+rho*I_path;
   
    otw(step+1) = LFout * fref/KDCO;
    otw(step+1) = floor(otw(step+1)*2^NB_dco)/2^NB_dco;
end

%% Plot the Inst frequency deviation of CKV
if 1 % Determine plot or not.
    fckv_div = 1./tckv_period(1:settle_sample) - fv;
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
rbw = 1e3;fstep = rbw;
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
if 0
    Out_sine = sin(2*pi*tckv(1+10000:round(Sim_Time*fcw-2e4)+10000)/Tv);
    figure;plot(Out_sine);
    [Px_sine,f] = fun_calc_psd_dbs(Out_sine, fv, rbw, fstep);
    Px_sine = fftshift(Px_sine);
    Fig = figure('Color',[1 1 1]);
    plot(f+fv/2,Px_sine-10*log10(2));
    grid on;
    xlabel('Frequency (Hz)','fontsize',16);
    ylabel('Phase noise Spectrum (dBc/Hz)','fontsize',16);
    xlim([fv/2 fv*3/2])
    set(gca,'fontsize',16);
    set(findobj(gca,'Type','line'),'LineWidth',2);
    set(gcf,'color','w');
end