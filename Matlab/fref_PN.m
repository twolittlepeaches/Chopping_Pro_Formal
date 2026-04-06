clc;clear;close all;

%filename = './fref.txt';
filename = 'E:\Tym_IC\Projects\ChoppingPLL_pro\Verilog\Chopping_pro__dig_V1\ckv.txt';
% filename = 'E:\Tym_IC\Projects\ChoppingFracDPLL\choppingPLL2025_1p4_AMS\ckv.txt';
formatSpec = '%19f%s%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
t = dataArray{:, 1};
clearvars filename formatSpec fileID dataArray ans;

t=t(16e5:end);

plot(t,'o-');
x=(1:length(t))';
[p,s]=polyfit(x,t,1);
t_fit = polyval(p,x);
hold on;
plot(x,t_fit,'*r-');
Tr_dev = t-t_fit;
figure;
plot(x,Tr_dev,'o-');
xlabel('points');
ylabel('time dev(ps)');
%close all;

fr=1e12/p(1);
phase_error=Tr_dev/1e12*(2*pi*fr);
len=length(phase_error);
phase_error = phase_error-mean(phase_error);
rbw = 1e3;
fftstr = sprintf('len=%d, rbw=%d', len, rbw);
[YdB, f] = fun_calc_psd_Bogdan(phase_error, fr, rbw);
delta_f = f(2)-f(1);
Y = 10.^(YdB/10);
sum_Y = sum(Y(2:end)) * delta_f;
sqrt_sum_Y = sqrt(sum_Y);
rms_jitter = sqrt_sum_Y / (2*pi*fr);
fprintf(2,'Total integrated rms jitter = %.5f ps.\n', rms_jitter*1e12);
figure;
semilogx(f, YdB-10*log10(2), 'g-');
f0str = sprintf('fr=%0.6f GHz,', fr/1e9);
f1str = sprintf('\nTotal integrated rms jitter = %.5f ps',rms_jitter*1e12);
ylim([-260, -60])
title(['CKV clock: ', f0str, f1str,' (', datestr(now), ')'])
xlabel(['Frequency [Hz] (FFT: ', fftstr, ')'])
ylabel('PN [dBc/Hz]')
grid on
zoom on