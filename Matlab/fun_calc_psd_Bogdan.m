
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (C) Robert Bogdan Staszewski, 2010-05-03
%%% Version: 1.0
%%%
%%% Program to plot power spectral density.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [XdB, f] = fun_calc_psd_Bogdan(x, fs, rbw, fstep)

% fun_calc_psd(x, rbw, fstep)
% Function to calculate power spectral density
%
% INPUT arguments
% x     :  data vector
% rbw   :  resolution bandwidth [Hz]
% fstep :  FFT frequency bin separation [Hz]
% OUTPUT
% XdB	: spectrum of x [dB]
% f	: frequency vector [Hz]

if nargin<4,   fstep = rbw/1.62;   end;
if nargin<3,   rbw = 100e3;   end;
if nargin<2,   fs = 1;   end;

len = length(x);
nwin = round(fs * 1.62 / rbw);
nfft = round(fs / fstep);
if nwin > len, nwin=len; rbw=fs*1.62/nwin; end
fftstr = sprintf('rbw=%gkHz, fstep=%gkHz, nfft=%d, nwin=%d', ...
                  rbw/1e3, fstep, nfft, nwin);
fprintf(2,'Calculating the PSD: %s ...\n', fftstr);
[X, f] = pwelch(x, hamming(nwin), [], nfft, fs); 
%X = X .* (sinc(f/fs)).^2;	% correct for ZOH
XdB = 10*log10(X);
XdB_sig = max(XdB)-10*log10(2);
fprintf('Signal PSD peak = %.2f dB, 10log(rbw) = %.1f\n', XdB_sig, 10*log10(rbw))
