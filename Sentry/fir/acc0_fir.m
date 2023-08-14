function Hd = acc0_fir
%ACC0_FIR Returns a discrete-time filter object.

% MATLAB Code
% Generated by MATLAB(R) 9.2 and the Signal Processing Toolbox 7.4.
% Generated on: 07-Jul-2023 20:35:51

% FIR Window Lowpass filter designed using the FIR1 function.

% All frequency values are in Hz.
Fs = 100;  % Sampling Frequency

N    = 200;       % Order
Fc   = 0.5;       % Cutoff Frequency
flag = 'scale';  % Sampling Flag

% Create the window vector for the design algorithm.
win = hamming(N+1);

% Calculate the coefficients using the FIR1 function.
b  = fir1(N, Fc/(Fs/2), 'low', win, flag);
Hd = dfilt.dffir(b);

% [EOF]
