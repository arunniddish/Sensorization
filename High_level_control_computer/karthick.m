fclose all; close all; clear mex; clear all; clc;

n = (0:(2^10-1)).';

sig = sign(sin(0.01 + 2*pi*1/40*n));
noi = randn(size(n))*sqrt(0.25);
r = sig + noi;

%figure; plot([sig, noi, r]); grid on;
% 
% % tech 1
% rTh = ((r > 0) * 2) - 1;
% figure; plot([rTh]); grid on;
% 
% % tech 2
% rThMa = conv(ones(11,1)/11, rTh); 
% rThMaTh = ((rThMa > 0) * 2) - 1;
% figure; plot([rThMaTh]); grid on;

L = 7;  % preferable to take odd number

% tech 3
rMa = conv(ones(L,1)/L, r);
rMa = rMa((L-1)/2 + n);
rMaTh = ((rMa > 0) * 2) - 1;

figure; plot([sig rMaTh]); grid on;

% tech 4
rWin = buffer(r, L, L-1);
rWinSort = sort(rWin, 1);
rMed = rWinSort( ceil(L/2), :).';
rMedTh = ((rMed > 0) * 2) - 1;

figure; plot([sig rMedTh]); grid on;

dummy = 1;