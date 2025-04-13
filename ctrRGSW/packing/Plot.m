clear all;
clc;
close all;

% uUnenc = readmatrix('uUnenc.csv');
% uEnc = readmatrix('uEnc.csv');
% 
% uDiff = uUnenc-uEnc;
% uNorm = vecnorm(uDiff,inf,2);
% save('NaiveSmall.mat','uNorm')
% 
% plot(uNorm)

uDiffSmall = readmatrix('uDiffSmall.csv');
uDiffLarge = readmatrix('uDiffLarge.csv');
plot(uDiffSmall)
hold on;
plot(uDiffLarge)
save('PackSmall.mat','uDiffSmall')
save('PackLarge.mat','uDiffLarge')