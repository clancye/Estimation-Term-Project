
%   Data is in variables:
%
%   immL
%   immCT
%   kf
%
%   and is accesed by immL.FPOS, for example

load_performance_data;

plot(immCT.RMSPOS,"-b","linewidth",2);
hold on
plot(immL.RMSPOS,"-g","linewidth",2);
hold on
plot(kf.RMSPOS,"-k","linewidth",2);
hold on
plot(RAWRMSPOS,"-c","linewidth",2);
title("RMSPOS");