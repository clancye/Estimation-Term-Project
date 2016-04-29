
%   Data is in variables:
%
%   immL
%   immCT
%   kf
%
%   and is accesed by immL.FPOS, for example

load_performance_data;

plot_RMSPOS
plot(immCT.RMSPOS,"-b");
hold on
plot(immL.RMSPOS,"-g");
hold on
plot(kf.RMSPOS,"-k");