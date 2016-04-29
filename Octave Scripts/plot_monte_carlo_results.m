
%   Data is in variables:
%
%   immL
%   immCT
%   kf
%
%   and is accesed by immL.FPOS, for example

load_performance_data;

%%               RMSPOS              %%
clf
plot(immCT.RMSPOS,"-b","linewidth",2);
hold on
plot(immL.RMSPOS,"-g","linewidth",2);
hold on
plot(kf.RMSPOS,"-k","linewidth",2);
hold on
plot(RAWRMSPOS,"-c","linewidth",2);
title("RMSPOS","fontsize",20);
ylabel("m","fontsize",16);
xlabel("Time(10s)","fontsize",16);
pause

%%               RMSSPD              %%
clf
plot(immCT.RMSSPD,"-b","linewidth",2);
hold on
plot(immL.RMSSPD,"-g","linewidth",2);
hold on
plot(kf.RMSSPD,"-k","linewidth",2);
title("RMSSPD","fontsize",20);
ylabel("m/s","fontsize",16);
xlabel("Time(10s)","fontsize",16);
pause

%%               RMSCRS              %%
clf
plot(immCT.RMSCRS,"-b","linewidth",2);
hold on
plot(immL.RMSCRS,"-g","linewidth",2);
hold on
plot(kf.RMSCRS,"-k","linewidth",2);
title("RMSCRS","fontsize",20);
ylabel("rad","fontsize",16);
xlabel("Time(10s)","fontsize",16);
pause

%%               NEES              %%
clf
plot(immCT.NEES,"-b","linewidth",2);
hold on
plot(immL.NEES,"-g","linewidth",2);
hold on
plot(kf.NEES,"-k","linewidth",2);
title("NEES","fontsize",20);
ylabel("m/s","fontsize",16);
xlabel("Time(10s)","fontsize",16);
pause

%%               MOD2PR              %%
clf
plot(immCT.MOD2PR,"-b","linewidth",2);
hold on
plot(immL.MOD2PR,"-g","linewidth",2);
title("MOD2PR","fontsize",20);
ylabel("Prob","fontsize",16);
xlabel("Time(10s)","fontsize",16);
pause