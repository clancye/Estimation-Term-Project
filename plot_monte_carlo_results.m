
path = "/home/clancy/Projects/Estimation Project 2016/Performance Data/";
keys = ["FPOS","FVEL","NEES","NORXE","RMSCRS","RMSPOS","RMSSPD","RMSVEL"];

results.FPOS = load(strcat(path,"FPOS.txt"));
results.FVEL = load(strcat(path,"FVEL.txt"));
results.NEES = load(strcat(path,"NEES.txt"));
results.NORXE = load(strcat(path,"NORXE.txt"));
results.RMSCRS = load(strcat(path,"RMSCRS.txt"));
results.RMSPOS = load(strcat(path,"RMSPOS.txt"));
results.RMSSPD = load(strcat(path,"RMSSPD.txt"));
results.RMSVEL = load(strcat(path,"RMSVEL.txt"));
%results.MOD2PR = load(strcat(path,"MOD2PR.txt"));

x = getfield(results,"FPOS");
plot(x);