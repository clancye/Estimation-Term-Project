rootPath = "/home/clancy/Projects/Estimation Project 2016/Testing Data/Performance Data/";
immLPath = strcat(rootPath , "immL/");
immCTPath = strcat(rootPath , "immCT/");
kfPath = strcat(rootPath , "kf/");
keys = ["FPOS","FVEL","NEES","NORXE","RMSCRS","RMSPOS","RMSSPD","RMSVEL"];

immL.FPOS = load(strcat(immLPath,"FPOS.txt"));
immL.FVEL = load(strcat(immLPath,"FVEL.txt"));
immL.NEES = load(strcat(immLPath,"NEES.txt"));
immL.NORXE = load(strcat(immLPath,"NORXE.txt"));
immL.RMSCRS = load(strcat(immLPath,"RMSCRS.txt"));
immL.RMSPOS = load(strcat(immLPath,"RMSPOS.txt"));
immL.RMSSPD = load(strcat(immLPath,"RMSSPD.txt"));
immL.RMSVEL = load(strcat(immLPath,"RMSVEL.txt"));
immL.MOD2PR = load(strcat(immLPath,"MOD2PR.txt"));

immCT.FPOS = load(strcat(immCTPath,"FPOS.txt"));
immCT.FVEL = load(strcat(immCTPath,"FVEL.txt"));
immCT.NEES = load(strcat(immCTPath,"NEES.txt"));
immCT.NORXE = load(strcat(immCTPath,"NORXE.txt"));
immCT.RMSCRS = load(strcat(immCTPath,"RMSCRS.txt"));
immCT.RMSPOS = load(strcat(immCTPath,"RMSPOS.txt"));
immCT.RMSSPD = load(strcat(immCTPath,"RMSSPD.txt"));
immCT.RMSVEL = load(strcat(immCTPath,"RMSVEL.txt"));
immCT.MOD2PR = load(strcat(immCTPath,"MOD2PR.txt"));

kf.FPOS = load(strcat(kfPath,"FPOS.txt"));
kf.FVEL = load(strcat(kfPath,"FVEL.txt"));
kf.NEES = load(strcat(kfPath,"NEES.txt"));
kf.NORXE = load(strcat(kfPath,"NORXE.txt"));
kf.RMSCRS = load(strcat(kfPath,"RMSCRS.txt"));
kf.RMSPOS = load(strcat(kfPath,"RMSPOS.txt"));
kf.RMSSPD = load(strcat(kfPath,"RMSSPD.txt"));
kf.RMSVEL = load(strcat(kfPath,"RMSVEL.txt"));

RAWRMSPOS = load(strcat(immCTPath,"RAWRMSPOS.txt"));
