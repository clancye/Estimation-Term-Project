clf
rootPath = "/home/clancy/Projects/Estimation Project 2016/Testing Data/";
immCTData = load(strcat(rootPath,'immCT.txt'));
immLData = load(strcat(rootPath,'immL.txt'));
kfData = load(strcat(rootPath,'kf.txt'));
tpData = load(strcat(rootPath,'Generated Target Trajectories/','Term Project Data.txt'));
measurements = load(strcat(rootPath,'measurements.txt'));
plot(tpData(:,1),tpData(:,3),"-b","linewidth",2);
hold on
plot(immCTData(:,1),immCTData(:,3),"-r","linewidth",2);
hold on
plot(immLData(:,1),immLData(:,3),"-g","linewidth",2);
hold on
plot(kfData(:,1),kfData(:,3),"c","linewidth",2);
hold on
plot(measurements(:,1),measurements(:,2),"-k+","linewidth",2);

