clf
rootPath = "/home/clancy/Projects/Estimation Project 2016/Testing Data/";
immCTData = load(strcat(rootPath,'immCT.txt'));
immLData = load(strcat(rootPath,'immL.txt'));
kfData = load(strcat(rootPath,'kf.txt'));
tpData = load(strcat(rootPath,'Generated Target Trajectories/','Term Project Data.txt'));
measurements = load(strcat(rootPath,'measurements.txt'));

dataSize = size(tpData)/10;
dataSize(1)
sampledTPData = zeros(dataSize(1),4);
j=1;
for i = 0:size(tpData)-1
  if(mod(i,10)==0)
    sampledTPData(j,1) = tpData(i+1,1);
    sampledTPData(j,3) = tpData(i+1,3);
    j = j+1;
  endif
endfor
plot(sampledTPData(:,1),sampledTPData(:,3),"-b+","linewidth",2);
hold on
plot(immCTData(:,1),immCTData(:,3),"-ro","linewidth",2);
hold on
plot(immLData(:,1),immLData(:,3),"-g","linewidth",2);
hold on
plot(kfData(:,1),kfData(:,3),"c","linewidth",2);
hold on
plot(measurements(:,1),measurements(:,2),"-ks","linewidth",2);

