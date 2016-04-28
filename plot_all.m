clf
immCTData = load('immCT.txt');
immLData = load('immL.txt');
kfData = load('kf.txt');
tpData = load('Term Project Data.txt');
measurements = load('measurements.txt');
plot(tpData(:,1),tpData(:,3),"-b","linewidth",2);
hold on
plot(immCTData(:,1),immCTData(:,3),"-r","linewidth",2);
hold on
plot(immLData(:,1),immLData(:,3),"-g","linewidth",2);
hold on
plot(kfData(:,1),kfData(:,3),"c","linewidth",2);
hold on
plot(measurements(:,1),measurements(:,2),"-k","linewidth",2);

