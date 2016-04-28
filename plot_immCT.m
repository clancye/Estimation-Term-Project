clf
immCTData = load('immCT.txt');
tpData = load('Term Project Data.txt');
measurements = load('measurements.txt');
plot(tpData(:,1),tpData(:,3),"-b","linewidth",2);
hold on
plot(immCTData(:,1),immCTData(:,3),"-r","linewidth",2);


