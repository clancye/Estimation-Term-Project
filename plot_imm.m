clf
immData = load('imm.txt');
tpData = load('Term Project Data.txt');
measurements = load('measurements.txt');
plot(tpData(:,1),tpData(:,3),"-b","linewidth",2);
hold on
plot(immData(:,1),immData(:,3),"-r","linewidth",2);
hold on
scatter(measurements(:,1),measurements(:,2),'filled');

