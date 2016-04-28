clf
tpData = load('Term Project Data.txt');
KFData = load('KFData.txt');
EKFData = load('EKFData.txt');
measurements = load('measurements.txt');
plot(tpData(:,1),tpData(:,3),"linewidth",2);
hold on
plot(KFData(:,1),KFData(:,3),"r", "linewidth",2);
hold on
plot(EKFData(:,1),EKFData(:,3),"g", "linewidth",2);
hold on
scatter(measurements(:,1),measurements(:,2),'filled');
