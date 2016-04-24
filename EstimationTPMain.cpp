#include "EstimationTPMain.h"

using namespace std;
KalmanFilter setupKalmanFilter(StateVector sensorState, TimeType Ts, VProcessNoiseGainMatrix V ) {
  SystemMatrix F;
  NoiseGainMatrix Gamma;
  MeasurementMatrix H;
  ProcessNoiseCovarianceMatrix Q;
  MeasurementCovarianceMatrix R;
  random_device rd;
  mt19937 generatorX(rd()), generatorY(rd()), generatorOmega(rd());
  normal_distribution<double> noiseX(0,V(0,0)), noiseY(0,V(1,1)),noiseOmega(0,V(2,2));

  Gamma <<
  0.5*Ts*Ts, 0,         0,
  Ts,        0,         0,
  0,         0.5*Ts*Ts, 0,
  0,         Ts,        0,
  0,         0,         Ts;


  F << 1, Ts, 0, 0, 0,
       0, 1, 0, 0, 0,
       0, 0, 1, Ts, 0,
       0, 0, 0, 1, 0,
       0, 0, 0, 0, 0;
  function<StateVector(StateVector)> predictState = [=] (StateVector x) mutable{
    ProcessNoiseVector sigmaV;
    sigmaV<< noiseX(generatorX), noiseY(generatorY), noiseOmega(generatorOmega);
    return F*x + Gamma*sigmaV;
  };
  Q = Gamma*V*Gamma.transpose();
  H << 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0;
  R<<2500, 0,
     0,    .0003046;//.0003046 is 1 degree squared in radians

  KalmanFilter myFilter(sensorState, Ts, F, R, H, Q,predictState);

  return myFilter;
}
/*
ExtendedKalmanFilter setupExtendedKalmanFilter(StateVector sensorState) {
  TimeType Ts = 10;//sampling time
  SystemMatrix Fstraight, Fturn;
  NoiseGainMatrix Gamma;
  MeasurementMatrix H;
  ProcessNoiseCovarianceMatrix Q;
  MeasurementCovarianceMatrix R;
  VProcessNoiseGainMatrix V;
  V << 1, 0, 0,
          0, 1, 0,
          0, 0, 0;//sigma v
  random_device rd;
  mt19937 generatorX(rd()), generatorY(rd()), generatorOmega(rd());
  normal_distribution<double> noiseX(0,V(0,0)), noiseY(0,V(1,1)),noiseOmega(0,V(2,2));
  function<ProcessNoiseVector()> makeProcessNoise = [=] () mutable{
    ProcessNoiseVector sigmaV;
    sigmaV<< noiseX(generatorX), noiseY(generatorY), noiseOmega(generatorOmega);
    return sigmaV;
  };
  Gamma <<
  0.5*Ts*Ts, 0,         0,
          Ts,        0,         0,
          0,         0.5*Ts*Ts, 0,
          0,         Ts,        0,
          0,         0,         Ts;


  Fstraight << 1, Ts, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, Ts, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

  Q = Gamma*V*Gamma.transpose();
  H << 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0;
  R<<2500, 0,
          0,    .0003046;//.0003046 is 1 degree squared in radians

  ExtendedKalmanFilter myFilter(sensorState, Ts, Fstraight,R, H, Q,makeProcessNoise);

  return myFilter;
}*/

int main() {
  cout<<"Which data set do you want to use?"<<endl<<"1 - Term Project"<<endl<<"2 - Example from page 218"<<endl<<"3 - test"<<endl;
  string dataset,filename, configID, path="/home/clancy/Projects/Estimation Project 2016/";
  cin >> dataset;
  if(dataset == "1") {
    filename = path + "Term Project Data.txt";
    configID = "term project";//check DataGenerator.h for correct config IDs
  }
  else if(dataset == "2") {
    filename = path + "page218Example.txt";
    configID = "pg218 example";//check DataGenerator.h for correct config IDs
  }
  else if(dataset == "3") {
    filename = path + "test.txt";
    configID = "term project";//check DataGenerator.h for correct config IDs
  }

  /*Generate Data */
  cout << "Generating data in file " << filename<<endl;
  EstimationTPDataGenerator generator(configID,filename);

  /*Make the target*/
  Target target(filename);//instantiate the target

  /*Make the sensors*/
  StateVector sensorState;
  if(dataset == "1")//1 means term project
    sensorState << -10000,0,0,0,0;//for term project
  else
    sensorState<< 0,0,0,0,0;//for example from page 218

  RangeSensor range(sensorState,0,50);//std dev
  AzimuthSensor azimuth(sensorState,0,.01745);//std dev

  /* Make the Kalman Filter*/
  VProcessNoiseGainMatrix V1,V2;
  TimeType Ts = 10;
  V1 << 1, 0, 0,
       0, 1, 0,
       0, 0, 0;//sigma v
  KalmanFilter kf1 = setupKalmanFilter(sensorState,Ts,V1);
  V2<< .1, 0, 0,
       0, .1, 0,
       0, 0, 0;
  KalmanFilter kf2 = setupKalmanFilter(sensorState,Ts,V2);



  MeasurementVector z0,z1;
  z0(0) = range.Measure(target);
  z0(1) = azimuth.Measure(target);
  target.Advance(10);
  z1(0) = range.Measure(target);
  z1(1) = azimuth.Measure(target);
  target.Advance(10);


  ofstream exampleData(path+"exampleKFData.txt");
  kf1.Initialize(z0,z1);
  for(int i = 0;i<48;i++) {
    z1(0) = range.Measure(target);
    z1(1) = azimuth.Measure(target);
    kf1.Update(z1);
    exampleData <<kf1;
    target.Advance(10);
  }
  exampleData.close();




  return 0;
}