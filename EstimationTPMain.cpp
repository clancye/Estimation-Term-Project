#include "EstimationTPMain.h"

using namespace std;
KalmanFilter setupKalmanFilter() {
  TimeType Ts = 1;//sampling time
  SystemMatrix F;
  NoiseGainMatrix Gamma;
  MeasurementMatrix H;
  ProcessNoiseCovarianceMatrix Q;
  MeasurementCovarianceMatrix R;
  VProcessNoiseGainMatrix V;
  Gamma <<
  0.5*Ts*Ts, 0,         0,
  Ts,        0,         0,
  0,         0.5*Ts*Ts, 0,
  0,         Ts,        0,
  0,         0,         Ts;

  V << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;//sigma v
  F << 1, Ts, 0, 0, 0,
       0, 1, 0, 0, 0,
       0, 0, 1, Ts, 0,
       0, 0, 0, 1, 0,
       0, 0, 0, 0, 1;
  H << 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0;
  Q = Gamma*V*Gamma.transpose();
  R<<2500, 0,
     0,    .0174533;//variance/standard deviation for page 218
  random_device rd;
  mt19937 generator(rd());
  normal_distribution<double> noiseX(0,V(0,0)), noiseY(0,V(1,1)),noiseOmega(0,V(2,2));
  function<double()> makeNoise = [generator,noiseX,noiseY,noiseOmega] () mutable{
    return noiseX(generator);
  };
  KalmanFilter myKalmanFilter(Ts, F, V, Gamma,R, H, Q,makeNoise);

  return myKalmanFilter;
}
/*
ExtendedKalmanFilter setupExtendedKalmanFilter() {
  DataType Ts = 1;//sampling time
  function<SystemMatrix()> systemMatrixGenerator = [=]() {
    SystemMatrix F;
    F << 1, Ts, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, Ts, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    return F;
  };
  function<MeasurementMatrix()> measurementMatrixGenerator = []() {
    MeasurementMatrix H;
    H << 1, 0, 0, 0, 0;
    return H;
  };
  function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator = [=]() {
    ProcessNoiseCovarianceMatrix Q;
    NoiseGainMatrix Gamma;
    VProcessNoiseGainMatrix V;
    Gamma <<
    0.5*Ts*Ts,
            Ts,
            0,
            0,
            0;

    V << 1;//sigma v

    Q = Gamma*V*Gamma.transpose();
    return Q;
  };
  function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator = []() {
    MeasurementCovarianceMatrix R;
    R<<1;//variance/standard deviation for page 218
    return R;
  };
  function<StateVector(StateVector)> predictState = [systemMatrixGenerator] (StateVector x) {
    SystemMatrix F = systemMatrixGenerator();
    StateVector nextX = F*x;
    return nextX;
  };
  ExtendedKalmanFilter myExtendedKalmanFilter(Ts,
                              systemMatrixGenerator,
                              measurementMatrixGenerator,
                              processNoiseCovarianceGenerator,
                              measurementCovarianceGenerator,
                              predictState);

  return myExtendedKalmanFilter;
}*/
/*
ExtendedKalmanFilter setupExtendedKalmanFilter() {
  DataType Ts = 1;//sampling time
  function<SystemMatrix()> _systemMatrixGenerator = [=]() {
    SystemMatrix F;
    F << 1, Ts, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, Ts, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    return F;
  };
  function<MeasurementMatrix()> _measurementMatrixGenerator = []() {
    MeasurementMatrix H;
    H << 1, 0, 0, 0, 0;
    return H;
  };
  function<ProcessNoiseCovarianceMatrix()> _processNoiseCovarianceGenerator = [=]() {
    ProcessNoiseCovarianceMatrix Q;
    NoiseGainMatrix Gamma;
    VProcessNoiseGainMatrix V;
    Gamma <<
    0.5*Ts*Ts,
            Ts,
            0,
            0,
            0;

    V << 1;//sigma v

    Q = Gamma*V*Gamma.transpose();
    return Q;
  };
  function<MeasurementCovarianceMatrix()> _measurementCovarianceGenerator = []() {
    MeasurementCovarianceMatrix R;
    R<<1;//variance/standard deviation for page 218
    return R;
  };

  ExtendedKalmanFilter myKalmanFilter(Ts,
                              _systemMatrixGenerator,
                              _measurementMatrixGenerator,
                              _processNoiseCovarianceGenerator,
                              _measurementCovarianceGenerator);

  return myExtendedKalmanFilter;
}
*/

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

  RangeSensor range(sensorState,0,1);

  /* Make the Kalman Filter*/
  KalmanFilter myKalmanFilter = setupKalmanFilter();


  MeasurementVector z0,z1;
  z0(0) = range.Measure(target);
  target.Advance();
  z1(0) = range.Measure(target);
  target.Advance();


  ofstream exampleData(path+"exampleKFData.txt");
  myKalmanFilter.Initialize(z0,z1);
  for(int i = 0;i<90;i++) {
    z1(0) = range.Measure(target);
    auto myPair = myKalmanFilter.Update(z1);
    cout<<myKalmanFilter.getNoise()<<endl;
    exampleData <<myKalmanFilter;
    target.Advance();
  }
  exampleData.close();




  return 0;
}