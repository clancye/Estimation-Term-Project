#include "EstimationTPMain.h"

using namespace std;

int main() {
  cout<<"Which data set do you want to use?"<<endl<<"1 - Term Project"<<endl<<"2 - Example from page 218"<<endl<<"3 - test"<<endl;
  string dataset,filename, configID;
  cin >> dataset;
  if(dataset == "1") {
    filename = "/home/clancy/Projects/Estimation Project 2016/Term Project Data.txt";
    configID = "term project";//check DataGenerator.h for correct config IDs
  }
  else if(dataset == "2") {
    filename = "/home/clancy/Projects/Estimation Project 2016/page218Example.txt";
    configID = "pg218 example";//check DataGenerator.h for correct config IDs
  }
  else if(dataset == "3") {
    filename = "/home/clancy/Projects/Estimation Project 2016/test.txt";
    configID = "term project";//check DataGenerator.h for correct config IDs
  }

  /*Generate Data */
  cout << "Generating data in file " << filename<<endl;
  EstimationTPDataGenerator generator(configID,filename);

  /*Make the target*/
  DataType Ts = 1;//sampling time
  Target target(filename);//instantiate the target

  /*Make the sensors*/
  StateVector sensorState;
  if(dataset == "1")//1 means term project
    sensorState << -10000,0,0,0,0;//for term project
  else
    sensorState<< 0,0,0,0,0;//for example from page 218

  RangeSensor range(sensorState,0,1);

  /* Instantiate the Kalman Filter*/
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
      Ts;

    V << 0;

    Q = Gamma*V*Gamma.transpose();
    return Q;
  };
  function<MeasurementCovarianceMatrix()> _measurementCovarianceGenerator = []() {
    MeasurementCovarianceMatrix R;
    R<<1;//variance/standard deviation for page 218
    return R;
  };

  KalmanFilter myKalmanFilter(Ts,
                              _systemMatrixGenerator,
                              _measurementMatrixGenerator,
                              _processNoiseCovarianceGenerator,
                              _measurementCovarianceGenerator);

  MeasurementVector z0,z1;
  z0(0) = range.Measure(target);
  target.Advance();
  z1(0) = range.Measure(target);
  target.Advance();

  myKalmanFilter.Initialize(z0,z1);
  //MeasurementVector z;
  //auto myPair = myKalmanFilter.Update(z);
  //cout<<myPair.first<<endl;



  return 0;
}