#include "EstimationTPMain.h"

using namespace std;

int main() {
  cout<<"Which data set do you want to use?"<<endl<<"1 - Term Project"<<endl<<"2 - Example from page 218"<<endl;
  string dataset,filename;
  cin >> dataset;
  if(dataset == "1") {
    filename = "/home/clancy/Projects/Estimation Project 2016/Term Project Data.txt";
    string configID("term project");//check DataGenerator.h for correct config IDs
  }
  else if(dataset == "2") {
    filename = "/home/clancy/Projects/Estimation Project 2016/page218Example.txt";
    string configID("pg218 example");//check DataGenerator.h for correct config IDs
  }

  /*Uncomment below to generate data */
  /*cout << "Generating data in file " << filename<<endl;
  EstimationTPDataGenerator generator(configID,filename);*/


  DataType Ts = 1;
  Target target("My Target",filename);

  StateVector sensorState;
  if(sensorState.size()==5)
    sensorState << -10000,0,0,0,0;//for term project
  else
    sensorState<< 0,0;//for example from page 218

  RangeSensor range(sensorState,0,1);
  AzimuthSensor azimuth(sensorState,0,0.01745);
  for(int i = 0;i<25;i++) {
    cout << "range = " <<range.Measure(target)<<endl;
    cout << "azimuth = " <<azimuth.Measure(target)<<endl;
    target.Advance(10);
  }



  /* Kalman Filter Stuff*/
  function<SystemMatrix()> _systemMatrixGenerator = [=]() {
    SystemMatrix F;
    F << 1, Ts,
         0, 1;
    return F;
  };
  function<MeasurementMatrix()> _measurementMatrixGenerator = []() {
    MeasurementMatrix H;
    H << 1, 0;
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

  KalmanFilter myKalmanFilter(_systemMatrixGenerator,
  _measurementMatrixGenerator,
  _processNoiseCovarianceGenerator,
  _measurementCovarianceGenerator);


  //myKalmanFilter.Initialize(initial,p);
  //MeasurementVector z;
  //auto myPair = myKalmanFilter.Update(z);
  //cout<<myPair.first<<endl;



  return 0;
}