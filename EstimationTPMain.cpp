#include "EstimationTPMain.h"

using namespace std;

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/NEWTEST.txt";
  cout << "Generating data in file " << filename<<endl;
  string configID("term project");
  EstimationTPDataGenerator generator(configID,filename);


  DataType Ts = 1;


  /* Kalman Filter Stuff
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
    R<<1;
    return R;
  };

  KalmanFilter myKalmanFilter(_systemMatrixGenerator,
  _measurementMatrixGenerator,
  _processNoiseCovarianceGenerator,
  _measurementCovarianceGenerator);


  myKalmanFilter.Initialize(initial,p);
  MeasurementVector z;
  auto myPair = myKalmanFilter.Update(z);
  cout<<myPair.first<<endl;

  Target myTarget("My Target",filename);
  myTarget.Advance(6);
  StateVector x = myTarget.Sample();
  cout<<x;*/


  return 0;
}