#include "EstimationTPMain.h"

using namespace std;

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/NEWTEST.txt";
 /* cout << "Generating data in file " << filename<<endl;
  string configID("term project");
  EstimationTPDataGenerator generator(configID,filename);*/


  DataType Ts = 1;
  Target myTarget("My Target",filename);
  myTarget.Advance(6);
  StateVector x = myTarget.Sample();

  random_device random_device1,random_device2;
  mt19937 distanceNoiseGenerator(random_device1()), angleNoiseGenerator(random_device2());
  normal_distribution<double> distanceNoise(0,50),angleNoise(0,.01745);//page 488, angle in rads
  auto Distance = function<double(StateVector,StateVector)> (
          [distanceNoise,distanceNoiseGenerator]
          (StateVector sensorState, StateVector targetState) mutable {
            double x0 = sensorState(0), x1 = targetState(0), y0  = sensorState(2), y1 = targetState(2),distance;
            distance = sqrt(pow(x0-x1,2)+pow(y0-y1,2));
            distance += distanceNoise(distanceNoiseGenerator);
            return distance;
  });
  auto Azimuth = function<double(StateVector,StateVector)> (
          [angleNoise,angleNoiseGenerator]
          (StateVector sensorState, StateVector targetState) mutable {
            double x0 = sensorState(0), x1 = targetState(0), y0  = sensorState(2), y1 = targetState(2),azimuth;
            azimuth = atan2(y1-y0,x1-x0);
            azimuth += angleNoise(angleNoiseGenerator);
            return azimuth;
  });
  StateVector targetVec, sensorVec;
  sensorVec<<1,0,0,0,0;
  targetVec<<2,0,0,0,0;
  cout<<Azimuth(sensorVec,targetVec)<<endl;
  cout<<Azimuth(sensorVec,targetVec);

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


  cout<<x;*/


  return 0;
}