#include "EstimationTPMain.h"

using namespace std;

list<function<double(StateVector,StateVector)>> getMeasurementFunctions() {
  list<function<double(StateVector,StateVector)>> functions;
  random_device random_device1,random_device2;
  mt19937 rangeNoiseGenerator(random_device1()), angleNoiseGenerator(random_device2());
  normal_distribution<double> rangeNoise(0,50),angleNoise(0,.01745);//page 488, angle in rads
  auto Range = function<double(StateVector,StateVector)> (
          [rangeNoise,rangeNoiseGenerator]
                  (StateVector sensorState, StateVector targetState) mutable {
            double range = 0;
            int size = sensorState.size();
            for(int i = 0; i<size;i++) {
              range += pow(sensorState(i)-targetState(i),2);
            }
            range = sqrt(range);
            range += rangeNoise(rangeNoiseGenerator);
            return range;
          });
  auto Azimuth = function<double(StateVector,StateVector)> (
          [angleNoise,angleNoiseGenerator]
                  (StateVector sensorState, StateVector targetState) mutable {
            double x0 = sensorState(0), x1 = targetState(0), y0  = sensorState(2), y1 = targetState(2),azimuth;
            azimuth = atan2(y1-y0,x1-x0);
            azimuth += angleNoise(angleNoiseGenerator);
            return azimuth;
          });
  functions.push_back(Range);
  functions.push_back(Azimuth);
  return functions;
}

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/NEWTEST.txt";
 /* cout << "Generating data in file " << filename<<endl;
  string configID("term project");
  EstimationTPDataGenerator generator(configID,filename);*/


  DataType Ts = 1;
  Target target("My Target",filename);

  StateVector sensorState;
  sensorState << -10000,0,0,0,0;

  auto measurementFunctions = getMeasurementFunctions();

  EstimationTPSensor sensor(sensorState,measurementFunctions);
  auto m1 = sensor.Measure(target);
  for(auto x:m1)
    cout<<x<<",";
  cout<<endl;
  target.Advance(160);
  m1 = sensor.Measure(target);
  for(auto x:m1)
    cout<<x<<",";
  cout<<endl;


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