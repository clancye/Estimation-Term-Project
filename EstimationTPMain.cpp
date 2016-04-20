#include "EstimationTPMain.h"

using namespace std;

vector<pair<DataType ,DataType >> GenerateTurnRates() {
  vector<pair<DataType ,DataType >> someTurnRates;
  /*someTurnRates.push_back(make_pair(0,0));//start off straight
  someTurnRates.push_back(make_pair(100,2));//at 100s, turn left 2deg/sec
  someTurnRates.push_back(make_pair(130,0));//at 130s, continue straight
  someTurnRates.push_back(make_pair(200,-1));//at 200s, turn right 1deg/sec
  someTurnRates.push_back(make_pair(245,1));//at 245s, turn left 1deg/sec
  someTurnRates.push_back(make_pair(335,-1));//at 335s, turn right 1deg/sec
  someTurnRates.push_back(make_pair(380,0));//at 380s, continue straight
  someTurnRates.push_back(make_pair(500,0));//repeating to satisfy loop condition below
   */
  someTurnRates.push_back(make_pair(0,0));
  someTurnRates.push_back(make_pair(100,0));
  return someTurnRates;
};

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/page218Example.txt";
  cout << "Generating data in file " << filename<<endl;
  StateVector initial;
  initial <<0,10;
  StateCovarianceMatrix p;
  p<<1,0,
     0,1;
  pair<DataType ,DataType > interval(0.0,100);
  DataType Ts = 1;

  /*Make a vector describing how the turn rate changes*/
  auto turnRates = GenerateTurnRates();
  int turnRateCounter = 0;

  /*Make a lambda that generates the system matrix */
  auto FGenerator = [=](TimeType t) mutable {
    if(t >= turnRates[turnRateCounter+1].first) {
      turnRateCounter++;
    }
    double Omega = 3.14159265358979*turnRates[turnRateCounter].second/180;
    SystemMatrix F;
    if(Omega != 0) {
      F << 1, sin(Omega) / Omega, 0, -(1 - cos(Omega)) / Omega, 0,
              0, cos(Omega), 0, -sin(Omega), 0,
              0, (1 - cos(Omega)) / Omega, 1, sin(Omega) / Omega, 0,
              0, sin(Omega), 0, cos(Omega), 0,
              0, 0, 0, 0, 1;
    }
    else {
      F << 1, Ts,
           0, 1;
    }
    return F;
  };

  /*Generate the data*/
  generateData(initial, function<SystemMatrix(TimeType)>(FGenerator), interval, filename);

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
  cout<<x;


  return 0;
}