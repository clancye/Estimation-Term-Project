#include "EstimationTPMain.h"

using namespace std;

vector<function<double(StateVector,StateVector)>> getMeasurementFunctions() {
  vector<function<double(StateVector, StateVector)>> functions;
  random_device random_device1, random_device2;
  mt19937 rangeNoiseGenerator(random_device1()), angleNoiseGenerator(random_device2());
  //normal_distribution<double> rangeNoise(0, 50), angleNoise(0, .01745);//page 488, angle in rads
  normal_distribution<double> rangeNoise(0, 1), angleNoise(0, .01745);//page 218, angle doesn't matter
  auto Range = function<double(StateVector, StateVector)>(
          [rangeNoise, rangeNoiseGenerator]
                  (StateVector sensorState, StateVector targetState) mutable {
            double range = 0;
            int size = sensorState.size();
            for (int i = 0; i < size; i++) {
              if(i%2==0)//filter out the velocity state variables
                range += pow(sensorState(i) - targetState(i), 2);
            }
            range = sqrt(range);
            range += rangeNoise(rangeNoiseGenerator);
            return range;
          });
  functions.push_back(Range);
  StateVector x;
  if (x.size() > 4) {
  auto Azimuth = function<double(StateVector, StateVector)>(
          [angleNoise, angleNoiseGenerator]
                  (StateVector sensorState, StateVector targetState) mutable {
            double x0 = sensorState(0), x1 = targetState(0), y0 = sensorState(2), y1 = targetState(2), azimuth;
            azimuth = atan2(y1 - y0, x1 - x0);
            azimuth += angleNoise(angleNoiseGenerator);
            return azimuth;
          });
    functions.push_back(Azimuth);
  }
  return functions;
}

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

  auto measurementFunctions = getMeasurementFunctions();

  EstimationTPSensor sensor(sensorState,measurementFunctions);



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

  double m1;
  for(int i = 0;i<5;i++) {
    StateVector x = target.Sample();
    m1 = sensor.Measure(target)[0];
    cout << "x = "<<x<<endl<<"m1 = " << m1 << endl<<endl;
    target.Advance();
  }

  //myKalmanFilter.Initialize(initial,p);
  //MeasurementVector z;
  //auto myPair = myKalmanFilter.Update(z);
  //cout<<myPair.first<<endl;



  return 0;
}