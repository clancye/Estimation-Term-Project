#include "EstimationTPMain.h"

using namespace std;
KalmanFilter setupKalmanFilter(StateVector sensorState, TimeType Ts, VProcessNoiseGainMatrix V, double sigmaR,double sigmaTheta ) {
  SystemMatrix F;
  NoiseGainMatrix Gamma;
  MeasurementMatrix H;
  ProcessNoiseCovarianceMatrix Q;
  MeasurementCovarianceMatrix R;
  random_device rd;
  mt19937 generatorX(rd()), generatorY(rd());
  normal_distribution<double> noiseX(0,V(0,0)), noiseY(0,V(1,1));

  Gamma <<
  0.5*Ts*Ts, 0,         0,
  Ts,        0,         0,
  0,         0.5*Ts*Ts, 0,
  0,         Ts,        0,
  0,         0,         0;


  F << 1, Ts, 0, 0, 0,
       0, 1, 0, 0, 0,
       0, 0, 1, Ts, 0,
       0, 0, 0, 1, 0,
       0, 0, 0, 0, 0;

  function<SystemMatrix(StateVector)> generateSystemMatrix = [F] (StateVector x) {
    return F;
  };
  function<StateVector(StateVector)> predictState = [=] (StateVector x) mutable{
    ProcessNoiseVector sigmaV;
    sigmaV<< noiseX(generatorX), noiseY(generatorY), 0;
    return F*x + Gamma*sigmaV;
  };
  Q = Gamma*(V*V)*Gamma.transpose();//multiply V twice to get the variances
  H << 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0;
  R<<sigmaR*sigmaR, 0,
     0,    sigmaTheta*sigmaTheta;

  KalmanFilter myFilter(sensorState, sigmaR, sigmaTheta, Ts, generateSystemMatrix, R, H, Q,predictState);

  return myFilter;
}

ExtendedKalmanFilter setupExtendedKalmanFilter(StateVector sensorState, TimeType Ts, VProcessNoiseGainMatrix V, double sigmaR,double sigmaTheta ) {
  SystemMatrix F;
  NoiseGainMatrix Gamma;
  MeasurementMatrix H;
  ProcessNoiseCovarianceMatrix Q;
  MeasurementCovarianceMatrix R;
  random_device rd;
  mt19937 generatorX(rd()), generatorY(rd()), generatorOm(rd());
  normal_distribution<double> noiseX(0,V(0,0)), noiseY(0,V(1,1)),noiseOm(0,V(2,2));

  Gamma <<
          0.5*Ts*Ts, 0,         0,
          Ts,        0,         0,
          0,         0.5*Ts*Ts, 0,
          0,         Ts,        0,
          0,         0,         Ts;


  F <<1, Ts, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, Ts, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;
  /* Calculate Jacobians - CHECKED GOOD*/
  function<StateVector(StateVector)> calculateJacobians = [Ts] (StateVector x) {
    StateVector j;
    double Om = x(4), xDot = x(1), yDot = x(3);// omega, x derivative, y derivative
    double c = cos(Om*Ts), s = sin(Om*Ts);
    j(0) = (c*Ts*xDot/Om) - (s*xDot/(Om*Om)) - (s*Ts*yDot/Om) - ((-1+c)*yDot/(Om*Om));
    j(1) = -s*Ts*xDot - c*Ts*yDot;
    j(2) = (s*Ts*xDot/Om) - ((1-c)*xDot/(Om*Om)) + (c*Ts*yDot/Om) - (s*yDot/(Om*Om));
    j(3) = c*Ts*xDot - s*Ts*yDot;
    j(4) = 1;
    return j;
  };
/*generateSystemMatrix - CHECKED GOOD*/
  function<SystemMatrix(StateVector)> generateSystemMatrix = [=] (StateVector x) mutable {
    double Om = x(4);//Omega
    if(abs(Om)>.0001) {
      double s = sin (Om*Ts), c = cos(Om*Ts);//omega, and the trig terms
      StateVector j = calculateJacobians(x);
      F <<1, s/Om,        0, -(1-c)/Om, j(0),
          0, c,           0, -s,        j(1),
          0, (1-c)/Om,    1, s/Om,      j(2),
          0, s,           0, c,         j(3),
          0, 0,           0, 0,         1;
    }
    else {
      double xDot = x(1), yDot = x(3);
      F << 1, Ts, 0, 0,  -0.5*Ts*Ts*yDot,
           0, 1,  0, 0,  -Ts*yDot,
           0, 0,  1, Ts, 0.5*Ts*Ts*xDot,
           0, 0,  0, 1,  Ts*xDot,
           0, 0,  0, 0,  1;
    }
    return F;
  };
  /*predictState - CHECKED GOOD*/
  function<StateVector(StateVector)> predictState = [=] (StateVector x) mutable{
    ProcessNoiseVector sigmaV;
    double Om = x(4);
    if(abs(Om)>.0001) {//don't use the limiting form!
      double s = sin (Om*Ts), c = cos(Om*Ts);//omega, and the trig terms
      F <<1, s/Om,     0, -(1-c)/Om, 0,
          0, c,        0, -s,        0,
          0, (1-c)/Om, 1, s/Om,      0,
          0, s,        0, c,         0,
          0, 0,        0, 0,         1;
    }
    sigmaV<< noiseX(generatorX), noiseY(generatorY), noiseOm(generatorOm);
    return F*x + Gamma*sigmaV;
  };
  Q = Gamma*(V*V)*Gamma.transpose();//multiply V twice to get the variances
  H << 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0;
  R<<sigmaR*sigmaR, 0,
     0,    sigmaTheta*sigmaTheta;//.0003046 is 1 degree squared in radians

  ExtendedKalmanFilter myFilter(sensorState, sigmaR, sigmaTheta, Ts, generateSystemMatrix, R, H, Q,predictState);

  return myFilter;
}

double average(vector<double> vec) {
  return accumulate(vec.begin(),vec.end(),0.0)/NUM_TRIALS;
}

int main() {
  cout<<"Which data set do you want to use?"<<endl<<"1 - Term Project"<<endl<<"2 - Example from page 218"<<endl<<"3 - test"<<endl;
  string dataset,filename, configID,performance, path="/home/clancy/Projects/Estimation Project 2016/", performancePath = path+"Performance Data/";
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
  /*cout << "Generating data in file " << filename<<endl;
  EstimationTPDataGenerator generator(configID,filename);*/



  /*Make the sensors*/
  StateVector sensorState;
  if(dataset == "1")//1 means term project
    sensorState << -10000,0,0,0,0;//for term project
  else
    sensorState<< 0,0,0,0,0;//for example from page 218

  double sigmaR = 50, sigmaTheta = .01745;

  RangeSensor range(sensorState,0,sigmaR);//std dev
  AzimuthSensor azimuth(sensorState,0,sigmaTheta);//std dev, 1 deg in radians


  /* Make the Kalman Filter*/
  VProcessNoiseGainMatrix V1,V2,V3;//stddev
  TimeType Ts = 10;
  V1 << .2, 0, 0,
       0, .2, 0,
       0, 0, 0;//sigma v
  V2 << 6.0, 0, 0,
          0, 6.0, 0,
          0, 0, 0;//sigma v
  V3<<.1, 0, 0,
      0, .1, 0,
      0, 0, .005;

  StateVector x;

  PerformanceEvaluator pe;
  pe.SetFilePath(performancePath);
  for(int j = 0;j<NUM_TRIALS;j++) {


    /*Make the target*/
    Target target(filename);//instantiate the target
    KalmanFilter kf1 = setupKalmanFilter(sensorState, Ts, V1, sigmaR, sigmaTheta);
    KalmanFilter kf2 = setupKalmanFilter(sensorState,Ts,V2,sigmaR,sigmaTheta);
    ExtendedKalmanFilter ekf1 = setupExtendedKalmanFilter(sensorState, Ts, V3, sigmaR, sigmaTheta);
    /*Get the initial  measurements*/
    MeasurementVector z0, z1;
    z0(0) = range.Measure(target);
    z0(1) = azimuth.Measure(target);
    target.Advance(10);
    z1(0) = range.Measure(target);
    z1(1) = azimuth.Measure(target);
    target.Advance(10);

    ofstream immCTData(path+"immCT.txt");
    ofstream immLData(path+"immL.txt");
    ofstream kfData(path + "kf.txt");
    ofstream measurements(path+"measurements.txt");
    kf1.Initialize(z0, z1);
    ekf1.Initialize(z0, z1);
    kf2.Initialize(z0,z1);
    IMM immCT(kf1, ekf1);
    IMM immL(kf1,kf2);
    for (int i = 0; i < NUM_SAMPLES-1;i++) {
      x = target.Sample();
      z1(0) = range.Measure(target);
      z1(1) = azimuth.Measure(target);
      measurements<<z1(0)*cos(z1(1))-10000<<","<<z1(0)*sin(z1(1))<<endl;
      immCT.Update(z1);
      immL.Update(z1);
      kf2.Update(z1);
      immCTData<<immCT;
      immLData<<immL;
      kfData<<kf2;
      pe.EvaluateIntermediate(immCT.GetEstimate(),target.Sample());
      target.Advance(10);
    }
    pe.FinishEvaluatingRun();
    immCTData.close();
    immLData.close();
    kfData.close();
  }
  pe.CalculateFinalResults();
  pe.WriteResultsToFile();
  return 0;
}