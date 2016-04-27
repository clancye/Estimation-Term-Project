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
    if(abs(Om)>.05) {
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
    if(abs(Om)>.05) {//don't use the limiting form!
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
  string dataset,filename, configID,performance, path="/home/clancy/Projects/Estimation Project 2016/";
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
  V2 << 7, 0, 0,
          0, 7, 0,
          0, 0, 0;//sigma v
  V3<<1, 0, 0,
      0, 1, 0,
      0, 0, .03;

  vector<double> NORXE_v, FPOS_v, FVEL_v, RMSPOS_v, RMSVEL_v, RMSSPD_v, RMSCRS_v, NEES_v, MOD2PR_v;
  StateVector x;

  performance = path+"performance.txt";
  PerformanceEvaluator pe(performance);

  for(int j = 0;j<NUM_TRIALS;j++) {

    double NORXE = 0, FPOS = 0, FVEL = 0, SPOS = 0, SVEL = 0, SSPD = 0, SCRS = 0, NEES = 0, MOD2PR = 0;

    /*Make the target*/
    Target target(filename);//instantiate the target
    KalmanFilter kf1 = setupKalmanFilter(sensorState, Ts, V1, sigmaR, sigmaTheta);
    KalmanFilter kf2 = setupKalmanFilter(sensorState,Ts,V2,sigmaR,sigmaTheta);
    ExtendedKalmanFilter ekf1 = setupExtendedKalmanFilter(sensorState, Ts, V3, sigmaR, sigmaTheta);

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
    kf1.Initialize(z0, z1);
    ekf1.Initialize(z0, z1);
    kf2.Initialize(z0,z1);
    IMM immCT(kf1, ekf1);
    IMM immL(kf1,kf2);
    for (int i = 0; i < NUM_SAMPLES-1;i++) {
      x = target.Sample();
      z1(0) = range.Measure(target);
      z1(1) = azimuth.Measure(target);
      immCT.Update(z1);
      immL.Update(z1);
      kf2.Update(z1);
      immCTData<<immCT;
      immLData<<immL;
      kfData<<kf2;
      pe.Evaluate(immCT.GetEstimate(),target.Sample());
      /*NORXE += imm.GetNORXE(x);
      FPOS += imm.GetFPOS();
      FVEL += imm.GetFVEL();
      SPOS += imm.GetSPOS(x);
      SVEL += imm.GetSVEL(x);
      SSPD += imm.GetSSPD(x);
      SCRS += imm.GetSCRS(x);
      NEES += imm.GetNEES(x);
      MOD2PR += imm.GetMOD2PR();*/
      target.Advance(10);
    }
    NORXE /= NUM_SAMPLES;
    NORXE_v.push_back(NORXE);

    FPOS /= NUM_SAMPLES;
    FPOS = sqrt(FPOS);
    FPOS_v.push_back(FPOS);

    FVEL /= NUM_SAMPLES;
    FVEL = sqrt(FVEL);
    FVEL_v.push_back(FVEL);

    SPOS /= NUM_SAMPLES;
    SPOS = sqrt(SPOS);
    RMSPOS_v.push_back(SPOS);

    SVEL /= NUM_SAMPLES;
    SVEL = sqrt(SVEL);
    RMSVEL_v.push_back(SVEL);

    SSPD /= NUM_SAMPLES;
    SSPD = sqrt(SSPD);
    RMSSPD_v.push_back(SSPD);

    SCRS /= NUM_SAMPLES;
    SCRS = sqrt(SCRS);
    RMSCRS_v.push_back(SCRS);

    NEES /= NUM_SAMPLES;
    NEES_v.push_back(NEES);

    MOD2PR /= NUM_SAMPLES;
    MOD2PR_v.push_back(MOD2PR);
    immCTData.close();
    immLData.close();
    kfData.close();
  }
  double NORXEavg = average(NORXE_v);
  double FPOSavg = average(FPOS_v);
  double FVELavg = average(FVEL_v);
  double RMSPOSavg = average(RMSPOS_v);
  double RMSVELavg = average(RMSVEL_v);
  double RMSSPDavg = average(RMSVEL_v);
  double RMSCRSavg = average(RMSCRS_v);
  double NEESavg = average(NEES_v);
  double MOD2PRavg = average(MOD2PR_v);

  cout<<"NORXEavg = "<<NORXEavg<<
                "   FPOSavg = "<<FPOSavg<<endl<<
                "   FVELavg = "<<FVELavg<<endl<<
                "   RMSPOSavg = "<<RMSPOSavg<<endl<<
                "   RMSVELavg = "<<RMSVELavg<<endl<<
                "   RMSSPDavg = "<<RMSVELavg<<endl<<
                "   RMSCRSavg = "<<RMSCRSavg<<endl<<
                "   NEESavg = "<<NEESavg<<endl<<
                "   MOD2PRavg = "<<MOD2PRavg<<endl;



  return 0;
}