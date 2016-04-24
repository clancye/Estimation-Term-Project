//
// Created by clancy on 4/22/16.
//

#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter(){ }

KalmanFilter::KalmanFilter(StateVector sensorState,
                          TimeType Ts,
                          SystemMatrix F,
                          VProcessNoiseGainMatrix V,
                          NoiseGainMatrix Gamma,
                          MeasurementCovarianceMatrix R,
                          MeasurementMatrix H,
                          ProcessNoiseCovarianceMatrix Q,
                          function<ProcessNoiseVector()> makeProcessNoise):
                            _sensorState(sensorState),
                            _Ts(Ts),
                            _F(F),
                            _V(V),
                            _Gamma(Gamma),
                            _R(R),
                            _H(H),
                            _Q(Q),
                            _makeProcessNoise(makeProcessNoise){ }

void KalmanFilter::Initialize(MeasurementVector z0, MeasurementVector z1) {
  z0 = ConvertToCartesian(z0);
  z1 = ConvertToCartesian(z1);
  cout<<"z0 = "<<z0<<endl<<"z1 = "<<z1<<endl;
  _x(0) = z1(0);//x position
  double xDot = (z1(0)-z0(0))/_Ts;
  _x(1) = xDot; //x speed
  _x(2) = z1(1);//y position
  double yDot = (z1(1)-z0(1))/_Ts;
  _x(3) = yDot;//y speed
  _x(4) = 0;//omega
  double Rx = _R(0,0);
  double Ry = _R(1,1);
  _P<< Rx,     Rx/_Ts,         0,      0,              0,
          Rx/_Ts, 2*Rx/(_Ts*_Ts), 0,      0,              0,
          0,      0,              Ry,     Ry/_Ts,         0,
          0,      0,              Ry/_Ts, 2*Ry/(_Ts*_Ts), 0,
          0,      0,              0,      0,              0;
  cout <<"_x = "<<_x<<endl<<endl<<"_P = "<<_P<<endl;
}

pair<StateVector,StateCovarianceMatrix> KalmanFilter::Update(MeasurementVector measurement) {
  measurement = ConvertToCartesian(measurement);
  UpdateCovarianceAndGain();
  UpdateStateEstimate(measurement);
  pair<StateVector, StateCovarianceMatrix> estimates = make_pair(_x,_P);
  _t++;
  return estimates;
}

MeasurementVector KalmanFilter::ConvertToCartesian(MeasurementVector z) {
  MeasurementVector z1;
  double r = z(0), theta = z(1);
  z1(0) = r*cos(theta) + _sensorState(0);
  z1(1) = r*sin(theta) + _sensorState(2);
  return z1;
}

void KalmanFilter::UpdateCovarianceAndGain() {
  _P = _F*_P*_F.transpose()+_Q;
  _S = _R + _H*_P*_H.transpose();//measurement prediction covariance
  _W = _P*_H.transpose()*_S.inverse();//gain matrix
  _P = _P - _W*_S*_W.transpose();
}

void KalmanFilter::UpdateStateEstimate(MeasurementVector z) {
  PredictState();
  _z = _H*_x;
  _v = z - _z;//actual measurement less predicted
  _x = _x + _W*_v;
}

void KalmanFilter::PredictState() {
  _processNoise = _makeProcessNoise();
  _x = _F*_x + _Gamma*_processNoise;
}

ofstream& operator<<(ofstream& of,  const KalmanFilter& filter) {
  IOFormat myFormat(StreamPrecision, 0, ", ", ",", "", "", "", "");//Formatting for outputting Eigen matrix
  //of << "t = "<<filter._t<<endl;
  of <<filter._x.format(myFormat)<<endl;
  //of << "P = "<<filter._P.format(OctaveFmt)<<endl;
  //of << "W = "<<filter._W.format(OctaveFmt)<<endl;
  return of;
}

ProcessNoiseVector KalmanFilter::getProcessNoise() {
  return _makeProcessNoise();
}



