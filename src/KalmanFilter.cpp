//
// Created by clancy on 4/22/16.
//

#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter(){ }

KalmanFilter::KalmanFilter(TimeType Ts,
                          SystemMatrix F,
                          VProcessNoiseGainMatrix V,
                          NoiseGainMatrix Gamma,
                          MeasurementCovarianceMatrix R,
                          MeasurementMatrix H,
                          ProcessNoiseCovarianceMatrix Q,
                          function<ProcessNoiseVector()> makeProcessNoise):
                            _Ts(Ts),
                            _F(F),
                            _V(V),
                            _Gamma(Gamma),
                            _R(R),
                            _H(H),
                            _Q(Q),
                            _makeProcessNoise(makeProcessNoise){ }

void KalmanFilter::Initialize(MeasurementVector z0, MeasurementVector z1) {
  _x(0) = z1(0);//x position
  double xDot = (z1(0)-z0(0))/_Ts;
  _x(1) = xDot; //x speed
  //_x(2) = z1(2);//y position
  //double yDot = (z1(2)-z0(2))/_Ts;
  //_x(3) = yDot;//y speed
  _x(4) = 0;//omega
  double Rx = _R(0,0);
  double Ry = _R(0,0);
  _P<< Rx,     Rx/_Ts,         0,      0,              0,
          Rx/_Ts, 2*Rx/(_Ts*_Ts), 0,      0,              0,
          0,      0,              Ry,     Ry/_Ts,         0,
          0,      0,              Ry/_Ts, 2*Ry/(_Ts*_Ts), 0,
          0,      0,              0,      0,              0;
  cout <<"_x = "<<_x<<endl<<endl<<"_P = "<<_P<<endl;
}

pair<StateVector,StateCovarianceMatrix> KalmanFilter::Update(MeasurementVector measurement) {
  UpdateCovarianceAndGain();
  UpdateStateEstimate(measurement);
  pair<StateVector, StateCovarianceMatrix> estimates = make_pair(_x,_P);
  _t++;
  return estimates;
}

void KalmanFilter::UpdateCovarianceAndGain() {
  _P = _F*_P*_F.transpose()+_Q;
  _S = _R + _H*_P*_H.transpose();//measurement prediction covariance
  _W = _P*_H.transpose()*_S.inverse();//gain matrix
  _P = _P - _W*_S*_W.transpose();
}

void KalmanFilter::UpdateStateEstimate(MeasurementVector z) {
  double v = 3;
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
  IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");//Formatting for outputting Eigen matrix
  of << "t = "<<filter._t<<endl;
  of << "x = "<<filter._x.format(OctaveFmt)<<endl;
  of << "P = "<<filter._P.format(OctaveFmt)<<endl;
  of << "W = "<<filter._W.format(OctaveFmt)<<endl;
  return of;
}

ProcessNoiseVector KalmanFilter::getProcessNoise() {
  return _makeProcessNoise();
}



