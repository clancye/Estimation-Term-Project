//
// Created by clancy on 4/22/16.
//

#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter(TimeType Ts,
                          function<SystemMatrix()> systemMatrixGenerator,
                          function<MeasurementMatrix()> measurementMatrixGenerator,
                          function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator,
                          function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator):
                            _Ts(Ts),
                            _systemMatrixGenerator(systemMatrixGenerator),
                            _measurementMatrixGenerator(measurementMatrixGenerator),
                            _processNoiseCovarianceGenerator(processNoiseCovarianceGenerator),
                            _measurementCovarianceGenerator(measurementCovarianceGenerator) { }

void KalmanFilter::Initialize(MeasurementVector z0,MeasurementVector z1) {
  _x(0) = z1(0);
  double xDot = (z1(0)-z0(0))/_Ts;
  _x(1) = xDot;
  _x(2) = _x(3) = _x(4) = 0;
  _R = _measurementCovarianceGenerator();
  double Rx = _R(0,0);
  _P<< Rx, Rx/_Ts, 0,0,0,
       Rx/_Ts, 2*Rx/(_Ts*_Ts), 0,0,0,
       0,0,0,0,0,
       0,0,0,0,0,
       0,0,0,0,0;
  cout<<"_x = "<<_x<<endl<<endl<<"_P = "<<_P<<endl;
}

pair<StateVector,StateCovarianceMatrix> KalmanFilter::Update(MeasurementVector measurement) {
  _F = _systemMatrixGenerator();//system matrix
  cout<<"HERE"<<endl;
  _Q = _processNoiseCovarianceGenerator();//process noise covariance
  _R = _measurementCovarianceGenerator();//measurement noise covariance
  _H = _measurementMatrixGenerator();//measurement matrix
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
  _x = _F*_x;
  _z = _H*_x;
  _v = z - _z;//actual measurement less predicted
  _x = _x + _W*_v;
}

ofstream& operator<<(ofstream& of,  const KalmanFilter& kf) {
  IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");//Formatting for outputting Eigen matrix
  of << "t = "<<kf._t<<endl;
  of << "x = "<<kf._x.format(OctaveFmt)<<endl;
  of << "P = "<<kf._P.format(OctaveFmt)<<endl;
  of << "W = "<<kf._W.format(OctaveFmt)<<endl;
  return of;
}

