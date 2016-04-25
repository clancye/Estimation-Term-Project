//
// Created by clancy on 4/22/16.
//

#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter(){ }

KalmanFilter::KalmanFilter(StateVector sensorState,
                          double sigmaR,
                          double sigmaTheta,
                          TimeType Ts,
                          function<SystemMatrix(StateVector)> generateSystemMatrix,
                          MeasurementCovarianceMatrix R,
                          MeasurementMatrix H,
                          ProcessNoiseCovarianceMatrix Q,
                          function<StateVector(StateVector)> predictState):
                            _sensorState(sensorState),
                            _sigmaR(sigmaR),
                            _sigmaTheta(sigmaTheta),
                            _Ts(Ts),
                            _generateSystemMatrix(generateSystemMatrix),
                            _R(R),
                            _H(H),
                            _Q(Q),
                            _predictState(predictState){
  _validityConstant = sigmaTheta*sigmaTheta/sigmaR;
  _initialR = _R;
}

void KalmanFilter::Initialize(MeasurementVector z0, MeasurementVector z1) {
  z0 = ConvertToCartesian(z0);
  z1 = ConvertToCartesian(z1);
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
       0,      0,              0,      0,              Rx;
}

pair<StateVector,StateCovarianceMatrix> KalmanFilter::Update(MeasurementVector measurement) {
  measurement = ConvertToCartesian(measurement);
  _F = _generateSystemMatrix(_x);
  UpdateCovarianceAndGain();
  UpdateStateEstimate(measurement);
  pair<StateVector, StateCovarianceMatrix> estimates = make_pair(_x,_P);
  _t++;
  return estimates;
}

MeasurementVector KalmanFilter::ConvertToCartesian(MeasurementVector z) {
  MeasurementVector z1;
  double r = z(0), theta = z(1);
  if((r*_validityConstant)>0.4) {//debiasing
    double b1 = exp(-(_sigmaTheta*_sigmaTheta)/2);
    z1(0) = r*cos(theta)/b1 + _sensorState(0);
    z1(1) = r*sin(theta)/b1 + _sensorState(2);
  }
  else {
    z1(0) = r * cos(theta) + _sensorState(0);
    z1(1) = r * sin(theta) + _sensorState(2);
  }
  return z1;
}

void KalmanFilter::UpdateCovarianceAndGain() {
  _P = _F*_P*_F.transpose()+_Q;
  _S = _R + _H*_P*_H.transpose();//measurement prediction covariance
  _W = _P*_H.transpose()*_S.inverse();//gain matrix
  _P = _P - _W*_S*_W.transpose();
}

void KalmanFilter::UpdateStateEstimate(MeasurementVector z) {
  _x = _predictState(_x);
  _z = _H*_x;
  _v = z - _z;//actual measurement less predicted
  _x = _x + _W*_v;
}

pair<StateVector,StateCovarianceMatrix> KalmanFilter::GetEstimate() {
  return make_pair(_x,_P);
};

void KalmanFilter::Reinitialize(pair<StateVector,StateCovarianceMatrix> params) {
  _x = params.first;
  _P = params.second;
}

double KalmanFilter::GetLikelihood() {
  StateCovarianceMatrix tempMatrix = 2*3.14159265358979*_S;
}

ofstream& operator<<(ofstream& of,  const KalmanFilter& filter) {
  IOFormat myFormat(StreamPrecision, 0, ", ", ",", "", "", "", "");//Formatting for outputting Eigen matrix
  //of << "t = "<<filter._t<<endl;
  of <<filter._x.format(myFormat)<<endl;
  //of << "P = "<<filter._P.format(OctaveFmt)<<endl;
  //of << "W = "<<filter._W.format(OctaveFmt)<<endl;
  return of;
}



