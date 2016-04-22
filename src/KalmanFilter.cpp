//
// Created by clancy on 4/22/16.
//

#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter(function<SystemMatrix()> systemMatrixGenerator,
                          function<MeasurementMatrix()> measurementMatrixGenerator,
                          function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator,
                          function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator):
                            _systemMatrixGenerator(systemMatrixGenerator),
                            _measurementMatrixGenerator(measurementMatrixGenerator),
                            _processNoiseCovarianceGenerator(processNoiseCovarianceGenerator),
                            _measurementCovarianceGenerator(measurementCovarianceGenerator) { }

void KalmanFilter::Initialize(StateVector initialState, StateCovarianceMatrix initialCovariance) {
  _x = initialState;
  _P = initialCovariance;
}

pair<StateVector,StateCovarianceMatrix> KalmanFilter::Update(MeasurementVector measurement) {
  _F = _systemMatrixGenerator();//system matrix
  _Q = _processNoiseCovarianceGenerator();//process noise covariance
  _R = _measurementCovarianceGenerator();//measurement noise covariance
  _H = _measurementMatrixGenerator();//measurement matrix
  UpdateCovarianceAndGain();
  UpdateStateEstimate(measurement);
  pair<StateVector, StateCovarianceMatrix> estimates = make_pair(_x,_P);
  _predictions.push_back(estimates);
  return estimates;
}

void KalmanFilter::UpdateCovarianceAndGain() {
  _P = _F*_P*_F.transpose()+_Q;
  _S = _R + _H*_P*_H.transpose();//measurement prediction covarianc
  _W = _P*_H.transpose()*_S.inverse();//gain matrix
  _P = _P - _W*_S*_W.transpose();
}

void KalmanFilter::UpdateStateEstimate(MeasurementVector z) {
  _x = _F*_x;
  _z = _H*_x;
  _v = z - _z;//actual measurement less predicted
  _x = _x + _W*_v;
}

