//
// Created by clancy on 4/22/16.
//

#include "../include/KalmanFilter.h"

KalmanFilter::KalmanFilter(TimeType Ts,
                          function<SystemMatrix()> systemMatrixGenerator,
                          function<MeasurementMatrix()> measurementMatrixGenerator,
                          function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator,
                          function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator):
                            Filter(Ts,
                            systemMatrixGenerator,
                            measurementMatrixGenerator,
                            processNoiseCovarianceGenerator,
                            measurementCovarianceGenerator){ }


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

void KalmanFilter::UpdateStateEstimate(MeasurementVector z) {
  PredictState();
  _z = _H*_x;
  _v = z - _z;//actual measurement less predicted
  _x = _x + _W*_v;
}



