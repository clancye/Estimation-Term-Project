//
// Created by clancy on 4/11/16.
//

#ifndef ESTIMATION_PROJECT_2016_KALMANFILTER_H
#define ESTIMATION_PROJECT_2016_KALMANFILTER_H

#include "EstimationTPTypeDefinitions.h"

#include <utility>
#include <vector>
#include <functional>

using namespace std;

class KalmanFilter {
  TimeType _t;//time

  StateVector _x;//state estimate
  MeasurementVector _z, _v;// measurement estimate, and residual
  StateCovarianceMatrix _P;//covariance matrix
  GainMatrix _W;//gain matrix
  SystemMatrix _F;//system matrix
  ProcessNoiseCovarianceMatrix _Q;//noise covariance
  MeasurementCovarianceMatrix _R;//measurement covariance
  MeasurementMatrix _H;//measurement matrix
  MeasurementPredictionCovarianceMatrix _S;//measurement prediction covariance

  function<SystemMatrix()> _systemMatrixGenerator;
  function<MeasurementMatrix()> _measurementMatrixGenerator;
  function<ProcessNoiseCovarianceMatrix()> _processNoiseCovarianceGenerator;
  function<MeasurementCovarianceMatrix()> _measurementCovarianceGenerator;

  vector<pair<StateVector,StateCovarianceMatrix>>  _predictions;

  void UpdateCovarianceAndGain() {
    _P = _F*_P*_F.transpose()+_Q;
    _S = _R + _H*_P*_H.transpose();//measurement prediction covariance
    _W = _P*_H.transpose()*_S.inverse();//gain matrix
    _P = _P - _W*_S*_W.transpose();
  }

  void UpdateStateEstimate(MeasurementVector z) {
    _x = _F*_x;
    _z = _H*_x;
    _v = z - _z;//actual measurement less predicted
    _x = _x + _W*_v;
  }
  public:
  KalmanFilter(){}
  KalmanFilter(function<SystemMatrix()> systemMatrixGenerator,
               function<MeasurementMatrix()> measurementMatrixGenerator,
               function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator,
               function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator):
          _systemMatrixGenerator(systemMatrixGenerator),
          _measurementMatrixGenerator(measurementMatrixGenerator),
          _processNoiseCovarianceGenerator(processNoiseCovarianceGenerator),
          _measurementCovarianceGenerator(measurementCovarianceGenerator) { }

  void Initialize(StateVector initialState, StateCovarianceMatrix initialCovariance) {
    _x = initialState;
    _P = initialCovariance;
  }

  pair<StateVector,StateCovarianceMatrix> Update(MeasurementVector measurement) {
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
};

#endif //ESTIMATION_PROJECT_2016_KALMANFILTER_H
