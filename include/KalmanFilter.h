//
// Created by clancy on 4/11/16.
//

#ifndef ESTIMATION_PROJECT_2016_KALMANFILTER_H
#define ESTIMATION_PROJECT_2016_KALMANFILTER_H

#include "EstimationTPTypeDefinitions.h"

#include <iostream>
#include <utility>
#include <vector>
#include <functional>
#include <fstream>
#include <random>

using namespace std;

class KalmanFilter {
  protected:
  StateVector _sensorState;
  TimeType _Ts;//sampling time
  volatile int _t = 0;//actual time in units in which the system advances

  StateVector _x;//state estimate
  MeasurementVector _z, _v;// measurement estimate, and residual
  StateCovarianceMatrix _P;//covariance matrix
  GainMatrix _W;//gain matrix
  SystemMatrix _F;//system matrix
  ProcessNoiseVector _processNoise;
  ProcessNoiseCovarianceMatrix _Q;//noise covariance
  MeasurementCovarianceMatrix _R;//measurement covariance
  MeasurementMatrix _H;//measurement matrix
  MeasurementCovarianceMatrix _S;//measurement prediction covariance
  function<StateVector(StateVector)> _predictState;
  function<SystemMatrix(StateVector)> _generateSystemMatrix;

  void UpdateStateEstimate(MeasurementVector z);
  void UpdateCovarianceAndGain();
  MeasurementVector ConvertToCartesian(MeasurementVector z);

  public:
  KalmanFilter();
  KalmanFilter(StateVector sensorState,
              TimeType Ts,
              function<SystemMatrix(StateVector)> generateSystemMatrix,
              MeasurementCovarianceMatrix R,
              MeasurementMatrix H,
              ProcessNoiseCovarianceMatrix Q,
              function<StateVector(StateVector)> predictState);

  virtual pair<StateVector,StateCovarianceMatrix> Update(MeasurementVector measurement);
  void Initialize(MeasurementVector z0,MeasurementVector z1);
  friend ofstream& operator<<(ofstream& of,const KalmanFilter& kf);

};

#endif //ESTIMATION_PROJECT_2016_KALMANFILTER_H
