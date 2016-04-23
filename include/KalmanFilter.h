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


using namespace std;

class KalmanFilter {
  TimeType _Ts;//sampling time
  volatile int _t = 0;//actual time in units in which the system advances

  StateVector _x;//state estimate
  MeasurementVector _z, _v;// measurement estimate, and residual
  StateCovarianceMatrix _P;//covariance matrix
  GainMatrix _W;//gain matrix
  SystemMatrix _F;//system matrix
  ProcessNoiseCovarianceMatrix _Q;//noise covariance
  MeasurementCovarianceMatrix _R;//measurement covariance
  MeasurementMatrix _H;//measurement matrix
  MeasurementCovarianceMatrix _S;//measurement prediction covariance

  function<SystemMatrix()> _systemMatrixGenerator;
  function<MeasurementMatrix()> _measurementMatrixGenerator;
  function<ProcessNoiseCovarianceMatrix()> _processNoiseCovarianceGenerator;
  function<MeasurementCovarianceMatrix()> _measurementCovarianceGenerator;

  void UpdateCovarianceAndGain();
  void UpdateStateEstimate(MeasurementVector z);

  public:
  KalmanFilter(){}
  KalmanFilter(TimeType Ts,
               function<SystemMatrix()> systemMatrixGenerator,
               function<MeasurementMatrix()> measurementMatrixGenerator,
               function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator,
               function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator);

  void Initialize(MeasurementVector z0,MeasurementVector z1);
  pair<StateVector,StateCovarianceMatrix> Update(MeasurementVector measurement);

  friend ofstream& operator<<(ofstream& of,const KalmanFilter& kf);
};

#endif //ESTIMATION_PROJECT_2016_KALMANFILTER_H
