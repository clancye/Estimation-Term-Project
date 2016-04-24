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
  TimeType _Ts;//sampling time
  volatile int _t = 0;//actual time in units in which the system advances

  StateVector _x;//state estimate
  MeasurementVector _z, _v;// measurement estimate, and residual
  StateCovarianceMatrix _P;//covariance matrix
  GainMatrix _W;//gain matrix
  SystemMatrix _F;//system matrix
  VProcessNoiseGainMatrix _V;
  NoiseGainMatrix _Gamma;
  ProcessNoiseCovarianceMatrix _Q;//noise covariance
  MeasurementCovarianceMatrix _R;//measurement covariance
  MeasurementMatrix _H;//measurement matrix
  MeasurementCovarianceMatrix _S;//measurement prediction covariance
  function<double()> _makeNoise;

  void UpdateStateEstimate(MeasurementVector z);
  void UpdateCovarianceAndGain();
  void PredictState();
  public:
  KalmanFilter();
  KalmanFilter(TimeType Ts,
              SystemMatrix F,
              VProcessNoiseGainMatrix V,
              NoiseGainMatrix Gamma,
              MeasurementCovarianceMatrix R,
              MeasurementMatrix H,
              ProcessNoiseCovarianceMatrix Q,
              function<double()> makeNoise);

  virtual pair<StateVector,StateCovarianceMatrix> Update(MeasurementVector measurement);
  void Initialize(MeasurementVector z0,MeasurementVector z1);
  double getNoise();
  friend ofstream& operator<<(ofstream& of,const KalmanFilter& kf);

};

#endif //ESTIMATION_PROJECT_2016_KALMANFILTER_H
