//
// Created by clancy on 4/11/16.
//

#ifndef ESTIMATION_PROJECT_2016_KALMANFILTER_H
#define ESTIMATION_PROJECT_2016_KALMANFILTER_H

#include <utility>

class KalmanFilter {
  TimeType _currentTime;
  auto _currentEstimate;
  auto _currentCovariance;
  auto _gainMatrix;
  auto _systemMatrixGenerator;
  auto _measurementMatrixGenerator;
  auto _processNoiseCovarianceGenerator;
  auto _controlGainGenerator;
  auto _measurementCovarianceGenerator;

  vector<StateVector>  _predictions;

  void UpdateCovarianceAndGain() {
    auto R = _measurementCovarianceGenerator();
    auto H = _measurementMatrixGenerator();
    auto S = R + H*_currentCovariance*H.transpose();
    _gainMatrix = _currentCovariance*H.transpose()*S.inverse();
    auto F = _systemMatrixGenerator();
    auto Q = _processNoiseCovarianceGenerator();
    _currentCovariance = F*_currentCovariance*F.transpose()+Q;
  }
  public:
  KalmanFilter(){}
  KalmanFilter(StateVector initialState,
               StateCovarianceMatrix initialCovariance,
               function<SystemMatrix(TimeType)> systemMatrixGenerator,
               function<MeasurementMatrix(TimeType)> measurementMatrixGenerator,
               function<ProcessNoiseCovarianceMatrix(TimeType)> processNoiseCovarianceGenerator,
               function<ControlGainMatrix(TimeType)> controlGainGenerator,
               function<MeasurementCovarianceMatrix(TimeType)> measurementCovarianceGenerator):
          _currentEstimate(initialState),
          _currentCovariance(initialCovariance),
          _systemMatrixGenerator(systemMatrixGenerator),
          _measurementMatrixGenerator(measurementMatrixGenerator),
          _processNoiseCovarianceGenerator(processNoiseCovarianceGenerator),
          _controlGainGenerator(controlGainGenerator),
          _measurementCovarianceGenerator(measurementCovarianceGenerator)
        {
          UpdateCovarianceAndGain();
        }
  pair<StateVector,StateCovarianceMatrix> Update(StateVector measurement, InputVector input) {
    UpdateCovarianceAndGain(_currentTime);
  }
};

#endif //ESTIMATION_PROJECT_2016_KALMANFILTER_H
