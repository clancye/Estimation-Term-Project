//
// Created by clancy on 4/22/16.
//

#ifndef ESTIMATION_PROJECT_2016_EXTENDEDKALMANFILTER_H
#define ESTIMATION_PROJECT_2016_EXTENDEDKALMANFILTER_H

#include "KalmanFilter.h"

class ExtendedKalmanFilter : public KalmanFilter {
  public:
  ExtendedKalmanFilter();
  ExtendedKalmanFilter(StateVector sensorState,
                      double sigmaR,
                      double sigmaTheta,
                      TimeType Ts,
                      function<SystemMatrix(StateVector)> generateSystemMatrix,
                      MeasurementCovarianceMatrix R,
                      MeasurementMatrix H,
                      ProcessNoiseCovarianceMatrix Q,
                      function<StateVector(StateVector)> predictState);


};


#endif //ESTIMATION_PROJECT_2016_EXTENDEDKALMANFILTER_H
