//
// Created by clancy on 4/11/16.
//

#ifndef ESTIMATION_PROJECT_2016_KALMANFILTER_H
#define ESTIMATION_PROJECT_2016_KALMANFILTER_H

#include "Filter.h"

using namespace std;

class KalmanFilter : public Filter {

  void UpdateStateEstimate(MeasurementVector z);

  public:
  KalmanFilter():Filter(){}
  KalmanFilter(TimeType Ts,
               function<SystemMatrix()> systemMatrixGenerator,
               function<MeasurementMatrix()> measurementMatrixGenerator,
               function<ProcessNoiseCovarianceMatrix()> processNoiseCovarianceGenerator,
               function<MeasurementCovarianceMatrix()> measurementCovarianceGenerator);

  pair<StateVector,StateCovarianceMatrix> Update(MeasurementVector measurement);

};

#endif //ESTIMATION_PROJECT_2016_KALMANFILTER_H
