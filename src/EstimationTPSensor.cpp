//
// Created by clancy on 4/21/16.
//

#include "../include/EstimationTPSensor.h"

vector<double> EstimationTPSensor::Measure(Target& aTarget) {
  vector<double> measurements;
  StateVector sample = aTarget.Sample();
  for_each(_functions.begin(),_functions.end(), [&](function<double(StateVector,StateVector)> fun) {
    measurements.push_back(fun(_location,sample));
  });
  return measurements;
}