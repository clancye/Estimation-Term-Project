//
// Created by clancy on 4/22/16.
//

#include "../include/RangeSensor.h"

double RangeSensor::Measure(Target& aTarget) {
  StateVector targetState = aTarget.Sample();
  double range = 0;
  int size = _sensorState.size();
  for (int i = 0; i < size; i++) {
    if(i%2==0)//filter out the velocity state variables
      range += pow(_sensorState(i) - targetState(i), 2);
  }
  range = sqrt(range);
  range += _distribution(_generator);
  return range;
}