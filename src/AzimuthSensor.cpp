//
// Created by clancy on 4/22/16.
//

#include "../include/AzimuthSensor.h"

double AzimuthSensor::Measure(Target& aTarget) {
  StateVector targetState = aTarget.Sample();
  double x0 = _sensorState(0), x1 = targetState(0), y0 = _sensorState(2), y1 = targetState(2), azimuth;
  azimuth = atan2(y1 - y0, x1 - x0);
  double noise = _distribution(_generator);
  azimuth += noise;
  return azimuth;
}