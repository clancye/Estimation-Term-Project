//
// Created by clancy on 4/22/16.
//

#ifndef ESTIMATION_PROJECT_2016_RANGESENSOR_H
#define ESTIMATION_PROJECT_2016_RANGESENSOR_H

#include "Sensor.h"

class RangeSensor : public Sensor {
  public:
  RangeSensor(StateVector sensorState, double mean, double variance):Sensor(sensorState,mean,variance){}
  double Measure(Target& aTarget);
};


#endif //ESTIMATION_PROJECT_2016_RANGESENSOR_H
