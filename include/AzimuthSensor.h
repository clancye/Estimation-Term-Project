//
// Created by clancy on 4/22/16.
//

#ifndef ESTIMATION_PROJECT_2016_AZIMUTHSENSOR_H
#define ESTIMATION_PROJECT_2016_AZIMUTHSENSOR_H

#include "Sensor.h"

class AzimuthSensor : public Sensor {
  public:
  AzimuthSensor(StateVector sensorState, double mean, double stddev):Sensor(sensorState,mean,stddev){}
  double Measure(Target& aTarget);
};


#endif //ESTIMATION_PROJECT_2016_AZIMUTHSENSOR_H
