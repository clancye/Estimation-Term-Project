//
// Created by clancy on 4/21/16.
//

#ifndef ESTIMATION_PROJECT_2016_ESTIMATIONTPSENSOR_H
#define ESTIMATION_PROJECT_2016_ESTIMATIONTPSENSOR_H

#include "EstimationTPTypeDefinitions.h"
#include "Target.h"

#include <vector>
#include <algorithm>
#include <random>

using namespace std;

class Sensor {
  protected:
  StateVector _sensorState;
  random_device _rd;
  mt19937 _generator;
  normal_distribution<double> _distribution;

  public:
  Sensor(StateVector sensorState, double mean, double variance):
          _sensorState(sensorState),
          _rd(),
          _generator(_rd()),
          _distribution(mean,variance){}
  virtual double Measure(Target& aTarget) = 0;
};


#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPSENSOR_H
