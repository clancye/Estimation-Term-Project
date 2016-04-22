//
// Created by clancy on 4/21/16.
//

#ifndef ESTIMATION_PROJECT_2016_ESTIMATIONTPSENSOR_H
#define ESTIMATION_PROJECT_2016_ESTIMATIONTPSENSOR_H

#include "EstimationTPTypeDefinitions.h"
#include "Target.h"

#include <vector>
#include <algorithm>

using namespace std;

class EstimationTPSensor {
  StateVector _location;
  vector<function<double(StateVector,StateVector)>> _functions;

  public:
  EstimationTPSensor() = delete;
  EstimationTPSensor(StateVector location, vector<function<double(StateVector,StateVector)>> functions):
          _location(location),
          _functions(functions){}
  vector<double> Measure(Target& aTarget);
};


#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPSENSOR_H
