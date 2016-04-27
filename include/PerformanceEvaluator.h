//
// Created by clancy on 4/27/16.
//

#ifndef ESTIMATION_PROJECT_2016_PERFORMANCEEVALUATOR_H
#define ESTIMATION_PROJECT_2016_PERFORMANCEEVALUATOR_H

#include <fstream>
#include <string>
#include <utility>
#include <memory>
#include <map>
#include <vector>
#include <iostream>

#include "EstimationTPTypeDefinitions.h"

using namespace std;
using SVptr = shared_ptr<StateVector>;
using SCMptr = shared_ptr<StateCovarianceMatrix>;
using MVptr = shared_ptr<MeasurementVector>;

class PerformanceEvaluator {
  string _filename;
  void WriteValuesToFile(map<string,double> performanceValues);
  map<string,double> CalculatePerformanceValues(SVptr xEst, SCMptr P, SVptr xReal);
  public:
  PerformanceEvaluator(string filename);
  void Evaluate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector xReal);
};


#endif //ESTIMATION_PROJECT_2016_PERFORMANCEEVALUATOR_H
