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
#include <functional>
#include <typeinfo>
#include <tuple>

#include "EstimationTPTypeDefinitions.h"

using namespace std;
using SVref = StateVector&;
using SCMref = StateCovarianceMatrix&;
using PerformanceFunction = function<double(SVref,SCMref,SVref)>;
using FinishFunction = function<double(vector<double>)>;

class PerformanceEvaluator {
  volatile int _sampleCount = 0;
  string _filename;
  map<string,tuple<vector<double>,PerformanceFunction,function<double(vector<double>)>>> _performanceValueTuples;
  map<string,double> _results;
  void WriteValuesToFile(map<string,double> performanceValues);
  double CalculateNORXE(SVref xEst,SCMref P,SVref xReal);
  double CalculateFPOS(SVref xEst,SCMref P,SVref xReal);
  double CalculateFVEL(SVref xEst,SCMref P,SVref xReal);
  double CalculatePOS(SVref xEst,SCMref P,SVref xReal);
  void CalculatePerformanceValues(SVref xEst, SCMref P, SVref xReal);
  double CalculateAverage(vector<double> vec);
  double CalculateRMS(vector<double> vec);

  public:
  PerformanceEvaluator(string filename);
  void EvaluateIntermediate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector xReal);
  void FinishEvaluating();

};


#endif //ESTIMATION_PROJECT_2016_PERFORMANCEEVALUATOR_H
