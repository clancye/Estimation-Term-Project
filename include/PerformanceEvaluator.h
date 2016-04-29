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
using VecPtr = shared_ptr<vector<double>>;
using FinishFunction = function<double(VecPtr)>;

class PerformanceEvaluator {
  volatile int _sampleCount = 0;
  string _filename;
  map<string,tuple<VecPtr,PerformanceFunction,function<double(VecPtr)>>> _performanceValueTuples;
  map<string,double> _results;

  void ClearVectors();

  double CalculateNORXE(SVref xEst,SCMref P,SVref xReal);
  double CalculateFPOS(SVref xEst,SCMref P,SVref xReal);
  double CalculateFVEL(SVref xEst,SCMref P,SVref xReal);
  double CalculatePOS(SVref xEst,SCMref P,SVref xReal);
  double CalculateVEL(SVref xEst,SCMref P,SVref xReal);
  double CalculateSPD(SVref xEst,SCMref P,SVref xReal);
  double CalculateCRS(SVref xEst,SCMref P,SVref xReal);
  double CalculateNEES(SVref xEst,SCMref P,SVref xReal);

  double CalculateAverage(VecPtr vec);//mean
  double CalculateRMS(VecPtr vec);//root mean square
  double CalculateRM(VecPtr vec);//root mean

  public:
  PerformanceEvaluator();
  PerformanceEvaluator(string filename);
  void EvaluateIntermediate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector xReal);
  void FinishEvaluating();
  void SetFilename(string filename);

};


#endif //ESTIMATION_PROJECT_2016_PERFORMANCEEVALUATOR_H
