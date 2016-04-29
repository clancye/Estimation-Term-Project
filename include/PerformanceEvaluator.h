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
using FinishFunction = function<void(VecPtr)>;

class PerformanceEvaluator {
  volatile int _sampleCount = 0;
  volatile int _runCount = 0;
  int _finalSampleCount;
  string _filepath,_rawPerformancePath;
  map<string,tuple<VecPtr,PerformanceFunction,FinishFunction>> _performanceValueTuples;

  void ClearVectors();
  double Square(double x);

  double CalculateNORXE(SVref xEst,SCMref P,SVref xReal);
  double CalculateFPOS(SVref xEst,SCMref P,SVref xReal);
  double CalculateFVEL(SVref xEst,SCMref P,SVref xReal);
  double CalculatePOS(SVref xEst,SCMref P,SVref xReal);
  double CalculateVEL(SVref xEst,SCMref P,SVref xReal);
  double CalculateSPD(SVref xEst,SCMref P,SVref xReal);
  double CalculateCRS(SVref xEst,SCMref P,SVref xReal);
  double CalculateNEES(SVref xEst,SCMref P,SVref xReal);

  void CalculateAverage(VecPtr vec);//mean
  void CalculateRM(VecPtr vec);//root mean

  public:
  PerformanceEvaluator();
  PerformanceEvaluator(string filename);

  void EvaluateIntermediate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector xReal);
  void FinishEvaluatingRun();
  void CalculateFinalResults();
  void WriteResultsToFile();

  void SetFilePath(string filepath);
  void SetRawPerformancePath(string filepath);

};


#endif //ESTIMATION_PROJECT_2016_PERFORMANCEEVALUATOR_H
