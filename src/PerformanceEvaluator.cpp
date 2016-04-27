//
// Created by clancy on 4/27/16.
//

#include "../include/PerformanceEvaluator.h"

PerformanceEvaluator::PerformanceEvaluator(string filename):_filename(filename){}

void PerformanceEvaluator::Evaluate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector x) {
  shared_ptr<StateVector> xEst = make_shared<StateVector>(estimate.first);
  shared_ptr<StateCovarianceMatrix> P = make_shared<StateCovarianceMatrix>(estimate.second);
  shared_ptr<StateVector> xReal = make_shared<StateVector>(x);

  map<string,double> performanceValues = CalculatePerformanceValues(xEst,P,xReal);
  WriteValuesToFile(performanceValues);

}

map<string,double> PerformanceEvaluator::CalculatePerformanceValues(SVptr xEst, SCMptr P, SVptr xReal) {
  map<string,double> performanceValues;
  performanceValues["HI"] = 1.0;
  performanceValues["BYE"] = 2.0;
  for(auto x:performanceValues) {
    cout<<x.first<<" = "<<x.second<<endl;
  }
  return performanceValues;
}

void PerformanceEvaluator::WriteValuesToFile(map<string,double> values) {
  ofstream of(_filename);
  of<<"HELLO THISIS DD "<<endl;
  of.close();
}