//
// Created by clancy on 4/27/16.
//

#include "../include/PerformanceEvaluator.h"

PerformanceEvaluator::PerformanceEvaluator(string filename):PerformanceEvaluator(){
  SetFilename(filename);
}

PerformanceEvaluator::PerformanceEvaluator(){
  _performanceValueTuples["NORXE"] = make_tuple(make_shared<vector<double>>(),
                                                PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                  return CalculateNORXE(xEst,P,xReal);
                                                  }),
                                                FinishFunction([=](VecPtr vec) {
                                                  return CalculateAverage(vec);
                                                }));
  _performanceValueTuples["FPOS"] = make_tuple(make_shared<vector<double>>(),
                                                PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                  return CalculateFPOS(xEst,P,xReal);
                                                }),
                                                FinishFunction([=](VecPtr vec) {
                                                  return CalculateRM(vec);
                                                }));
  _performanceValueTuples["FVEL"] = make_tuple(make_shared<vector<double>>(),
                                                PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                  return CalculateFVEL(xEst,P,xReal);
                                                }),
                                                FinishFunction([=](VecPtr vec) {
                                                  return CalculateRM(vec);
                                                }));
  _performanceValueTuples["RMSPOS"] = make_tuple(make_shared<vector<double>>(),
                                                PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                  return CalculatePOS(xEst,P,xReal);
                                                }),
                                                FinishFunction([=](VecPtr vec) {
                                                  return CalculateRMS(vec);
                                                }));
  _performanceValueTuples["RMSVEL"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return CalculateVEL(xEst,P,xReal);
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateRMS(vec);
                                                 }));
  _performanceValueTuples["RMSSPD"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return CalculateSPD(xEst,P,xReal);
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateRMS(vec);
                                                 }));
  _performanceValueTuples["RMSCRS"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return CalculateCRS(xEst,P,xReal);
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateRMS(vec);
                                                 }));
  _performanceValueTuples["NEES"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return CalculateNEES(xEst,P,xReal);
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateAverage(vec);
                                                 }));
}

void PerformanceEvaluator::EvaluateIntermediate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector x) {
  SVref xEst = estimate.first;
  SCMref P = estimate.second;
  SVref xReal = x;

  for(auto v:_performanceValueTuples) {
    PerformanceFunction f = get<1>(v.second);
    VecPtr vec = get<0>(v.second);
    vec->push_back(f(xEst,P,xReal));
  }
  _sampleCount++;
}

void PerformanceEvaluator::FinishEvaluating() {
  ofstream of(_filename);
  for(auto v:_performanceValueTuples) {
    string key = v.first;
    FinishFunction f = get<2>(v.second);
    VecPtr vec = get<0>(v.second);
    //cout<<"START " + key<<endl;
    //cout<<"vec = ";
    //for(auto e:*vec) cout<<e<<endl;
    _results[key] = f(vec);
    cout<< key + " = "<<_results[key]<<endl;
    of<<key+"="<<_results[key]<<endl;
  }
  of.close();
  _sampleCount = 0;
}

void PerformanceEvaluator::SetFilename(string filename) {
  _filename = filename;
}

double PerformanceEvaluator::CalculateAverage(VecPtr vec) {
  return accumulate(vec->begin(),vec->end(),0.0)/(1.0*_sampleCount);//multiply by 1.0 to make it a double
}

double PerformanceEvaluator::CalculateRMS(VecPtr vec) {
  return sqrt(accumulate(vec->begin(),vec->end(),0.0,[](double accum, double x) { return accum + x*x;})/(1.0*_sampleCount));
}

double PerformanceEvaluator::CalculateRM(VecPtr vec) {
  return sqrt((accumulate(vec->begin(),vec->end(),0.0))/(1.0*_sampleCount));
}

double PerformanceEvaluator::CalculateNORXE(SVref xEst,SCMref P,SVref xReal) {
  return (xReal(0) - xEst(0))/sqrt(P(0,0));
}

double PerformanceEvaluator::CalculateFPOS(SVref xEst,SCMref P,SVref xReal) {
  return P(0,0)+P(2,2);
}

double PerformanceEvaluator::CalculateFVEL(SVref xEst,SCMref P,SVref xReal) {
  return P(1,1)+P(3,3);
}

double PerformanceEvaluator::CalculatePOS(SVref xEst,SCMref P,SVref xReal) {
  return sqrt(pow(xEst(0)-xReal(0),2) + pow(xEst(2)-xReal(2),2));
}

double PerformanceEvaluator::CalculateVEL(SVref xEst,SCMref P,SVref xReal) {
  return sqrt(pow(xEst(1)-xReal(1),2) + pow(xEst(3)-xReal(3),2));
}

double PerformanceEvaluator::CalculateSPD(SVref xEst,SCMref P,SVref xReal) {
  double SPDEst = sqrt(pow(xEst(1),2) + pow(xEst(3),2));
  double SPDReal = sqrt(pow(xReal(1),2) + pow(xReal(3),2));
  return SPDReal - SPDEst;
}

double PerformanceEvaluator::CalculateCRS(SVref xEst,SCMref P,SVref xReal) {
  double CRSEst = atan2(xEst(3),xEst(1));
  double CRSReal = atan2(xReal(3),xReal(1));
  return CRSReal - CRSEst;
}

double PerformanceEvaluator::CalculateNEES(SVref xEst,SCMref P,SVref xReal) {
  StateVector x = xReal - xEst;
  return x.transpose()*P.inverse()*x;
}