//
// Created by clancy on 4/27/16.
//

#include "../include/PerformanceEvaluator.h"

PerformanceEvaluator::PerformanceEvaluator(string filepath):PerformanceEvaluator(){
  SetFilePath(filepath);
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
                                                  return Square(CalculatePOS(xEst,P,xReal));
                                                }),
                                                FinishFunction([=](VecPtr vec) {
                                                  return CalculateRM(vec);
                                                }));
  _performanceValueTuples["RMSVEL"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return Square(CalculateVEL(xEst,P,xReal));
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateRM(vec);
                                                 }));
  _performanceValueTuples["RMSSPD"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return Square(CalculateSPD(xEst,P,xReal));
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateRM(vec);
                                                 }));
  _performanceValueTuples["RMSCRS"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return Square(CalculateCRS(xEst,P,xReal));
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateRM(vec);
                                                 }));
  _performanceValueTuples["NEES"] = make_tuple(make_shared<vector<double>>(),
                                                 PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                   return CalculateNEES(xEst,P,xReal);
                                                 }),
                                                 FinishFunction([=](VecPtr vec) {
                                                   return CalculateAverage(vec);
                                                 }));
  /*_performanceValueTuples["MOD2PR"] = make_tuple(make_shared<vector<double>>(),
                                               PerformanceFunction([=](SVref xEst,SCMref P,SVref xReal) {
                                                 return CalculateMOD2PR(xEst,P,xReal);
                                               }),
                                               FinishFunction([=](VecPtr vec) {
                                                 return CalculateAverage(vec);
                                               }));*/
}

void PerformanceEvaluator::EvaluateIntermediate(pair<StateVector,StateCovarianceMatrix> estimate, StateVector x) {
  SVref xEst = estimate.first;
  SCMref P = estimate.second;
  SVref xReal = x;

  for(auto v:_performanceValueTuples) {
    PerformanceFunction f = get<1>(v.second);
    VecPtr vec = get<0>(v.second);
    if(_runCount==0)
      vec->push_back(f(xEst,P,xReal));//need to populate vectors first
    else
      (*vec)[_sampleCount] += f(xEst,P,xReal); //then we can perform addition assignment
  }
  _sampleCount++;
}

void PerformanceEvaluator::FinishEvaluatingRun() {
  _runCount++;
  _sampleCount = 0;
}

void PerformanceEvaluator::CalculateFinalResults() {
for(auto x:_performanceValueTuples) {
  auto vec = get<0>(x.second);//get the vector
  auto f = get<2>(x.second);//get the final function
  f(vec);//apply the final operation i.e. compute the rest of RM, RMS, or average
}
}

void PerformanceEvaluator::WriteResultsToFile() {
  cout<<"Writing results to file<----- need to write this code"<<endl;
}

void PerformanceEvaluator::ClearVectors() {
  for(auto v:_performanceValueTuples) {
    VecPtr vec = get<0>(v.second);
    for(auto it = vec->begin();it != vec->end();it++) *it = 0.0;
  }
}

void PerformanceEvaluator::SetFilePath(string filepath) {
  _filepath = filepath;
}

void PerformanceEvaluator::CalculateAverage(VecPtr vec) {
  for_each(vec->begin(),vec->end(),[this](double x) { return x/(1.0*_sampleCount);});//multiply by 1.0 to make it a double
}

void PerformanceEvaluator::CalculateRM(VecPtr vec) {
  CalculateAverage(vec);
  for_each(vec->begin(),vec->end(),[](double x) { return sqrt(x);});
}

double PerformanceEvaluator::CalculateNORXE(SVref xEst,SCMref P,SVref xReal) {
  double NORXE = (xReal(0) - xEst(0))/sqrt(P(0,0));
  return NORXE;
}

double PerformanceEvaluator::CalculateFPOS(SVref xEst,SCMref P,SVref xReal) {
  double FPOS = P(0,0)+P(2,2);
  return FPOS;
}

double PerformanceEvaluator::CalculateFVEL(SVref xEst,SCMref P,SVref xReal) {
  double FVEL = P(1,1)+P(3,3);
  return FVEL;
}

double PerformanceEvaluator::CalculatePOS(SVref xEst,SCMref P,SVref xReal) {
  double xDiff = xEst(0)-xReal(0), yDiff = xEst(2)-xReal(2);
  double POS = sqrt(pow(xDiff,2) + pow(yDiff,2));
  return POS;
}

double PerformanceEvaluator::CalculateVEL(SVref xEst,SCMref P,SVref xReal) {
  double xDiff = xEst(1)-xReal(1), yDiff = xEst(3)-xReal(3);
  double VEL = sqrt(pow(xDiff,2) + pow(yDiff,2));
  return VEL;
}

double PerformanceEvaluator::CalculateSPD(SVref xEst,SCMref P,SVref xReal) {
  double SPDEst = sqrt(pow(xEst(1),2) + pow(xEst(3),2));
  double SPDReal = sqrt(pow(xReal(1),2) + pow(xReal(3),2));
  double SPD = SPDReal - SPDEst;
  return SPD;
}

double PerformanceEvaluator::CalculateCRS(SVref xEst,SCMref P,SVref xReal) {
  double CRSEst = abs(atan2(xEst(3),xEst(1)));
  double CRSReal = abs(atan2(xReal(3),xReal(1)));
  double CRSdiff = CRSReal-CRSEst;
  return CRSdiff;
}

double PerformanceEvaluator::CalculateNEES(SVref xEst,SCMref P,SVref xReal) {
  StateVector x = xReal - xEst;
  double NEES = x.transpose()*P.inverse()*x;
  return NEES;
}

double PerformanceEvaluator::Square(double x){
  return pow(x,2);
}