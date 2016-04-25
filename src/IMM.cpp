//
// Created by clancy on 4/24/16.
//

#include "../include/IMM.h"

IMM::IMM(KalmanFilter f1, KalmanFilter f2){
  _filters.push_back(f1);
  _filters.push_back(f2);
  StateVector x;
  StateCovarianceMatrix P;
  x<< 0,0,0,0,0;
  P<<0,0,0,0,0,
     0,0,0,0,0,
     0,0,0,0,0,
     0,0,0,0,0,
     0,0,0,0,0;
  auto aPair = make_pair(x,P);
  _mixed.push_back(aPair);
  _mixed.push_back(aPair);
  _p<<.9,.1,
      .1,.9;
  _muMode<<.5,.5;
}

pair<StateVector,StateCovarianceMatrix> IMM::Update(MeasurementVector z) {
  CalculateMixingProbabilities();
  Mix();
  GetLikelihoods(z);
  UpdateModeProbabilities();
  Estimate();
  return make_pair(_x,_P);
};
/*WORKS*/
void IMM::CalculateNormalizingConstants() {
  _c<<0,0;
  for(int j = 0;j<NUM_FILTERS;j++) {
    for(int i = 0;i<NUM_FILTERS;i++) {
      _c(j) += _p(i,j)*_muMode(i);
    }
  }
}
/*WORKS*/
void IMM::CalculateMixingProbabilities() {
  CalculateNormalizingConstants();
  for(int i = 0;i<NUM_FILTERS;i++) {
    for(int j = 0;j<NUM_FILTERS;j++) {
      _muMix(i,j) = _p(i,j)*_muMode(i)/_c(j);
    }
  }
  cout<<"_muMix = "<<_muMix<<endl;
}
/*WORKS*/
void IMM::Mix() {
  for(int i = 0;i<NUM_FILTERS;i++) {
    _mixed[i].first <<0,0,0,0,0;
    _mixed[i].second<<0,0,0,0,0,
                      0,0,0,0,0,
                      0,0,0,0,0,
                      0,0,0,0,0,
                      0,0,0,0,0;
  }
  MixStateEstimates();
  MixStateCovarianceEstimates();
  for(int j = 0;j<NUM_FILTERS;j++) {
      _filters[j].Reinitialize(_mixed[j]);
  }
}
/*WORKS*/
void IMM::MixStateEstimates() {
  for(int j = 0;j<NUM_FILTERS;j++) {
    for(int i = 0;i<NUM_FILTERS;i++) {
      StateVector xi = _filters[i].GetEstimate().first;
      _mixed[j].first += xi*_muMix(i,j);
    }
  }
}
/*WORKS*/
void IMM::MixStateCovarianceEstimates() {
  for(int j = 0;j<NUM_FILTERS;j++) {
    for(int i = 0;i<NUM_FILTERS;i++) {
      StateVector xi = _filters[i].GetEstimate().first;
      StateCovarianceMatrix Pi = _filters[i].GetEstimate().second;
      StateVector temp = xi - _mixed[j].first;
      _mixed[j].second += _muMix(i,j)*(Pi+temp*temp.transpose());
    }
  }
}

void IMM::GetLikelihoods(MeasurementVector z) {
  for(int i = 0;i<NUM_FILTERS;i++){
    _filters[i].Update(z);
    _Lambda(i) = _filters[i].GetLikelihood();
  }
}

void IMM::UpdateModeProbabilities() {
  double c = 0;
  for(int j = 0;j<NUM_FILTERS;j++) {
    c += _Lambda(j)*_c(j);
  }
  for(int i = 0;i<NUM_FILTERS;i++) {
    _muMode(i) = _Lambda(i)*_c(i)/c;
  }
}

void IMM::Estimate() {
  _x<<0,0,0,0,0;
  _P<<0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0;
  for(int i = 0;i<NUM_FILTERS;i++) {
    StateVector xi = _filters[i].GetEstimate().first;
    _x += xi*_muMode(i);
  }
  for(int i = 0;i<NUM_FILTERS;i++) {
    StateVector xi = _filters[i].GetEstimate().first;
    StateVector temp = xi - _x;
    StateCovarianceMatrix Pi = _filters[i].GetEstimate().second;
    _P += _muMode(i)*(Pi+temp*temp.transpose());
  }
}

ofstream& operator<<(ofstream& of,  const IMM& imm) {
  IOFormat myFormat(StreamPrecision, 0, ", ", ",", "", "", "", "");//Formatting for outputting Eigen matrix
  //of << "t = "<<filter._t<<endl;
  of <<imm._muMode.format(myFormat)<<endl;
  //of << "P = "<<filter._P.format(OctaveFmt)<<endl;
  //of << "W = "<<filter._W.format(OctaveFmt)<<endl;
  return of;
}