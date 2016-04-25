//
// Created by clancy on 4/24/16.
//

#ifndef ESTIMATION_PROJECT_2016_IMM_H
#define ESTIMATION_PROJECT_2016_IMM_H

#include "EstimationTPTypeDefinitions.h"
#include "KalmanFilter.h"

class IMM {
  TransitionMatrix _p;
  MixProbabilityMatrix _muMix;
  vector<KalmanFilter> _filters;
  ModeProbabilityVector _muMode, _c;
  vector<pair<StateVector, StateCovarianceMatrix>> _mixed;
  LikelihoodVector _Lambda;

  void CalculateMixingProbabilities();
  void CalculateNormalizingConstants();
  void Mix();
  void GetLikelihoods();
  public:
  IMM(KalmanFilter f1, KalmanFilter f2);
};


#endif //ESTIMATION_PROJECT_2016_IMM_H
