//
// Created by clancy on 4/24/16.
//

#ifndef ESTIMATION_PROJECT_2016_IMM_H
#define ESTIMATION_PROJECT_2016_IMM_H

#include "EstimationTPTypeDefinitions.h"
#include "KalmanFilter.h"

class IMM {
  StateVector _x;
  StateCovarianceMatrix _P;
  TransitionMatrix _p;
  MixProbabilityMatrix _muMix;
  vector<KalmanFilter> _filters;
  ModeProbabilityVector _muMode, _c;
  vector<pair<StateVector, StateCovarianceMatrix>> _mixed;
  LikelihoodVector _Lambda;

  void CalculateMixingProbabilities();
  void CalculateNormalizingConstants();
  void Mix();
  void MixStateEstimates();
  void MixStateCovarianceEstimates();
  void GetLikelihoods(MeasurementVector z);
  void UpdateModeProbabilities();
  void Estimate();
  public:
  IMM(KalmanFilter f1, KalmanFilter f2);

  pair<StateVector,StateCovarianceMatrix> Update(MeasurementVector z);
  pair<StateVector,StateCovarianceMatrix> GetEstimate();

  double GetNORXE(StateVector x);
  double GetFPOS();
  double GetFVEL();
  double GetSPOS(StateVector x);
  double GetSVEL(StateVector x);
  double GetSSPD(StateVector x);
  double GetSCRS(StateVector x);
  double GetNEES(StateVector x);
  double GetMOD2PR();
  friend ofstream& operator<<(ofstream& of,const IMM& imm);
};


#endif //ESTIMATION_PROJECT_2016_IMM_H
