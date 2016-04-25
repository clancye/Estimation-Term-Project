//
// Created by clancy on 4/22/16.
//

#include "../include/ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(){ }

ExtendedKalmanFilter::ExtendedKalmanFilter(StateVector sensorState,
                                           double sigmaR,
                                           double sigmaTheta,
                                           TimeType Ts,
                                           function<SystemMatrix(StateVector)> generateSystemMatrix,
                                           MeasurementCovarianceMatrix R,
                                           MeasurementMatrix H,
                                           ProcessNoiseCovarianceMatrix Q,
                                           function<StateVector(StateVector)> predictState):
                                           KalmanFilter(sensorState,sigmaR,sigmaTheta,Ts,generateSystemMatrix,R,H,Q,predictState){ }