//
// Created by clancy on 4/10/16.
//

#ifndef ESTIMATION_PROJECT_2016_ESTIMATIONTPTYPEDEFINITIONS_H
#define ESTIMATION_PROJECT_2016_ESTIMATIONTPTYPEDEFINITIONS_H

#include </usr/include/eigen3/Eigen/Dense>
#include <functional>

using namespace Eigen;

const int NUM_FILTERS = 2;
const int NUM_STATES = 5;
const int NUM_MEASUREMENTS = 2;
const int NUM_PROCESS_NOISES = 3;

typedef double dataType;
typedef double TimeType;
typedef Matrix<dataType, NUM_STATES,1> StateVector;
typedef Matrix<dataType, NUM_STATES,NUM_STATES> SystemMatrix;
typedef Matrix<dataType, NUM_STATES,NUM_STATES> StateCovarianceMatrix;
typedef Matrix<dataType, NUM_STATES,NUM_STATES> ProcessNoiseCovarianceMatrix;
typedef Matrix<dataType, NUM_MEASUREMENTS,NUM_STATES> MeasurementMatrix;
typedef Matrix<dataType, NUM_MEASUREMENTS,NUM_MEASUREMENTS> MeasurementCovarianceMatrix;



#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPTYPEDEFINITIONS_H
