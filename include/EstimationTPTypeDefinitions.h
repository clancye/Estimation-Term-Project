//
// Created by clancy on 4/10/16.
//

#ifndef ESTIMATION_PROJECT_2016_ESTIMATIONTPTYPEDEFINITIONS_H
#define ESTIMATION_PROJECT_2016_ESTIMATIONTPTYPEDEFINITIONS_H

#include </usr/include/eigen3/Eigen/Dense>

using namespace Eigen;

#define NUM_FILTERS  2
#define  NUM_STATES  2
#define  NUM_MEASUREMENTS  1
#define  NUM_PROCESS_NOISES  1

typedef double DataType;
typedef double TimeType;
typedef Matrix<DataType, NUM_STATES,1> StateVector;
typedef Matrix<DataType, NUM_MEASUREMENTS,1> MeasurementVector;
typedef Matrix<DataType, NUM_STATES,NUM_STATES> SystemMatrix;
typedef Matrix<DataType, NUM_STATES,NUM_STATES> StateCovarianceMatrix;
typedef Matrix<DataType, NUM_STATES,NUM_STATES> ProcessNoiseCovarianceMatrix;
typedef Matrix<DataType, NUM_MEASUREMENTS,NUM_STATES> MeasurementMatrix;
typedef Matrix<DataType, NUM_MEASUREMENTS,NUM_MEASUREMENTS> MeasurementCovarianceMatrix;
typedef Matrix<DataType, NUM_MEASUREMENTS,NUM_MEASUREMENTS> MeasurementPredictionCovarianceMatrix;
typedef Matrix<DataType, NUM_STATES, NUM_MEASUREMENTS> GainMatrix;
typedef Matrix<DataType, NUM_STATES,NUM_PROCESS_NOISES> NoiseGainMatrix;
typedef Matrix<DataType, NUM_PROCESS_NOISES,NUM_PROCESS_NOISES> VProcessNoiseGainMatrix;



#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPTYPEDEFINITIONS_H
