//
// Created by clancy on 4/20/16.
//

#ifndef ESTIMATION_PROJECT_2016_ESTIMATIONTPDATAGENERATOR_H
#define ESTIMATION_PROJECT_2016_ESTIMATIONTPDATAGENERATOR_H

#include <map>
#include <string>
#include <utility>
#include <functional>
#include <fstream>
#include <iostream>
#include <vector>

#include "EstimationTPTypeDefinitions.h"

using namespace Eigen;
using namespace std;

class EstimationTPDataGenerator {
  string _termProjectID = string("term project");
  string _exampleID = string("pg218 example");
  SystemMatrix _systemMatrix;
  public:
  EstimationTPDataGenerator(string ID, string filename);

  private:
  vector<pair<DataType ,DataType >> GenerateTurnRates(string ID);
  void GenerateData(StateVector initial,
                    function<SystemMatrix(DataType)> FGenerator,
                    pair<double,double> timeInterval,
                    string filename);
};


#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPDATAGENERATOR_H
