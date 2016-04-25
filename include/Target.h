//
// Created by clancy on 4/20/16.
//

#ifndef ESTIMATION_PROJECT_2016_TARGET_H
#define ESTIMATION_PROJECT_2016_TARGET_H

#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include <memory>

#include "EstimationTPTypeDefinitions.h"
using namespace std;

class Target {
  string _dataFile;
  StateVector _state;
  ifstream _data;
  public:
  Target(string dataFile);
  ~Target() {
    _data.close();
  }
  void Advance(int times = 1);
  StateVector Sample();

  private:
  void Print(const string&& message);
  void Print(const string& message);
};


#endif //ESTIMATION_PROJECT_2016_TARGET_H
