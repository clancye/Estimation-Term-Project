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
  string _name, _dataFile;
  StateVector _state;
  ifstream _data;
  public:
  Target(const string& name, string dataFile): _name(name), _dataFile(dataFile){ }
  Target(const string&& name, string dataFile): _name(move(name)), _dataFile(dataFile){ }
  ~Target() {
    _data.close();
    Print("Deleting target \"" + _name + "\"");
  }
  void Advance(int times = 1);
  StateVector Sample();

  private:
  void Print(const string&& message);
  void Print(const string& message);
};


#endif //ESTIMATION_PROJECT_2016_TARGET_H
