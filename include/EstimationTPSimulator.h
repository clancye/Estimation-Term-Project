//
// Created by clancy on 4/10/16.
//

#ifndef ESTIMATION_PROJECT_2016_ESTIMATIONTPSIMULATOR_H
#define ESTIMATION_PROJECT_2016_ESTIMATIONTPSIMULATOR_H

#include <utility>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

template<typename T, typename StateVector, typename SystemMatrix>
void generateData(StateVector initial, function<SystemMatrix(T)> FGenerator, pair<T,T> timeInterval, T Ts, vector<pair<T,T>> turnRates, string filename) {
  ofstream outputFile;
  auto lengthOfTest = timeInterval.second - timeInterval.first;
  int numberOfDataPoints = floor(lengthOfTest / Ts);
  vector<StateVector> data(numberOfDataPoints);
  data[0] = initial;
  outputFile.open(filename);
  auto it = data.begin();
  it++;
  for(it; it!= data.end(); it++) {
    auto vec = *--it;//pre-decrement is essential
    it++;//set the iterator back to its original value;
    *it = (F * vec);//advance the system
    outputFile<<vec(0)<<","<<vec(1)<<","<<vec(2)<<endl;//put the data in the file
  }
  outputFile.close();
};

#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPSIMULATOR_H
