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

using namespace std;

template<typename T, typename StateVector, typename SystemMatrix>
void generateData(StateVector initial, SystemMatrix F, pair<T,T> timeInterval, T Ts, vector<pair<T,T>> turnRates, string filename) {
  ofstream outputFile;
  outputFile.open(filename);
  cout<<"test = "<<initial<<endl;
  outputFile<<initial(0)<<","<<initial(1)<<","<<initial(2)<<endl;
  outputFile<<initial(0)<<","<<initial(1)<<","<<initial(2)<<endl;
  outputFile.close();

};

#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPSIMULATOR_H
