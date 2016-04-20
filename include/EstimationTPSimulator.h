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
void generateData(StateVector initial, function<SystemMatrix(T)> FGenerator, pair<T,T> timeInterval, string filename) {

  /*initialize time variables */
  int time = 1;
  auto lengthOfTest = timeInterval.second - timeInterval.first;

  /*initialize data vector and iterator*/
  vector<StateVector> data(lengthOfTest);
  data[0] = initial;
  auto it = data.begin();
  it++;

  /*open the file */
  ofstream outputFile;
  outputFile.open(filename);

  /*generate the data*/
  for(it; it!= data.end(); it++) {
    auto vec = *--it;//pre-decrement, not post-decrement
    it++;//set the iterator back to its original value;
    SystemMatrix F(FGenerator(time));
    *it = (F * vec);//advance the system
    for(int i = 0;i<vec.size();i++) {
      outputFile << vec(i);
      if(i!= vec.size()-1) outputFile << ",";
    }
    outputFile<<endl;
    time++;//increment the seconds counter;
  }
  outputFile.close();
};

#endif //ESTIMATION_PROJECT_2016_ESTIMATIONTPSIMULATOR_H
