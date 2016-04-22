//
// Created by clancy on 4/20/16.
//

#include "../include/EstimationTPDataGenerator.h"

EstimationTPDataGenerator::EstimationTPDataGenerator(string ID, string filename) {

  StateVector initial;
  function<SystemMatrix(TimeType)> FGenerator;
  pair<double ,double > interval;
  if(ID == _termProjectID) {
    initial <<0, 0, 0, 250, 0;
    interval = make_pair(0.0,500);
    auto turnRates = GenerateTurnRates(ID);
    int turnRateCounter = 0;
    FGenerator = function<SystemMatrix(TimeType)>([=](TimeType t) mutable {
      if(t >= turnRates[turnRateCounter+1].first) {
        turnRateCounter++;
      }
      double Omega = 3.14159265358979*turnRates[turnRateCounter].second/180;//convert to rads
      SystemMatrix F;
      if(Omega != 0) {
        F << 1, sin(Omega) / Omega, 0, -(1 - cos(Omega)) / Omega, 0,
                0, cos(Omega), 0, -sin(Omega), 0,
                0, (1 - cos(Omega)) / Omega, 1, sin(Omega) / Omega, 0,
                0, sin(Omega), 0, cos(Omega), 0,
                0, 0, 0, 0, 1;
      }
      else {
        F << 1, 1, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 1, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
      }
      return F;
    });
  }
  else if(ID == _exampleID) {
    cout<<ID<<endl;

    initial <<0,10;
    interval = make_pair(0.0,100);
    FGenerator = function<SystemMatrix(TimeType)>([](TimeType t)  {
      SystemMatrix F;
        F << 1, 1,
             0, 1;
      return F;
    });
  }
  GenerateData(initial, FGenerator,interval,filename);
}

vector<pair<DataType ,DataType >> EstimationTPDataGenerator::GenerateTurnRates(string ID) {
  vector<pair<DataType ,DataType >> someTurnRates;
    someTurnRates.push_back(make_pair(0,0));//start off straight
    someTurnRates.push_back(make_pair(100,2));//at 100s, turn left 2deg/sec
    someTurnRates.push_back(make_pair(130,0));//at 130s, continue straight
    someTurnRates.push_back(make_pair(200,-1));//at 200s, turn right 1deg/sec
    someTurnRates.push_back(make_pair(245,1));//at 245s, turn left 1deg/sec
    someTurnRates.push_back(make_pair(335,-1));//at 335s, turn right 1deg/sec
    someTurnRates.push_back(make_pair(380,0));//at 380s, continue straight
    someTurnRates.push_back(make_pair(500,0));//repeating to satisfy loop condition below
  return someTurnRates;
}

void EstimationTPDataGenerator::GenerateData(StateVector initial,
                                             function<SystemMatrix(DataType)> FGenerator,
                                             pair<double,double> timeInterval,
                                             string filename)  {

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
