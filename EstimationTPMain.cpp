#include "EstimationTPMain.h"

using namespace std;

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/test.txt";
  cout << "Generating data in file " << filename<<endl;
  StateVector initial;
  initial <<1,5,3;
  SystemMatrix F;
  pair<dataType ,dataType > interval(0.0,500);
  dataType samplingTime = 10;
  vector<pair<dataType ,dataType >> turnRates;
  //turnRates.push_back(pair<dataType,dataType>())
  generateData(initial, F, interval, samplingTime, turnRates,filename);


  return 0;
}