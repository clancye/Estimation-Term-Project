#include "EstimationTPMain.h"

using namespace std;

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/test.txt";
  cout << "Generating data in file " << filename<<endl;
  StateVector initial;
  initial <<1,1,1;
  SystemMatrix F;
  F<< 2,0,0,
      0,2,0,
      0,0,2;
  pair<dataType ,dataType > interval(0.0,500);
  dataType samplingTime = 10;
  vector<pair<dataType ,dataType >> turnRates;
  turnRates.push_back(make_pair(100, 2));
  turnRates.push_back(make_pair(130, 0));
  turnRates.push_back(make_pair(200, -1));
  turnRates.push_back(make_pair(245, 1));
  turnRates.push_back(make_pair(335, -1));
  turnRates.push_back(make_pair(380, 0));
  generateData(initial, F, interval, samplingTime, turnRates,filename);

  return 0;
}