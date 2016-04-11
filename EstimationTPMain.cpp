#include "EstimationTPMain.h"

using namespace std;

int main() {
  string filename = "/home/clancy/Projects/Estimation Project 2016/test.txt";
  cout << "Generating data in file " << filename<<endl;
  StateVector initial;
  initial <<0,0,0,250;
  pair<dataType ,dataType > interval(0.0,500);
  dataType samplingTime = 10;

  /*Make a vector describing how the turn rate changes*/
  vector<pair<dataType ,dataType >> turnRates;
  turnRates.push_back(make_pair(0,0));//start off straight
  turnRates.push_back(make_pair(100,2));//at 100s, turn left 2deg/sec
  turnRates.push_back(make_pair(130,0));//at 130s, continue straight
  turnRates.push_back(make_pair(200,-1));//at 200s, turn right 1deg/sec
  turnRates.push_back(make_pair(245,1));//at 245s, turn left 1deg/sec
  turnRates.push_back(make_pair(335,-1));//at 335s, turn right 1deg/sec
  turnRates.push_back(make_pair(380,0));//at 380s, continue straight
  turnRates.push_back(make_pair(500,0));//repeating to satisfy loop condition below
  int turnRateCounter = 0;

  /*Make a lambda that generates the system matrix */
  auto FGenerator = [=](timeType t) mutable {
    if(t >= turnRates[turnRateCounter+1].first) {
      turnRateCounter++;
    }
    double Omega = 3.14159265358979*turnRates[turnRateCounter].second/180;
    SystemMatrix F;
    if(Omega != 0) {
      F << 1, sin(Omega) / Omega, 0, -(1 - cos(Omega)) / Omega,
              0, cos(Omega), 0, -sin(Omega),
              0, (1 - cos(Omega)) / Omega, 1, sin(Omega) / Omega,
              0, sin(Omega), 0, cos(Omega);
    }
    else {
      F << 1, 1, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 1,
           0, 0, 0, 1;
    }
    return F;
  };

  /*Generate the data*/
  generateData(initial, function<SystemMatrix(timeType)>(FGenerator), interval, filename);

  return 0;
}