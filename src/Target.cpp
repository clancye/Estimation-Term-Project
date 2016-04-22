//
// Created by clancy on 4/20/16.
//

#include "../include/Target.h"

void Target::Print(const string&& message) {
  cout<<move(message)<<endl;
}

void Target::Print(const string& message) {
  cout<<message<<endl;
}

void Target::Advance(int times) {
  _data = ifstream(_dataFile);
  for(int i = 0;i<times;i++) {
    string nextStateString;
    StateVector nextStateVector;
    if (getline(_data, nextStateString)) {
      istringstream nextStateStream(nextStateString);
      int index = 0;
      while (getline(nextStateStream, nextStateString, ',')) {
        nextStateVector(index) = stod(nextStateString);
        index++;
      }
      _state = nextStateVector;
    }
    else {
      _data.close();
      Print("No more data to read");
    }
  }
}
StateVector Target::Sample() {
  return _state;
}