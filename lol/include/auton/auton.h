#ifndef AUTONH
#define AUTONH

#include "vex.h" 
#include <cmath> 
#include <vector>
#include <queue>
using namespace vex;

void BlueMatchAuton();
struct robot{
  //Drive Train Values
  double x, y, ang;
  bool PP;
  //Robot Values
  bool intake, bruh;
  double roller;
};
inline std::queue<robot> q;

#endif 