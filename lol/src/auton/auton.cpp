#include "pid/pid.h"
#include "odom/odom.h"
#include "auton/auton.h"
using namespace vex;

//       0
// 270       90
//      180
//x, y, angle, Pure Pursuit, intake, shoot, roller
void BlueMatchAuton(){
  initialize(10.57, 112.72, 180);
  q.push({1, 112.72, 180, false, false, false, 1000}); //go on the roller
  q.push({14, 112.72, 180, false, false, false, 0}); //go back from the roller to the goal
  q.push({14, 112.72, 200, false, false, true, 0});
}

void RedMatchAuton(){
  initialize(138.48, 31.67, 0);
  q.push({141.41, 31.67, 0, false, false, false, 1000}); //go on the roller
  q.push({138.48, 31.67, 0, false, false, false, 0}); //go on the roller
  //q.push({141.2, 72.42, 0, false, false, false, 0}); //go on the roller
}
