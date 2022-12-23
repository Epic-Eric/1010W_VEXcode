#ifndef ODOM_H
#define ODOM_H

#include "vex.h" 
#include <cmath> 
#include <vector>
using namespace vex;

//Rotation: net amount of turning
//Orientation: 0-360 of any yaw pitch or roll movement
//Heading: 0-360 of only yaw, usually use this

/*units
distance: inches
rotation: radian
*/

/*formulas
Arc length:
s = rθ
arc length = radius * arc angle in radian

Law of Cos:
c^2 = a^2 + b^2 - 2abCosC

Sin = opposite / adjacent
Cos = adjacent / hypo
Tan = opposite / adjacent

y=mx + b
*/

inline double dis_FS = 5; //distance between forward and side tracking wheel
inline double dis_FI = 8.9; //distance between forward and intake


//-----variables-----//

inline double FTVal = 0, BTVal = 0; //current tracking wheel distances
inline double FWheelPos[3], SWheelPos[3], IPos[3]; //2 tracking wheels' x,y and angle at any moment 


//-----functions-----//
void setWheelVals(double x, double y, double ang, double wheel[3]); //set a specific wheel's vals
void calcWheelVals(double dis, double ang, double wheel[3]); //calc the coord and add on, (angle change for the back wheel, as +90 degrees)
//void calcCenterVals(); //calc the center coord and angle
void initialize(double fx, double fy, double orient);

#endif