#include "odom/odom.h"
#define sin(r) sin(r / 180 * M_PI)
#define cos(r) cos(r / 180 * M_PI)

void setWheelVals(double x, double y, double ang, double wheel[3]){
  wheel[0] += x;
  wheel[1] += y;
  wheel[2] = ang;
  IPos[0] += x;
  IPos[1] += y;
  IPos[2]  = FWheelPos[2];
}

void calcWheelVals(double dis, double ang, double wheel[3]){
  double x = dis * cos(ang); 
  double y = dis * sin(ang);
  setWheelVals(x, y, ang, wheel);
}

void initialize(double fx, double fy, double orient){
  FWheelPos[0] = fx;
  FWheelPos[1] = fy;
  FWheelPos[2] = orient;
  SWheelPos[0] = fx + dis_FS * cos(orient);
  SWheelPos[1] = fy + dis_FS * sin(orient);
  SWheelPos[2] = (orient+90);
  IPos[0] = fx + dis_FI * cos(orient);
  IPos[1] = fy + dis_FI * sin(orient);
  IPos[2] = orient;
  InertialSensor.setHeading(orient, degrees);
}

/*void calcCenterVals(){
  int m1 = (LWheelPos[1] - RWheelPos[1]) / (LWheelPos[0] - RWheelPos[0]); //m = y1 - y / x1 - x
  int b1 = (LWheelPos[1]) - (m1 * LWheelPos[0]); //y = mx + b ---> b = y - mx
  int m2 = -1/m1; //if 2 lines have to be perpendicular, 2 slopes are negative reciprocals
  int b2 = (BWheelPos[1]) - (m2 * BWheelPos[0]); //y = mx + b ---> b = y - mx
  //m1x + b1 = m2x+ b2 --> linear function, solve for x
  int x = (b2-b1)/(m1-m2); //isolate x
  int y = m1 * x + b1; //substitute x into 1st equation
  centerPos[0] = x;
  centerPos[1] = y;
  centerPos[2] = LWheelPos[2]; //Values assignment
}*/
