#include "odom/odom.h"
#define sin(r) sin(r / 180 * M_PI)
#define cos(r) cos(r / 180 * M_PI)

void calcWheelVals(double dis, double ang){
  double x = dis * cos(ang); 
  double y = dis * sin(ang);
  IPos[0] += x;
  IPos[1] += y;
  IPos[2]  = 360.00 - InertialSensor.heading(degrees);
}

void initialize(double fx, double fy, double orient){
  IPos[0] = fx;
  IPos[1] = fy;
  IPos[2] = orient;
  InertialSensor.setHeading(360.00 - orient, degrees);
}

double a_tan(double ratio){
    return atan(ratio) * 180 / M_PI;
}

double calc_ang(double x1, double y1, double x2, double y2){
    int x = std::abs(x2-x1);
    int y = std::abs(y2-y1);
    if(x2>x1 && y2>y1){ //1st quadrant
        return (a_tan(y/x));
    }
    if(x2<x1 && y2>y1){ //2nd quadrant
        return 180 - (a_tan(y/x));
    }
    if(x2<x1 && y2<y1){ //3rd quadrant
        return 180 + (a_tan(y/x));
    }
    if(x2>x1 && y2<y1){ //4th quadrant
        return 360 - (a_tan(y/x));
    }
    return 0;
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
