#include "pid/pid.h"

PID::PID(double k[3]){
  kP = k[0];
  kI = k[1];
  kD = k[2];
}

double PID::get_value(double error){
  //Derivative
  derivative = error - prevError;

  //Integral if its in the activation zone
  if(std::abs(error)<errorActiveZone){
       totalError += error;
  }
  else{
       totalError = 0;
  }

  //Assigns to prevError
  prevError = error;

  if(std::abs(error)<1e-3){
    return 0; //Target reached, stop
  }
  else return (error*kP + totalError*kI + derivative*kD);
}