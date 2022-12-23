#ifndef PID_H
#define PID_H
#include "vex.h" //user-defined header file uses " "
#include <cmath> //built-in library header file uses < >

using namespace vex;

struct PID { //struct = public, class = private, we've got noth' to hide

   //Settings:
   double kP;
   double kI;
   double kD;

   //Calculations:
   double error;  //Target state - Current state = Position
   double prevError = 0.0;  //Position 20ms ago
   double derivative;  //error - prevError = Speed
   double totalError = 0.0;  //The total error collected over time
   double errorActiveZone = 2.0; //1 inches

   PID(double k[3]);
   double get_value(double error);
};

//-------------------DRIVE BASE---------------------------------
inline double kDrive[3] = {0.44, 0.000001, 15}; //0.44 & 0.0000001 & 15//WHY DOESN"T PID VALUE CHANGE UNTIL YOU CHANGE MAIN? 
//--> If not changes to main, still runs (it assumes you had the same program) , but no compiles, otherwise too much time including downloading imports, so it doesn't compile the #include header files unnecessarily!! Have to make changes to main every time :)
inline PID driveVals(kDrive);
inline double kTurn[3] = {0.1155, 0, 0.025};
inline PID turnVals(kTurn);


#endif