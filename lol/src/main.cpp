/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftUp               motor         5               
// LeftMiddle           motor         6               
// LeftDown             motor         9               
// RightUp              motor         1               
// RightMiddle          motor         2               
// RightDown            motor         3               
// Controller1          controller                    
// Intake               motor         19              
// Cata                 motor         10              
// TW_forw              encoder       G, H            
// TW_side              encoder       E, F            
// InertialSensor       inertial      16              
// Left_String          digital_out   A               
// Limit                limit         B               
// Right_String         digital_out   C               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"

#include "odom/odom.h"
#include <cmath>
#include "pid/pid.h"
#include "thread"
#include <iostream>
#include <cstdio>
#include "auton/auton.h"
#include "PurePursuit/PurePursuit.h"
#define sin(r) sin(r / 180 * M_PI)
#define cos(r) cos(r / 180 * M_PI) //very important! was a big bug coz radian --> degrees in main necessary
using namespace vex;
competition Competition;

// define your global instances of motors and other devices here
#define calc_distance(rot) rot / 360 * 2.75 * M_PI;
double TW_A_dis, TW_B_dis;
double prev_TW_A_dis, prev_TW_B_dis;
double target_ang;
bool dir = true;
double err=3, ang_accept=1;
double dis = 1, dis_accept = 0.3, prev_dis = -1, powVal;
robot now;

void record(){
  while(true){
    TW_A_dis = calc_distance(TW_forw.rotation(vex::rotationUnits::deg)); 
    TW_B_dis = calc_distance(TW_side.rotation(vex::rotationUnits::deg));
    calcWheelVals(TW_A_dis - prev_TW_A_dis, (360.00 - InertialSensor.heading(degrees))); //to unit cirling it
    calcWheelVals(TW_B_dis - prev_TW_B_dis, std::fmod((360.00 - (InertialSensor.heading(degrees)-90)), 360.00));
     //-90 coz perpendicular
    prev_TW_A_dis = TW_A_dis;
    prev_TW_B_dis = TW_B_dis;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Forward encoder %f inches", TW_A_dis);
    Brain.Screen.newLine();
    Brain.Screen.print("Side encoder %f inches", TW_B_dis);
    Brain.Screen.newLine();
    Brain.Screen.print("IX: %f, IY: %f, IAng: %f", IPos[0], IPos[1], IPos[2]);
    Brain.Screen.newLine();
    Brain.Screen.print("TX: %f, TY: %f, TAng: %f", now.x, now.y, calc_ang(IPos[0], IPos[1], now.x, now.y));
    Brain.Screen.newLine();
    Brain.Screen.print("Dis: %f", dis);
    Brain.Screen.newLine();
    Brain.Screen.print("driveVal: %f", powVal);
    Brain.Screen.newLine();
    Brain.Screen.print("turn error: %f",  err);

    // Controller1.Screen.clearScreen();
    // Controller1.Screen.print(360.00-InertialSensor.heading(degrees));
    // Controller1.Screen.newLine();

    // if(!Limit.pressing()) Cata.spin(forward, 12, volt);
    // else Cata.stop();

    dis = get_dis({now.x, now.y}, {IPos[0], IPos[1]});
    wait(100, msec);
  }
}

void shoot(void){
  Left_String.set(true);
  Right_String.set(true);
}

void drive_motors(directionType dir, double pow, double turnval){
  LeftUp.spin(dir, pow+turnval, volt);
  LeftMiddle.spin(dir, pow+turnval, volt);
  LeftDown.spin(dir, pow+turnval, volt);
  RightUp.spin(dir, pow-turnval, volt);
  RightMiddle.spin(dir, pow-turnval, volt);
  RightDown.spin(dir, pow-turnval, volt);
}

void fire(void){
  while(Limit.pressing()) Cata.spin(forward, 12, volt);
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  InertialSensor.calibrate();
  Cata.setStopping(hold);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  // target_ang = 260;
  // while(true) {
  //     err = target_ang - InertialSensor.heading(degrees);
  //     if(std::abs(err) >= 180){
  //       err = -std::abs(err)/err * 360 - std::abs(err);
  //     }
  //     drive_motors(forward, 0, turnVals.get_value(err));
  //     wait(20, msec);
  //   }
  //Red / Blue with roller
  // LeftUp.spinFor(forward, 70, degrees, false);
  // RightUp.spinFor(forward, 70, degrees, false);
  // LeftMiddle.spinFor(forward, 70, degrees, false);
  // RightMiddle.spinFor(forward, 70, degrees, false);
  // LeftDown.spinFor(forward, 70, degrees, false);
  // RightDown.spinFor(forward, 70, degrees, false);
  // vex::task::sleep(100);
  // Intake.spinFor(forward, 170, degrees);
  // vex::task::sleep(100);
  // LeftUp.spinFor(reverse, 100, degrees, false);
  // RightUp.spinFor(reverse, 100, degrees, false);
  // LeftMiddle.spinFor(reverse, 100, degrees, false);
  // RightDown.spinFor(reverse, 100, degrees, false);
  // LeftDown.spinFor(reverse, 100, degrees, false);
  // RightMiddle.spinFor(reverse, 100, degrees, true);
  // vex::task::sleep(100);
  // while(!Limit.pressing()) Cata.spin(forward, 12, volt);
  // Cata.stop();
  // Cata.setStopping(hold);
  // LeftUp.spinFor(reverse, 300, degrees, false);
  // LeftMiddle.spinFor(reverse, 300, degrees, false);
  // LeftDown.spinFor(reverse, 300, degrees, false);
  // RightUp.spinFor(reverse, -300, degrees, false);
  // RightDown.spinFor(reverse, -300, degrees, false);
  // RightMiddle.spinFor(reverse, -300, degrees, true);
  // vex::task::sleep(100);
  // Intake.spin(forward, 12, volt);
  // LeftUp.spinFor(forward, 1000, degrees, false);
  // LeftMiddle.spinFor(forward, 1000, degrees, false);
  // LeftDown.spinFor(forward, 1000, degrees, false);
  // RightUp.spinFor(forward, 1000, degrees, false);
  // RightDown.spinFor(forward, 1000, degrees, false);
  // RightMiddle.spinFor(forward, 1000, degrees, false);
  // vex::task::sleep(100);

  //Red / Blue Without Roller
//   LeftUp.spinFor(forward, -500, degrees, false);
//   RightUp.spinFor(forward, -500, degrees, false);
//   LeftMiddle.spinFor(forward, -500, degrees, false);
//   RightMiddle.spinFor(forward, -500, degrees, false);
//   LeftDown.spinFor(forward, -500, degrees, false);
//   RightDown.spinFor(forward, -500, degrees, true);
//   vex::task::sleep(100);
//   RightUp.spinFor(forward, -700, degrees, false);
//   RightMiddle.spinFor(forward, -700, degrees, false);
//   RightDown.spinFor(forward, -700, degrees, false);
//   LeftUp.spinFor(forward, 700, degrees, false);
//   LeftMiddle.spinFor(forward, 700, degrees, false);
//   LeftDown.spinFor(forward, 700, degrees, true);
//   vex::task::sleep(100);
//   LeftUp.spinFor(forward, -800, degrees, false);
//   RightUp.spinFor(forward, -800, degrees, false);
//   LeftMiddle.spinFor(forward, -800, degrees, false);
//   RightMiddle.spinFor(forward, -800, degrees, false);
//   LeftDown.spinFor(forward, -800, degrees, false);
//   RightDown.spinFor(forward, -800, degrees, true);
//   vex::task::sleep(100);
//   RightUp.spinFor(forward, 700, degrees, false);
//   RightMiddle.spinFor(forward, 700, degrees, false);
//   RightDown.spinFor(forward, 700, degrees, true);
//   LeftUp.spinFor(forward, -700, degrees, false);
//   LeftMiddle.spinFor(forward, -700, degrees, false);
//   LeftDown.spinFor(forward, -700, degrees, true);
//   vex::task::sleep(100);
//   LeftUp.spinFor(forward, 650, degrees, false);
//   RightUp.spinFor(forward, 650, degrees, false);
//   LeftMiddle.spinFor(forward, 650, degrees, false);
//   RightMiddle.spinFor(forward, 650, degrees, false);
//   LeftDown.spinFor(forward, 650, degrees, false);
//   RightDown.spinFor(forward, 650, degrees, true);
//   vex::task::sleep(100);
//   Intake.spinFor(forward, 170, degrees, true);
//   vex::task::sleep(100);
//   LeftUp.spinFor(forward, -750, degrees, false);
//   RightUp.spinFor(forward, -850, degrees, false);
//   LeftMiddle.spinFor(forward, -750, degrees, false);
//   RightMiddle.spinFor(forward, -850, degrees, false);
//   LeftDown.spinFor(forward, -750, degrees, false);
//   RightDown.spinFor(forward, -850, degrees, true);
// while (true){
//     Cata.spin(forward, 12, volt);
//   }

  BlueMatchAuton();
  while(!q.empty()){
    //get values
    now = q.front(); q.pop();
    
    //turn
  target_ang = now.ang;

  while(true) {
      err = target_ang - InertialSensor.heading(degrees);
      if(std::abs(err) >= 180){
        err = -std::abs(err)/err * 360 - std::abs(err);
      }
      if(err<=1) {err=2; break;}
      drive_motors(forward, 0, turnVals.get_value(err));
      wait(20, msec);
    }

    //Drive
      prev_dis = -1; //had the bug of not resetting after an iteration

    while(std::abs(dis - dis_accept) >= 0.1 && (prev_dis == -1||dis <= prev_dis)){ //last part to prevent robot from runnin away and do shit
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Drive");
      prev_dis = dis;
      //important! Compare ipos with ipos values! Was a very big bug, coz the distance
      //got further and further away, even tho the direction and distance was correct
      powVal = driveVals.get_value(dis);
      drive_motors(forward, powVal, 0);
      wait(20, msec);
      Controller1.Screen.clearLine();
    }

    // // Intake
    // if(now.intake) Intake.spin(forward, 12, volt);
    // else Intake.stop();
    // wait(20, msec);

    // // Roller
    // if(now.roller!=0) {
    //   Intake.spin(forward, 12, volt);
    //   wait(now.roller, msec);
    // }
    // else Intake.stop();

    // // Catapult
    // if(now.bruh) fire();
    // wait(20, msec);
  }
}


void usercontrol(void) {
  // User control code here, inside the loop
  double turnImportance = 0.3; //How much turning slows down the speed of forward, 0 doesn't affect, 1 stops forward
  double turnSensitivity = 0.8; //How sensitive a turn is, 0 doesn't turn, 1 most sensitive
  LeftUp.setStopping(coast);
  LeftMiddle.setStopping(coast);
  LeftDown.setStopping(coast);
  RightUp.setStopping(coast);
  RightMiddle.setStopping(coast);
  RightDown.setStopping(coast);
  Cata.setStopping(hold);
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //---------------Drivetrain---------------
    if(Controller1.ButtonR1.pressing()) Cata.spin(forward, 12, volt);
    double motorTurnVal = Controller1.Axis1.position(percent);
    double motorForwardVal = Controller1.Axis3.position(percent);

    //Volts range: -12 --> 12, converts percentage to volts
    double motorTurnVolts = turnSensitivity*(motorTurnVal * 0.12);
    //Times forward volts by a percentage from how much you turn and how important the turn is to slowing down forward speed
    double motorForwardVolts = motorForwardVal * 0.12 * (1 - (std::abs(motorTurnVolts)/12 * turnImportance)); 

    LeftUp.spin(forward, motorForwardVolts + motorTurnVolts, volt);
    LeftMiddle.spin(forward, motorForwardVolts + motorTurnVolts, volt);
    LeftDown.spin(forward, motorForwardVolts + motorTurnVolts, volt);
    RightUp.spin(forward, motorForwardVolts - motorTurnVolts, volt);
    RightMiddle.spin(forward, motorForwardVolts - motorTurnVolts, volt);
    RightDown.spin(forward, motorForwardVolts - motorTurnVolts, volt);

    //Intake & Roller
    if(Controller1.ButtonL2.pressing()){
      Intake.spin(forward, 10, volt); //Intake
    } else if(Controller1.ButtonL1.pressing()){
      Intake.spin(reverse, 12, volt); //Roller
    }
    else Intake.stop();
      
    //Catapult == 8 rubber bands
    // if(!Limit.pressing()) Cata.spin(forward, 12, volt);
    // else Cata.stop();
    Controller1.ButtonR2.pressed(fire);


    //Pnuematic String
    //if(Brain.Timer.value()>=95)Controller1.ButtonB.pressed(shoot);
    Controller1.ButtonB.pressed(shoot);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() { //2 examples of threads that can run by itself, driver and while loop
  // Set up callbacks for autonomous and driver control periods.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  thread t1(record);
  t1.join();
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}