#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftUp = motor(PORT5, ratio6_1, true);
motor LeftMiddle = motor(PORT6, ratio6_1, true);
motor LeftDown = motor(PORT9, ratio6_1, true);
motor RightUp = motor(PORT1, ratio6_1, false);
motor RightMiddle = motor(PORT2, ratio6_1, false);
motor RightDown = motor(PORT3, ratio6_1, false);
controller Controller1 = controller(primary);
motor Intake = motor(PORT19, ratio6_1, false);
motor Cata = motor(PORT10, ratio36_1, true);
encoder TW_forw = encoder(Brain.ThreeWirePort.G);
encoder TW_side = encoder(Brain.ThreeWirePort.E);
inertial InertialSensor = inertial(PORT16);
digital_out Left_String = digital_out(Brain.ThreeWirePort.A);
limit Limit = limit(Brain.ThreeWirePort.B);
digital_out Right_String = digital_out(Brain.ThreeWirePort.C);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}