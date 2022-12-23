using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftUp;
extern motor LeftMiddle;
extern motor LeftDown;
extern motor RightUp;
extern motor RightMiddle;
extern motor RightDown;
extern controller Controller1;
extern motor Intake;
extern motor Cata;
extern encoder TW_forw;
extern encoder TW_side;
extern inertial InertialSensor;
extern digital_out Left_String;
extern limit Limit;
extern digital_out Right_String;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );