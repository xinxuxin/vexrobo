#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotor2 = motor(PORT20, ratio18_1, false);
motor RightMotor2 = motor(PORT10, ratio18_1, true);
motor LeftMotor1 = motor(PORT12, ratio18_1, false);
motor RightMotor1 = motor(PORT1, ratio18_1, true);
motor ArmMotor = motor(PORT19, ratio36_1, true);
motor LiftMotor = motor(PORT11, ratio36_1, true);
sonar LeftSonar = sonar(Brain.ThreeWirePort.G);
sonar RightSonar = sonar(Brain.ThreeWirePort.E);
gyro Gyro = gyro(Brain.ThreeWirePort.C);
limit LiftLimit = limit(Brain.ThreeWirePort.B);
bumper BumperL = bumper(Brain.ThreeWirePort.D);
bumper BumperR = bumper(Brain.ThreeWirePort.A);
/*vex-vision-config:begin*/
vision Vision = vision (PORT13, 50);
/*vex-vision-config:end*/
motor IntakeMotor1 = motor(PORT15, ratio18_1, false);
motor IntakeMotor2 = motor(PORT9, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}