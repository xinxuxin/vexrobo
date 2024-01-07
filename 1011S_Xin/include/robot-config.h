using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor LeftMotor2;
extern motor RightMotor2;
extern motor LeftMotor1;
extern motor RightMotor1;
extern motor ArmMotor;
extern motor LiftMotor;
extern sonar LeftSonar;
extern sonar RightSonar;
extern gyro Gyro;
extern limit LiftLimit;
extern bumper BumperL;
extern bumper BumperR;
extern signature Vision__SIG_1;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern motor IntakeMotor1;
extern motor IntakeMotor2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );