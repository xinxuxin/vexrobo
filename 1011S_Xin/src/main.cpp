/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Xin Xu, Sky Ning                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  VRC 2020 Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftMotor2           motor         20              
// RightMotor2          motor         10              
// LeftMotor1           motor         12              
// RightMotor1          motor         1               
// ArmMotor             motor         19              
// LiftMotor            motor         11              
// LeftSonar            sonar         G, H            
// RightSonar           sonar         E, F            
// Gyro                 gyro          C               
// LiftLimit            limit         B               
// BumperL              bumper        D               
// BumperR              bumper        A               
// Vision               vision        13              
// IntakeMotor1         motor         15              
// IntakeMotor2         motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
using namespace vex;

competition Competition;
// PID logic
/*void PID(float value){
  int Kp = 0.5;
  int Ki = 0.2;
  int Kd = 0.1;
  float error;
  int integral;
  while (condition){
  error = (value) – ();
  integral += error;
  if (error = 0)
  {
  integral = 0;
  }
  if ( abs(error) > 40)
  {
  integral = 0;
  }
  derivative = error – previous_error;
  previous_error = error;
  speed = Kp*error + Ki*integral + Kd*derivative;
}*/
float abs(float input){ //override abs function, absolute value of float type
  if (input <= 0){
    return -input;
  }
  return input;
}

int BasicPID(float Constant, float adjust){
  float speed = 0;
  float error;
  float errorP;
  error = fabs(Constant) - fabs(LeftMotor2.rotation(rotationUnits::rev));
  errorP  = fabs((error/fabs(Constant)*100));
  if (fabs(errorP)<30){
    speed = adjust;
  }
  else if (fabs(errorP) >= 30 && fabs(errorP) <=5){
    speed = errorP * 3;
  }
  else{
    speed = errorP;
  }
  return speed;
}
//while loop and error control
void verticle(){ // make it always verticle to the wall USING: ultrasonic (the difference of two sonars)
  float DistanceL = LeftSonar.distance(mm);
  float DistanceR = RightSonar.distance(mm);
  float error = DistanceL - DistanceR;
  while (abs(error)>5){
    if (error < 0 ){
      LeftMotor1.spin(forward,10,pct);
      LeftMotor2.spin(forward,10,pct);
      RightMotor1.spin(reverse,10,pct);
      RightMotor2.spin(reverse,10,pct);
      error = LeftSonar.distance(mm) - RightSonar.distance(mm);
      if (error > -0.5 && error < 0.5){
        break;
      }
    }
    else if (error > 0 ){
      LeftMotor1.spin(reverse,10,pct);
      LeftMotor2.spin(reverse,10,pct);
      RightMotor1.spin(forward,10,pct);
      RightMotor2.spin(forward,10,pct);
      error = LeftSonar.distance(mm) - RightSonar.distance(mm);
      if (error > -0.5 && error < 0.5){
        break;
    }
    }
  }
}

// while loop and recursion
void adAngle(float error){//USING: adjust angle using GYRO final angle and error 
  int realAngle = Gyro.heading();
  int vel = 5;
  if (-error >= realAngle){
    while (realAngle <= 0){
      realAngle = Gyro.heading();
      LeftMotor1.spin(forward, vel, pct);
      LeftMotor2.spin(forward, vel, pct);
      RightMotor1.spin(reverse, vel, pct);
      RightMotor2.spin(reverse, vel, pct);
    }
  }
  else if (realAngle >= error){
    while (realAngle >= 0){
      realAngle = Gyro.heading();
      LeftMotor1.spin(reverse, vel, pct);
      LeftMotor2.spin(reverse, vel, pct);
      RightMotor1.spin(forward, vel, pct);
      RightMotor2.spin(forward, vel, pct);
    }
  }
}

//math calculation
void my_Move(int _pct_, int displacement){ // convert inch to cm unit: cm  USING: normal moving without PID
  LeftMotor1.setVelocity(_pct_,percent);
  LeftMotor2.setVelocity(_pct_,percent);
  RightMotor1.setVelocity(_pct_,percent);
  RightMotor2.setVelocity(_pct_,percent);

  float PERIMETER = 31.9664592325;
  float RequestedValue = displacement / PERIMETER * 360;

  LeftMotor1.spinFor(RequestedValue,degrees,false);
  LeftMotor2.spinFor(RequestedValue,degrees,false);
  RightMotor1.spinFor(RequestedValue,degrees,false);
  RightMotor2.spinFor(RequestedValue,degrees);

  
}

float pid_Kp = 1;	//1.2
float pid_Ki = 0;	//1.05
float pid_Kd = 0;	//0.0

//pid control
void pid_Move(int _pct_, int displacement) // convert inch to cm unit: cm IMPORTANT : precise PID MOVING 
{
  // WHEEL DIAMETER = 10.17524 cm  

  LeftMotor1.setVelocity(_pct_, percent);
  LeftMotor2.setVelocity(_pct_, percent);
  RightMotor1.setVelocity(_pct_, percent);
  RightMotor2.setVelocity(_pct_, percent);

  float PERIMETER = 31.9664592325;
  float pidRequestedValue = displacement / PERIMETER * 360;
	float pidErrorL;
	float pidErrorR;
	float pidLastErrorL;
	float pidLastErrorR;
	float pidCurrentSensorValueL;
	float pidCurrentSensorValueR;
	float pidDriveL;
	float pidDriveR;
	float pidIntegralL;
	float pidIntegralR;
	float pidDerivativeL;
	float pidDerivativeR;
  int startTime = Brain.Timer.value();
  int timeout = 1;
	LeftMotor1.setPosition(0,degrees);
	RightMotor1.setPosition(0,degrees);

  Controller1.Screen.print(pidRequestedValue);
	pidLastErrorL = 0;
	pidLastErrorR = 0;
	pidIntegralL = 0;
	pidIntegralR = 0;
  pidCurrentSensorValueL = LeftMotor1.position(degrees);
	pidCurrentSensorValueR = RightMotor1.position(degrees);
	while((Brain.Timer.value() - startTime) < timeout)
	{

		pidCurrentSensorValueL = LeftMotor1.position(degrees);
		pidCurrentSensorValueR = RightMotor1.position(degrees);

		//Proportional Part
		pidErrorL = abs(pidCurrentSensorValueL - pidRequestedValue);
		pidErrorR = abs(pidCurrentSensorValueR - pidRequestedValue);

		//Integral Part
		if(pid_Ki != 0)
		{
			if(((abs(pidErrorL) + abs(pidErrorR)) / 2) < 50)
			{
				pidIntegralL += pidErrorL;
				pidIntegralR += pidErrorR;
			}
			else
			{
				pidIntegralL = 0;
				pidIntegralR = 0;
			}
		}
		else
		{
   		pidIntegralL = 0;
   		pidIntegralR = 0;
   	}

		//Derivative Part
		pidDerivativeL = pidErrorL - pidLastErrorL;
    pidLastErrorL  = pidErrorL;
    pidDerivativeR = pidErrorR - pidLastErrorR;
    pidLastErrorR  = pidErrorR;

    //Calculating the motor speed
		pidDriveL = (pid_Kp * pidErrorL) + (pid_Ki * pidIntegralL) + (pid_Kd * pidDerivativeL);
		pidDriveR = (pid_Kp * pidErrorR) + (pid_Ki * pidIntegralR) + (pid_Kd * pidDerivativeR);
		LeftMotor1.spinFor(pidDriveL,degrees,false);
		LeftMotor2.spinFor(pidDriveL,degrees,false);
		RightMotor1.spinFor(pidDriveR,degrees,false);
		RightMotor2.spinFor(pidDriveR,degrees);
	}
}

// While loop
void sonar_Move(int _pct_, int displacement){ // Using sonar to locate the robot and move to the specific position
  LeftMotor1.setVelocity(_pct_,percent);
  LeftMotor2.setVelocity(_pct_,percent);
  RightMotor1.setVelocity(_pct_,percent);
  RightMotor2.setVelocity(_pct_,percent);

  float DistanceL = LeftSonar.distance(mm);
  float DistanceR = RightSonar.distance(mm);
  float averageDistance = (DistanceL + DistanceR) / 20; //unit :cm
  verticle();
  while (averageDistance < displacement){
    DistanceL = LeftSonar.distance(mm);
    DistanceR = RightSonar.distance(mm);
    averageDistance = (DistanceL + DistanceR) / 20;

    LeftMotor1.spin(forward, _pct_, percent);
    LeftMotor2.spin(forward, _pct_, percent);
    RightMotor1.spin(forward, _pct_, percent);
    RightMotor2.spin(forward, _pct_, percent);
  
  }
  while (averageDistance > displacement){
    DistanceL = LeftSonar.distance(mm);
    DistanceR = RightSonar.distance(mm);
    averageDistance = (DistanceL + DistanceR) / 20;

    LeftMotor1.spin(reverse, _pct_, percent);
    LeftMotor2.spin(reverse, _pct_, percent);
    RightMotor1.spin(reverse, _pct_, percent);
    RightMotor2.spin(reverse, _pct_, percent);
  }
  verticle();
}


  void pid_Turn(int Angle) // unit: cm precise PID turning
{
  // WHEEL DIAMETER = 10.17524 cm  
int _pct_ = 60;
  LeftMotor1.setVelocity(_pct_, percent);
  LeftMotor2.setVelocity(_pct_, percent);
  RightMotor1.setVelocity(_pct_, percent);
  RightMotor2.setVelocity(_pct_, percent);

  float turningRatio = 13.4536 / 4.006 ; // turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
  float pidRequestedValue = turningRatio * Angle;
	float pidErrorL;
	float pidErrorR;
	float pidLastErrorL;
	float pidLastErrorR;
	float pidCurrentSensorValueL;
	float pidCurrentSensorValueR;
	float pidDriveL;
	float pidDriveR;
	float pidIntegralL;
	float pidIntegralR;
	float pidDerivativeL;
	float pidDerivativeR;
  int startTime = Brain.Timer.value();
  int timeout = 1;
	LeftMotor1.setPosition(0,degrees);
	RightMotor1.setPosition(0,degrees);

  Controller1.Screen.print(pidRequestedValue);
	pidLastErrorL = 0;
	pidLastErrorR = 0;
	pidIntegralL = 0;
	pidIntegralR = 0;
  pidCurrentSensorValueL = LeftMotor1.position(degrees);
	pidCurrentSensorValueR = RightMotor1.position(degrees);
	while((Brain.Timer.value() - startTime) < timeout)
	{

		pidCurrentSensorValueL = LeftMotor1.position(degrees);
		pidCurrentSensorValueR = RightMotor1.position(degrees);

		//Proportional Part
		pidErrorL = abs(pidCurrentSensorValueL - pidRequestedValue);
		pidErrorR = abs(pidCurrentSensorValueR - pidRequestedValue);

		//Integral Part
		if(pid_Ki != 0)
		{
			if(((abs(pidErrorL) + abs(pidErrorR)) / 2) < 50)
			{
				pidIntegralL += pidErrorL;
				pidIntegralR += pidErrorR;
			}
			else
			{
				pidIntegralL = 0;
				pidIntegralR = 0;
			}
		}
		else
		{
   		pidIntegralL = 0;
   		pidIntegralR = 0;
   	}

		//Derivative Part
		pidDerivativeL = pidErrorL - pidLastErrorL;
    pidLastErrorL  = pidErrorL;
    pidDerivativeR = pidErrorR - pidLastErrorR;
    pidLastErrorR  = pidErrorR;

    //Calculating the motor speed
		pidDriveL = (pid_Kp * pidErrorL) + (pid_Ki * pidIntegralL) + (pid_Kd * pidDerivativeL);
		pidDriveR = (pid_Kp * pidErrorR) + (pid_Ki * pidIntegralR) + (pid_Kd * pidDerivativeR);
		LeftMotor1.spinFor(pidDriveL,degrees,false);
		LeftMotor2.spinFor(pidDriveL,degrees,false);
		RightMotor1.spinFor(-pidDriveR,degrees,false);
		RightMotor2.spinFor(-pidDriveR,degrees);
    Controller1.Screen.print(pidDriveL);
	}
}



void GNadjustAngle(float Dest){ //general adjust angle (could adjust to specific angle) USING: Gyro
  int realAngle = Gyro.heading();
  int vel = 10;
  float turnA = Dest - realAngle;
  if (0 <= turnA && turnA <= 180){
    while (realAngle <= Dest){ // right turn
      realAngle = Gyro.heading();
      LeftMotor1.spin(forward, vel, pct);
      LeftMotor2.spin(forward, vel, pct);
      RightMotor1.spin(reverse, vel, pct);
      RightMotor2.spin(reverse, vel, pct);
    }
  }
  else if (0 > turnA && turnA >= -180){
    while (realAngle >= Dest){ // left turn
      realAngle = Gyro.heading();
      LeftMotor1.spin(reverse, vel, pct);
      LeftMotor2.spin(reverse, vel, pct);
      RightMotor1.spin(forward, vel, pct);
      RightMotor2.spin(forward, vel, pct);
    }
  }
}

void liftlim(){ // USING: Limit Switch to limit the position of tray
  if (LiftLimit.pressing()){
    LiftMotor.spinFor(10,degrees);
  }
  }


void slide(double _pct_, float displacement, bool bl){ //Mecanum wheel normal slide function ***** set right as positive unit: cm
  LeftMotor1.setVelocity(_pct_,percent);
  LeftMotor2.setVelocity(_pct_,percent);
  RightMotor1.setVelocity(_pct_,percent);
  RightMotor2.setVelocity(_pct_,percent);

  float PERIMETER = 31.9664592325;
  float RequestedValue = displacement / PERIMETER * 360;

  LeftMotor1.spinFor(RequestedValue,degrees,false);
  LeftMotor2.spinFor(-RequestedValue,degrees,false);
  RightMotor1.spinFor(-RequestedValue ,degrees,false);
  RightMotor2.spinFor(RequestedValue,degrees, bl);

  
}

void pop(){ // pop out function TIME : 0.3 seconds ** final pop **
  IntakeMotor1.spinFor(-830,degrees,false);
  IntakeMotor2.spinFor(-830,degrees);
}

void my_Turn( float degrees ) {  // only turn left negative
    float CurrentValue = Gyro.heading();
    float error = CurrentValue - degrees;
    while (true){
      CurrentValue = Gyro.heading();
      LeftMotor1.spin(reverse,60,pct);
      LeftMotor2.spin(reverse,60,pct);
      RightMotor1.spin(forward,60,pct);
      RightMotor2.spin(forward,60,pct);
      if (error < 0){
        break;
      }
    }
}

void Turn( float degrees ) {  // width = 9'' length = 10''
    // Note: +90 degrees is a right turn
    // turningRatio = squre root 9^2 + 8^2
    float turningRatio = 13.4536 / 4.006 ; // turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
    float wheelDegrees = turningRatio * degrees * 1.45;    
    // Divide by two because each wheel provides half the rotation
    int pct = 40;
    LeftMotor1.startRotateFor(wheelDegrees, vex::rotationUnits::deg, pct, vex::velocityUnits::pct);
    LeftMotor2.startRotateFor(wheelDegrees, vex::rotationUnits::deg, pct, vex::velocityUnits::pct);
    RightMotor1.rotateFor(- wheelDegrees, vex::rotationUnits::deg, pct, vex::velocityUnits::pct,false);
    RightMotor2.rotateFor(- wheelDegrees * 0.97, vex::rotationUnits::deg, pct, vex::velocityUnits::pct);
}

void gyroTurn (float setPoint, float kP, float kI, float kD) { // precise gyro turn
  int Go = Gyro.heading();
  int errorTotal = 0;
  int error = 0;
  int errorLast = 0;
  int pTerm;
  int iTerm;
  int dTerm;
  int Power;
  while(Go < setPoint) {
      // Calculate error
      error = setPoint - Go;
      errorTotal += error;
      errorLast = error;

      // Find proportional term
      pTerm = error * kP;
      // Find integral term
      iTerm = kI * errorTotal;
      // Find derivative term
      dTerm = kD * (error - errorLast);

      // Compute output to send to motors
      Power = pTerm + iTerm + dTerm;

      // Set your motor speeds
     LeftMotor1.spinFor(-Power,degrees,false);
    LeftMotor2.spinFor(-Power,degrees,false);
  RightMotor1.spinFor(Power,degrees,false);
  RightMotor2.spinFor(Power,degrees);

  }
}

void Lift(bool c){
  if (c){ //100% to 20%
    int x = 1;
    while(LiftMotor.position(degrees) < 660){
      
      LiftMotor.spin(forward,100-x, percent);
      x += 2;
    
    }
    }
  else if (!c){
    LiftMotor.setVelocity(90,percent);
    LiftMotor.spinFor(530,degrees,false);
    wait(400,msec);
    }
  LiftMotor.setVelocity(40,pct);
}


//implements exponetial power

/* FrontLeft = Ch3 + Ch1 + Ch4
RearLeft = Ch3 + Ch1 - Ch4
FrontRight = Ch3 - Ch1 - Ch4
RearRight = Ch3 - Ch1 + Ch4

Where:
Ch1 = Right joystick X-axis
Ch3 = Left joystick Y-axis
Ch4 = Left joystick X-axis
Where positive is to the right or up on the joysticks. */
void Mecanum(){
  LeftMotor1.spin(directionType::fwd,Controller1.Axis1.value() + Controller1.Axis3.value() + Controller1.Axis4.value() , velocityUnits::pct);
  LeftMotor2.spin(directionType::fwd,Controller1.Axis1.value() + Controller1.Axis3.value() - Controller1.Axis4.value(), velocityUnits::pct);
  RightMotor1.spin(directionType::rev,Controller1.Axis1.value() - Controller1.Axis3.value() + Controller1.Axis4.value() , velocityUnits::pct);
  RightMotor2.spin(directionType::rev,Controller1.Axis1.value() - Controller1.Axis3.value() - Controller1.Axis4.value() , velocityUnits::pct);
}

void autostack(){ //lles than 7
  IntakeMotor1.setVelocity(90,percent);
  IntakeMotor2.setVelocity(90,percent);
  IntakeMotor1.spinFor(-230,degrees,false);
  IntakeMotor2.spinFor(-230,degrees);
  LiftMotor.setTimeout(2,seconds);
  LiftMotor.setVelocity(80,percent);
  LiftMotor.spinFor(270,degrees);
  LiftMotor.setVelocity(45,percent);
  LiftMotor.spinFor(205,degrees);
  LiftMotor.setVelocity(25,percent);
  LiftMotor.spinFor(200,degrees);
  wait(50,msec);
  my_Move(50,-40);
  LiftMotor.spinToPosition(300,degrees);
  
}

void autostack2(){// cubes greater than 7 ***************************
  LiftMotor.setVelocity(100,percent);
  LiftMotor.spinFor(400,degrees);
  LiftMotor.setVelocity(60,percent);
  //LiftMotor.spinFor(80,degrees);
  LiftMotor.setVelocity(60,percent);
  LiftMotor.spinFor(80,degrees);
  LiftMotor.setVelocity(25,percent);
  LiftMotor.spinFor(100,degrees);
  LiftMotor.setVelocity(10,percent);
  LiftMotor.spinFor(80,degrees);
  wait(500,msec);
  LiftMotor.setVelocity(100,percent);
  my_Move(40,-30);
  LiftMotor.spinToPosition(0,degrees);
}

void autoTowerLow(){// lowest = -444.2 second = -490
  ArmMotor.setVelocity(100,pct);
  ArmMotor.spinToPosition(-485,degrees,true);
  IntakeMotor1.setVelocity(95,pct);
  IntakeMotor1.setVelocity(95,pct);
  IntakeMotor1.spinFor(reverse,2,turns,false);
  IntakeMotor2.spinFor(reverse,2,turns,true);
  wait(400,msec);
  ArmMotor.spinToPosition(0,degrees);

}
void autoTowerHigh(){// lowest = -444.2 high = -655
  ArmMotor.setVelocity(100,pct);
  ArmMotor.spinToPosition(-655,degrees,true);
  IntakeMotor1.setVelocity(95,pct);
  IntakeMotor1.setVelocity(95,pct);
  IntakeMotor1.spinFor(reverse,2,turns,false);
  IntakeMotor2.spinFor(reverse,2,turns,true);
  wait(400,msec);
  ArmMotor.spinToPosition(0,degrees);

}
void fivepointsB(){ // blue small 

      my_Move(30,23);
      my_Move(40,-18);
      pop();
      ArmMotor.spinToPosition(0,degrees,false);
      wait(400,msec);
      verticle();
      IntakeMotor1.setVelocity(90,percent);
      IntakeMotor2.setVelocity(90,percent);
      IntakeMotor1.spinFor(8.5,turns,false);
      IntakeMotor2.spinFor(8.5,turns,false);
      my_Move(50,122);
      wait(100,msec);
      my_Move(60,-95);
      verticle();
      Turn(-137);
      wait(100,msec);
      my_Move(60,38);
      autostack();
}

void fivepointsR(){ // five points red
    my_Move(30,22);
    my_Move(40,-17);
    pop();
    ArmMotor.spinToPosition(0,degrees,false);
    wait(400,msec);
    verticle();
    IntakeMotor1.setVelocity(90,percent);
    IntakeMotor2.setVelocity(90,percent);
    IntakeMotor1.spinFor(8.3,turns,false);
    IntakeMotor2.spinFor(8.3,turns,false);
    my_Move(50,122);
    wait(100,msec);
    my_Move(60,-94);
    verticle();
    Turn(137);
    wait(100,msec);
    my_Move(60,39);
    autostack();
}

void fivepointsRB(){ // red big
    pop();
    ArmMotor.spinToPosition(0,degrees);
    wait(400,msec);
    my_Move(50,65);
    Turn(-35);
    IntakeMotor1.setVelocity(80,percent);
    IntakeMotor2.setVelocity(80,percent);
    IntakeMotor1.spinFor(2,turns,false);
    IntakeMotor2.spinFor(2,turns,false);
    my_Move(100,60);
    wait(100,msec);
    /*Turn(-160);
    IntakeMotor1.setVelocity(80,percent);
    IntakeMotor2.setVelocity(80,percent);
    IntakeMotor1.spinFor(5,turns,false);
    IntakeMotor2.spinFor(5,turns,false);
    my_Move(60,-94);
   Turn(90);*/
}

/*void vis(){ // vision sensor tracking cube
  Vision.takeSnapshot(Vision__PPBOX);

    int currentValue = Vision.objects[2].centerX;
    int x = 25;
     Controller1.Screen.print(Vision.objects[2].exists);
     while(true){
       currentValue = Vision.objects[2].centerX;
       if (103 >= currentValue && currentValue >=97){
          LeftMotor1.stop();
        LeftMotor2.stop();
        RightMotor1.stop();
        RightMotor2.stop();
        break;
        }
    if (currentValue <= 95){
      while (currentValue <= 100){
        Vision.takeSnapshot(Vision__PPBOX);
        currentValue = Vision.objects[2].centerX;
        
        LeftMotor1.spin(forward, x,pct);
        LeftMotor2.spin(forward, x,pct);
        RightMotor1.spin(reverse, x,pct);
        RightMotor2.spin(reverse, x,pct);
        if (currentValue >=97){
          LeftMotor1.stop();
        LeftMotor2.stop();
        RightMotor1.stop();
        RightMotor2.stop();
        }
      }
      
    }
    if (currentValue >= 105){
      while (currentValue > 100){
        Vision.takeSnapshot(Vision__PPBOX);
        currentValue = Vision.objects[2].centerX;
        
        LeftMotor1.spin(reverse, x,pct);
        LeftMotor2.spin(reverse, x,pct);
        RightMotor1.spin(forward, x,pct);
        RightMotor2.spin(forward, x,pct);
        if (currentValue <= 103){
          LeftMotor1.stop();
        LeftMotor2.stop();
        RightMotor1.stop();
        RightMotor2.stop();
        }
      }
    }
    }
}*/
void ninepointsauto(){ // 9 points skills run IMPORTANT CANNOT USE
    my_Move(30,20);
    my_Move(40,-18);
    pop();
    ArmMotor.spinToPosition(0,degrees);
    //verticle();
    IntakeMotor1.setVelocity(90,percent);
    IntakeMotor2.setVelocity(90,percent);
    IntakeMotor1.spinFor(2,turns,false);
    IntakeMotor2.spinFor(2,turns,false);
    my_Move(50,50);
    autoTowerLow();
    my_Move(40,-50);
    Turn(45);
    verticle();
    wait(200,msec);
    slide(40,-20,true);
    verticle();

    IntakeMotor1.spinFor(9,turns,false);
    IntakeMotor2.spinFor(9,turns,false);
    my_Move(30,120);
    verticle();
    /*slide(60,20,true);
    verticle();
    IntakeMotor1.spinFor(2,turns,false);
    IntakeMotor2.spinFor(2,turns,false);
    my_Move(40,40);
    slide(60,-20,true);*/

    IntakeMotor1.spinFor(11.2,turns,false);
    IntakeMotor2.spinFor(11.2,turns,false);
    my_Move(30,160);
    Turn(-90);
    slide(70,58,true);
    my_Move(30,26);
    Turn(10);
    autostack2();
}
void butt(){
  //intake
  if (Controller1.ButtonR2.pressing()) {  // out
      IntakeMotor1.spin(reverse, 85, pct);
      IntakeMotor2.spin(reverse, 85, pct);
    } else if (Controller1.ButtonR1.pressing()) { // in
      IntakeMotor1.spin(forward, 85, pct); // +10 %
      IntakeMotor2.spin(forward, 85, pct);
    } else {
      IntakeMotor1.stop();
      IntakeMotor2.stop();
    }
    //arm
    if (Controller1.ButtonL2.pressing()) {
      ArmMotor.spin(forward, 80, pct);
    } else if (Controller1.ButtonL1.pressing()) { // rise
      ArmMotor.spin(reverse, 80, pct);
    } else {
      ArmMotor.stop(hold);
    }
    //lift
    if (Controller1.ButtonDown.pressing() && ! LiftLimit.pressing()) { //lift down
      //LiftMotor.spin(reverse, 90 ,pct);
    } else if (Controller1.ButtonUp.pressing()) { // lift up
      if (LiftMotor.position(degrees) < 660){
      LiftMotor.spin(forward, 40, pct);
      }
    } else {
      LiftMotor.stop(hold);
    }
    //autostack
    if (Controller1.ButtonA.pressing()) { 
      autostack2();
    }

    if (Controller1.ButtonY.pressing()) { 
      //Controller1.Screen.print(ArmMotor.position(degrees));
      //autoTowerLow();
      //fivepointsRB();
      ninepointsauto();
      
    }
    if (Controller1.ButtonLeft.pressing()) { 
      //Controller1.Screen.print(ArmMotor.position(degrees));
      //autoTowerLow();
     //ArmMotor.spinToPosition(0,degrees);
     slide(60,20,true);
    }
    if (Controller1.ButtonDown.pressing()) { 
      //Controller1.Screen.print(ArmMotor.position(degrees));
      //autoTowerLow();
     ArmMotor.spinToPosition(0,degrees);
    }
    //if (Controller1.ButtonLeft.pressing()) {
      //pop();
      /*ArmMotor.spinToPosition(-18,degrees,false); 
      IntakeMotor1.setVelocity(95,percent);
      IntakeMotor2.setVelocity(95,percent);
      IntakeMotor1.spinFor(7,turns,false);
      IntakeMotor2.spinFor(7,turns,false);
      my_Move(60,122);
      wait(100,msec);
      
      Turn(27);
      my_Move(95,-122);
      verticle();
      slide(45,-23,true);
      verticle();
      IntakeMotor1.spinFor(8,turns,false);
      IntakeMotor2.spinFor(8,turns,false);
      my_Move(60,108);
      my_Move(80,-107);
      Turn(-130);
      my_Move(60,34);//13
      
      autostack2();
      
      ArmMotor.spinToPosition(-18,degrees,false); 
      IntakeMotor1.setVelocity(90,percent);
      IntakeMotor2.setVelocity(90,percent);
      IntakeMotor1.spinFor(7,turns,false);
      IntakeMotor2.spinFor(7,turns,false);
      my_Move(60,122);
      wait(100,msec);
      sonar_Move(80,45);
      wait(100,msec);
      verticle();
      slide(90,-80,true);
      verticle();
      IntakeMotor1.spinFor(7.5,turns,false);
      IntakeMotor2.spinFor(7.5,turns,false);
      my_Move(60,106);
      sonar_Move(80, 57);
      Turn(-135);
      my_Move(70,36);//13
      autostack2();*/
    //}
    if (Controller1.ButtonRight.pressing()) { // 7 points blue
      my_Move(30,20);
      my_Move(40,-18);
      pop();
      ArmMotor.spinToPosition(0,degrees,false); 
      IntakeMotor1.setVelocity(90,percent);
      IntakeMotor2.setVelocity(90,percent);
      IntakeMotor1.spinFor(8.5,turns,false);
      IntakeMotor2.spinFor(8.5,turns,false);
      my_Move(40,122);
      my_Move(75,-110);
      wait(200,msec);
      verticle();
      slide(80,-80.96,true);
      Turn(20);
      verticle();
      
      IntakeMotor1.spinFor(7.3,turns,false);
      IntakeMotor2.spinFor(7.3,turns,false);
      my_Move(40,105);
      my_Move(70,-115);
      wait(300,msec);
      verticle();
      Turn(-110);
      slide(50,-30,true);
      Turn(-10);
      my_Move(50,18);
      Turn(-10);
      autostack2();
   }
}

void push(){    // one point push
  pop();
  my_Move(50,-120);
  my_Move(50,80);
}


void pre_auton(void) {
  vexcodeInit();
  Gyro.calibrate(1); 
  Gyro.setHeading(0, degrees); 
  LiftMotor.setPosition(0,degrees);
  IntakeMotor1.setVelocity(80,percent);
  IntakeMotor2.setVelocity(80,percent);
}


void autonomous(void) {
  // IMPORTANT *************************** 一分推通用  push();
  // IMPORTANT *************************** 红色小区  fourpointsR();
  // IMPORTANT *************************** 蓝色小区 fivepointsB();
  // IMPORTANT *************************** Skills blue 9 points CANNOT USE
int position = 2; // 1 = 一分推通用; 2 = 红色小区fivepointsR(); 3 = fivepointsB(); 6 = skills blue
// ***************************
// skills trial1 position 3
//        trial 2 position 4
// **************************
//15s auto position 2 & 3
switch (position){
  
  case 1: //一分推通用
    push(); // 注意位置
    break;
  
  case 2: // 红色小区
    fivepointsR(); 
    break;
  
  case 3: // 蓝色小区
    fivepointsB();
    break;
  
  case 4: // skills blue auto 7分
    pop();
    ArmMotor.spinToPosition(-18,degrees,false); 
      IntakeMotor1.setVelocity(90,percent);
      IntakeMotor2.setVelocity(90,percent);
      IntakeMotor1.spinFor(7,turns,false);
      IntakeMotor2.spinFor(7,turns,false);
      my_Move(55,122);
      wait(100,msec);
      sonar_Move(80,45);
      wait(100,msec);
      verticle();
      slide(70,-80,true);
      verticle();
      IntakeMotor1.spinFor(7.5,turns,false);
      IntakeMotor2.spinFor(7.5,turns,false);
      my_Move(60,106);
      sonar_Move(60, 57);
      Turn(-136);
      my_Move(50,37);//13
      autostack2();
    
    case 5:
      ArmMotor.spinToPosition(-18,degrees,false); 
      IntakeMotor1.setVelocity(85,percent);
      IntakeMotor2.setVelocity(85,percent);
      //IntakeMotor1.spinFor(8,turns,false);
      //IntakeMotor2.spinFor(8,turns,false);
      my_Move(60,122);
      my_Move(80,-80);
      slide(40,-82,true);
      verticle();
      //IntakeMotor1.spinFor(7,turns,false);
      //IntakeMotor2.spinFor(7,turns,false);
      my_Move(60,90);
      my_Move(80,-90);
      Turn(-135);
      my_Move(70,28);
      break;
      
      case 6:
       //ninepointsauto();
      break;
}
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  IntakeMotor1.setStopping(hold);
  IntakeMotor2.setStopping(hold);
  ArmMotor.setStopping(hold);
  LiftMotor.setPosition(0,degrees);
  LeftMotor1.setVelocity(130,rpm);
  LeftMotor2.setVelocity(130,rpm);
  RightMotor1.setVelocity(130,rpm);
  RightMotor2.setVelocity(130,rpm);
  ArmMotor.setPosition(50,degrees);
  while (1) {
    butt();
    Mecanum();
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(50, msec);
  }
}