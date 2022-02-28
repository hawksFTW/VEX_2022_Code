/*----------------------------------------------------------------------------
*
* Copyright (C) 2021 Team 10F - All Rights Reserved
* Any other team can NOT use, distribute, or modify this code 
* as per the terms of VEX honor code.
*
*---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: VEX */
/* Created: Wed Aug 11 2021 */
/* Description: 10F Optical Shaft Encoder Odometry V5 C++ Project */
/* */
/* Name: */
/* Date */
/* Class: */
/* */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name] [Type] [Port(s)]
// Drivetrain drivetrain 1, 10, 11, 20, D
// ClawMotor motor 3 
// ArmMotor motor 8 
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

const double waitAfterAction = 0.1; // msec
const bool dowait = true;

void forward1(double dist, int speed=50) //forward with variable speed
{
driveTrain.driveFor(directionType::fwd, dist, distanceUnits::in, speed, velocityUnits::pct, false);

/*
backMotorA.spinFor(directionType::rev, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, false);
backMotorB.spinFor(directionType::fwd, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, false);
frontMotorB.spinFor(directionType::rev, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, false);
frontMotorA.spinFor(directionType::fwd, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, true);
*/
if(dowait) wait(waitAfterAction, msec);
}

// forward using separate motor commands
void forward2(double dist, int speed) //forward with variable speed
{
// driveTrain.driveFor(directionType::fwd, dist, distanceUnits::in, speed, velocityUnits::pct, true);

double revs = dist/(3.1416*4);
backMotorA.spinFor(directionType::fwd, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, false);
backMotorB.spinFor(directionType::fwd, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, false);
frontMotorB.spinFor(directionType::fwd, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, false);
frontMotorA.spinFor(directionType::fwd, revs, rotationUnits::rev, speed, vex::velocityUnits::pct, true);
if(dowait) wait(waitAfterAction, msec);
}

void backward(double dist, int speed=50)
{
driveTrain.driveFor(directionType::rev, dist, distanceUnits::in, speed, velocityUnits::pct, true);
if(dowait) wait(waitAfterAction, msec);
}

bool encLeftReverse = false; // reverse means true
bool encRightReverse = false; 
bool encBackReverse = false; // reverse means true

double getRotation(encoder e, rotationUnits units, bool reverse)
{
double r = e.rotation(units);
if(reverse)
r = -r;
return r;
}

double getRotVel(encoder e, velocityUnits units, bool reverse)
{
double v = e.velocity(units);
if(reverse)
v = -v;
return v;
}

// revs
double getRotPos(double rev0L, double rev0R)
{
double revL = getRotation(encLeft, rotationUnits::rev, encLeftReverse) - rev0L;
double revR = getRotation(encRight, rotationUnits::rev, encRightReverse) - rev0R; 
return (revL + revR)/2;
}

// rpm
double getRotVel()
{
double vL = getRotVel(encLeft, velocityUnits::rpm, encLeftReverse);
double vR = getRotVel(encRight, velocityUnits::rpm, encRightReverse);
return (vL + vR)/2;
}


// forward/backward using rotation sensors
void odomUpdate(double dist, int speed, directionType dir, int brake) //forward with variable speed
{
float speedTh = 12.5;
double v0 = speed;
if (speed <= speedTh)
{
// dont use encoder
if(dir == directionType::fwd) 
forward1(dist, speed);
else
backward(dist, speed);
return;
}
const double pi = 3.1415926;
double d = 3.25; // in
int timestep = 10; // msec
double stopDistTh = 2; // in 0.25
double ax0 = 10; // in, stop accelerating down from here 5
double x0 = 12; // in, start slowing down from here 14 for WP and 18 for elims 15 8 FOR SKILLS
// reset rot sensors
// rotSensorLeft.resetPosition();
// rotSensorRight.resetPosition();
double InitL = getRotation(encLeft, rotationUnits::rev, encLeftReverse)*pi*d;
double InitR = getRotation(encRight, rotationUnits::rev, encRightReverse)*pi*d;

Brain.Screen.print("Initial encoder revs = %f L, %f R\n", InitL, InitR);
Brain.Screen.newLine();

if(dist < (ax0 + x0))
{
if(dist > x0)
ax0 = dist - x0;
else {
ax0 = 0;
x0 = dist;
}
}

double RDist = 0;
double LDist = 0;
double ADist = 0;
// start moving 
// driveTrain.drive(dir, speed, velocityUnits::pct);

// min velocity to start with
const double v_min = 10;

// accelerating phase
while(ADist < ax0) 
{
double v = (v0/ax0)*(ADist);
if(v < v_min)
v = v_min;

driveTrain.drive(dir, v, velocityUnits::pct);
LDist = abs((getRotation(encLeft, rotationUnits::rev, encLeftReverse)*pi*d) - InitL);
RDist = abs((getRotation(encRight, rotationUnits::rev, encRightReverse)*pi*d) - InitR);
ADist = (RDist + LDist) / 2;
wait(timestep, msec);
}

driveTrain.drive(dir, speed, velocityUnits::pct);

// drive phase
while(ADist < (dist-x0)) 
{
LDist = abs((getRotation(encLeft, rotationUnits::rev, encLeftReverse)*pi*d) - InitL);
RDist = abs((getRotation(encRight, rotationUnits::rev, encRightReverse)*pi*d) - InitR);
ADist = (RDist + LDist) / 2;
wait(timestep, msec);
}

// Start slowing down
while(ADist < (dist-stopDistTh)) 
{
LDist = abs((getRotation(encLeft, rotationUnits::rev, encLeftReverse)*pi*d) - InitL);
RDist = abs((getRotation(encRight, rotationUnits::rev, encRightReverse)*pi*d) - InitR);
ADist = (RDist + LDist) / 2;
double v = (v0/x0)*(dist-ADist);
driveTrain.drive(dir, v, velocityUnits::pct);
wait(timestep, msec);
}

driveTrain.stop(brakeType::brake);
} 

// forward with rot sensor 
void forwardEnc(double dist, double speed=70, int brakeTh=500)
{
  odomUpdate(dist, speed, directionType::fwd, brakeTh);
}

// backward with rot sensor
void backwardEnc(double dist, int speed=70, int brakeTh=500)
{
  odomUpdate(dist, speed, directionType::rev, brakeTh);
}

void turnToHeading(double angle, int speed=25)
{
  driveTrain.turnToHeading(angle, rotationUnits::deg, speed, velocityUnits::pct, true);
}

double tryRotSensor(encoder s, char *name)
{
  // s.resetPosition();
  double rev0 = s.position(rotationUnits::rev);
  driveTrain.driveFor(directionType::fwd, 12, distanceUnits::in);
  double rev1 = s.position(rotationUnits::rev);
  Brain.Screen.print("Initial %s encoder revs = %f \n", name, rev0);
  Brain.Screen.newLine();
  // Controller1.Screen.print("Initial %s rot sensor revs = %f \n", name, rev0);
  Brain.Screen.print("Final %s encoder revs = %f \n", name, rev1);
  Brain.Screen.newLine();
  // Controller1.Screen.print("Final %s rot sensor revs = %f \n", name, rev1);
  return (rev1 - rev0);
}

void tryInertialSensor()
{
  double hd = isensor.heading();
  Brain.Screen.print("Initial sensor heading = %f \n", hd);
  Brain.Screen.newLine();

  turnToHeading(10);

  hd = isensor.heading();
  Brain.Screen.print("After turning sensor heading = %f \n", hd);
  Brain.Screen.newLine();
}

void resetEncoders()
{
encLeft.resetRotation();
encRight.resetRotation();
encBack.resetRotation();
}

void mfm(double rev, int speed, bool wfc)
{
  mogo.spinFor(directionType::fwd, rev, rotationUnits::rev, speed, velocityUnits::pct, wfc); //neg up and pos down
}

void mrm(double rev, int speed, bool wfc)
{
  RingScooper.spinFor(directionType::fwd, rev, rotationUnits::rev, speed, velocityUnits::pct, wfc);
}

void m4bm(double rev, int speed, bool wfc)
{
  FourBar.spinFor(directionType::fwd, rev, rotationUnits::rev, speed, velocityUnits::pct, wfc);
}

void mwpm(double rev, int speed, bool wfc)
{
  wp.spinFor(directionType::fwd, rev, rotationUnits::rev, speed, velocityUnits::pct, wfc);
}

void liftFrontMogo(bool completion = false)
{
  mogo.spinTo(-0.45, rotationUnits::rev, 100, velocityUnits::pct, completion);
}

void releaseFrontMogo(bool completion = false)
{
  mogo.spinTo(-0.95, rotationUnits::rev, 100, velocityUnits::pct, completion);
}

void liftBackMogo(bool completion = false)
{
  FourBar.spinTo(3.75, rotationUnits::rev, 100, velocityUnits::pct, completion);
}

void releaseBackMogo(bool completion  = false)
{
 FourBar.spinTo(-0.05, rotationUnits::rev, 100, velocityUnits::pct, completion);
}

void hookMogo()
{
  piston1.open();
  wait(0.2, sec);
  FourBar.spinTo(0.15, rotationUnits::rev, 50, velocityUnits::pct, true);
}

void unhookMogo()
{
  FourBar.spinTo(0.975, rotationUnits::rev, 50, velocityUnits::pct, true); //7
  piston1.close();
}

void unhookMogo1plat()
{
  FourBar.spinTo(0.975, rotationUnits::rev, 30, velocityUnits::pct, true); //7
  piston1.close();
}


void balanceMogo(bool completion = false)
{
  mogo.spinTo(-0.9, rotationUnits::rev, 4, velocityUnits::pct, completion);
}

void mfrm(double revs, bool completion = false)
{
  mogo.spinTo(revs, rotationUnits::rev, 100, velocityUnits::pct, completion);
}

void driveWithdistSensorbckd(double speed)
{
  while(true)
  {
    double dist = dSensor.objectDistance(inches);
    //driveTrain.drive(directionType::rev);
    driveTrain.drive(directionType::rev, speed, velocityUnits::pct);
    if(dist < 1) //1
    {
      driveTrain.stop(brakeType::brake);
      break;
    }
  }
}

void spline(int d, int speed1, int speed2)
{
  frontMotorA.startRotateFor(directionType::fwd, d, rotationUnits::rev, speed1, velocityUnits::rpm);
  frontMotorB.startRotateFor(directionType::rev, d, rotationUnits::rev, speed2, velocityUnits::rpm);
  backMotorA.startRotateFor(directionType::fwd, d, rotationUnits::rev, speed1, velocityUnits::rpm);
  backMotorB.startRotateFor(directionType::rev, d, rotationUnits::rev, speed2, velocityUnits::rpm);
}

void timeout(double t)
{
  Brain.setTimer(t, timeUnits::sec);
}

void prerequisites()
{
  frontMotorA.resetRotation();
  frontMotorB.resetRotation();
  backMotorA.resetRotation();
  backMotorB.resetRotation();
  RingScooper.resetRotation();
  mogo.resetRotation();
  FourBar.resetRotation();
  encLeft.resetRotation();
  encRight.resetRotation();
  isensor.resetHeading();
  fmg.resetPosition();
  mogo.setStopping(brakeType::hold);
  FourBar.setStopping(brakeType::hold);
  driveTrain.setStopping(brakeType::brake);
}

void autonomous()
{
  prerequisites();
  isensor.resetHeading();
  isensor.calibrate();
  while(isensor.isCalibrating()) 
  {
    wait(1, sec); //1
  }
  forwardEnc(72);
  wait(1, sec);
  turnToHeading(90);
  //spline(1, 200, 50);
  //------------------------------------------------All Testing Code---------------------------------------------------
 
  // releaseFrontMogo();
  // wait(3, sec);
  // mfm(0.575, 100, true);
  // mrm(-10, 95, true);

  //-----------------------------------------------2 Goal 40 point Game Auton 100 speed--------------------------------
  
  // m4bm(-0.1, 100, false);
  // mfm(-1.78, 50, false);
  // backwardEnc(47.5, 100);
  // piston1.open();
  // m4bm(0.2, 100, true);
  // forwardEnc(38);
  // turnToHeading(132);
  // piston1.close();
  // //mfm(-0.2, 50, true);
  // wait(0.2, sec);
  // forwardEnc(43, 75);
  // wait(0.3, sec);
  // mfm(0.67, 100, true);
  // backwardEnc(47, 75);

  //-----------------------------------------------1 Goal Elims Auton 100 speed--------------------------------

  // releaseFrontMogo();
  // forwardEnc(47, 100);
  // mfm(0.575, 100, true);
  // backwardEnc(40);

  //-----------------------------------------------Left Half Win Point Game Auton-------------------------------------------

  //WITH WINPOINT DROP
  // mwpm(0.25, 25, false);
  // turnToHeading(10, 100);
  // mwpm(-0.25, 30, true);
  // releaseFrontMogo();
  // forwardEnc(46, 100);
  // mfm(0.58, 100, true);
  // backwardEnc(30);

  //WITHOUT WINPOINT DROP
  // backwardEnc(47.5, 100);
  // piston1.open();
  // liftBackMogo();
  // forwardEnc(35);

  //-----------------------------------------------Right Half Win Point Game Auton-------------------------------------------
  
  // backwardEnc(47, 100);
  // //driveWithdistSensorbckd(100);
  // piston1.open();
  // //hookMogo();
  // liftBackMogo();
  // turnToHeading(15, 90);
  // forwardEnc(22);
  //  turnToHeading(270);
  //  forwardEnc(12);
  //  mfm(0.6, 100, true);
  // mrm(-7, 100, false);

  //-----------------------------------------------Win Point Game Auton-------------------------------------------

  // mwpm(0.25, 10, true);
  // forwardEnc(23.5, 80);
  // mwpm(-0.25, 100, false);
  // turnToHeading(88, 20);
  // forwardEnc(60);
  // releaseFrontMogo();
  // forwardEnc(32);
  // mfm(0.6, 100, true);
  // mrm(-5, 100, false);
  // backwardEnc(18);
  // piston1.close();
  // turnToHeading(180, 20);
  // backwardEnc(25);
  // hookMogo();
  // forwardEnc(35, 100);

//-------------------------------------------------Programming Skills--------------------------------------------------

// backwardEnc(14.5, 100);
// releaseFrontMogo();
// turnToHeading(323, 17);
// forwardEnc(13.5);
// mfm(0.6, 100, false);
// //liftFrontMogo();
// backwardEnc(0.25); //0.245
// turnToHeading(1, 13);
// mrm(-3, 100, false);
// m4bm(-0.5, 100, false);
// backwardEnc(44.5, 85);
// hookMogo();
// turnToHeading(35, 20); //39
// liftBackMogo();
// backwardEnc(51); //53.5
// turnToHeading(8.65, 13); //8, 8.6
// m4bm(-0.45, 65, true);
// piston1.close();
// //unhookMogo1plat();
// //backwardEnc(0.2, 100); //1
// turnToHeading(277, 20);
// releaseBackMogo();
// wait(0.3, sec);
// releaseFrontMogo();
// driveWithdistSensorbckd(85); //backwardEnc(42); //43
// piston1.open(); //hookMogo();
// m4bm(0.2, 100, false);
// forwardEnc(26.25);
// turnToHeading(322, 15); //319
// forwardEnc(100, 100);
// releaseFrontMogo();
// backwardEnc(10.35); //10.5
// turnToHeading(280, 23);  //275
// forwardEnc(12, 90);
// liftFrontMogo(true);
// //turnToHeading(276, 50);
// liftBackMogo();
// backwardEnc(55.5, 85);
// turnToHeading(180);
// backwardEnc(2.5); //3
// unhookMogo();
// turnToHeading(270);
// releaseBackMogo(); 
// forwardEnc(42.5); //38
// turnToHeading(0, 20);
// backwardEnc(45, 65);
// hookMogo();
// liftBackMogo();
// turnToHeading(321, 13); //322
// backwardEnc(28); //33.5
// m4bm(-0.5, 30, true);
// piston1.close();
// //unhookMogo1plat();
// //mrm(-5, 100, false);
// forwardEnc(8);
// releaseBackMogo();
// turnToHeading(270, 15);
// forwardEnc(25);
// turnToHeading(317, 15);
// driveWithdistSensorbckd(50); //backwardEnc(24);
// piston1.open();
// forwardEnc(20);
// m4bm(0.2, 30, false);
// turnToHeading(209, 30);
// //mfm(-1, 100, false);
// liftBackMogo();
// backwardEnc(85, 90);
// piston1.close();
// //unhookMogo1plat();
// // m4bm(-0.5, 30, true);
// // turnToHeading(160);
// // piston1.close();
// forwardEnc(30, 100);
// mfm(-1, 100, false);
// releaseBackMogo();
// backwardEnc(10, 100);
// turnToHeading(27, 75);
// driveWithdistSensorbckd(100);
// hookMogo();
// turnToHeading(335);
// liftBackMogo();
// backwardEnc(40, 100);
// piston1.close();
// forwardEnc(20, 100);


/*

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// check rot sensor dir
char left[16] = "left";
char right[16] = "right";
double rev = tryRotSensor(encLeft, left);
wait(10000, msec);
if(rev < 0)
{
Brain.Screen.print("Reverse left encoder in software \n");
// Controller1.Screen.print("Reverse left rot sensor in software \n");
}

rev = tryRotSensor(encRight, right);
wait(20000, msec);
if(rev < 0)
{
Brain.Screen.print("Reverse right encoder in software \n");
// Controller1.Screen.print("Reverse right rot sensor in software \n");
}

// check inertial sensor
//tryInertialSensor();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
}

void reset(motor &m)
{
m.resetPosition();
m.resetRotation();
} 

void initMotors(void)
{
reset(frontMotorA);
reset(frontMotorB);
reset(backMotorA);
reset(backMotorB);
reset(RingScooper);
reset(RingCache);
reset(mogo);
reset(FourBar);

encLeft.resetRotation();
encRight.resetRotation();
}


void usercontrol(void) {
// User control code here, inside the loop

// init motors
  double ax3 = Controller1.Axis3.value();
  double ax2 = Controller1.Axis2.value();

  if(abs(ax3) < 10) ax3 = 0;
  if(abs(ax2) < 10) ax2 = 0;

  if(ax3 > 90) ax3 = 100;
  if(ax3 < -90) ax3 = -100;
  if(ax2 > 90) ax2 = 100;
  if(ax2 < -90) ax2 = -100;

  while(true){

    leftDrive.spin(directionType::fwd, Controller1.Axis3.value() * 0.75, velocityUnits::pct); //(Axis3+Axis4)/2
    rightDrive.spin(directionType::fwd, Controller1.Axis2.value() * 0.75, velocityUnits::pct);//(Axis3-Axis4)/2

    // leftDrive.spin(directionType::fwd, ax3, velocityUnits::pct); //(Axis3+Axis4)/2
    // rightDrive.spin(directionType::fwd, ax2, velocityUnits::pct);//(Axis3-Axis4)/2

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 30, "Motor Temp LeftFront: %f", frontMotorA.temperature(vex::celsius));
    Brain.Screen.printAt(5, 50, "Motor Temp RightFront: %f", frontMotorB.temperature(vex::celsius));
    Brain.Screen.printAt(5, 70, "Motor Temp LeftBack: %f", backMotorA.temperature(vex::celsius));
    Brain.Screen.printAt(5, 90, "Motor Temp RightBack: %f", backMotorB.temperature(vex::celsius));
    Brain.Screen.printAt(5, 110, "revs up mogo: %f", mogo.rotation(rotationUnits::rev));
    Brain.Screen.printAt(5, 130, "Back Enc: %f", encBack.position(rotationUnits::deg));
    Brain.Screen.printAt(5, 150, "Left Enc: %f", encLeft.position(rotationUnits::deg));
    Brain.Screen.printAt(5, 170, "Rot Sensor: %f", fmg.position(rotationUnits::rev)); //0.654297
    
    wait(0.1, sec);

    //IF ELSE STARTS HERE ----------------------------------------

    if(Controller1.ButtonR1.pressing())
    {
      FourBar.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonR2.pressing())
    {
      FourBar.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    }
    else
    {
      FourBar.stop(brakeType::hold);
    }

    if(Controller1.ButtonL1.pressing())
    {
      mogo.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing())
    {
      mogo.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
    }
    else
    {
      mogo.stop(brakeType::hold);
    }

    if(Controller1.ButtonY.pressing())
    {
      piston1.close();
    }
    else
    {
      piston1.open();
    }

    if(Controller1.ButtonB.pressing())
    {
      RingScooper.spin(vex::directionType::rev, 95, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonDown.pressing())
    {
      RingScooper.stop();
    }

    if(Controller1.ButtonUp.pressing())
    {
      RingScooper.spin(vex::directionType::fwd, 95, vex::velocityUnits::pct);
    }

    if(Controller1.ButtonA.pressing())
    {
      frontMotorA.setBrake(brakeType::hold);
      frontMotorB.setBrake(brakeType::hold);
      backMotorA.setBrake(brakeType::hold);
      backMotorB.setBrake(brakeType::hold);
      leftDrive.spin(directionType::fwd, Controller1.Axis3.value(), velocityUnits::pct); //(Axis3+Axis4)/2
      rightDrive.spin(directionType::fwd, Controller1.Axis2.value(), velocityUnits::pct);//(Axis3-Axis4)/2

    }

    if(Controller1.ButtonLeft.pressing())
    {
      frontMotorA.setBrake(brakeType::coast);
      frontMotorB.setBrake(brakeType::coast);
      backMotorA.setBrake(brakeType::coast);
      backMotorB.setBrake(brakeType::coast);
      leftDrive.spin(directionType::fwd, Controller1.Axis3.value(), velocityUnits::pct); //(Axis3+Axis4)/2
      rightDrive.spin(directionType::fwd, Controller1.Axis2.value(), velocityUnits::pct);//(Axis3-Axis4)/2
    }

  } //end while


}

void pre_auton()
{
  initMotors();
  encLeft.resetRotation();
  encRight.resetRotation();
  // isensor.resetHeading();
  // isensor.calibrate();
  // while(isensor.isCalibrating()) 
  // {
  //   wait(1, sec); //1
  // }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

// init
vexcodeInit();

// Set up callbacks for autonomous and driver control periods.
Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);
// Run the pre-autonomous function.
pre_auton();

// Prevent main from exiting with an infinite loop.
while (true) {
wait(100, msec);
}
}