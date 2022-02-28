/*----------------------------------------------------------------------------
*
* Copyright (C) 2021 Team 10F - All Rights Reserved
* Any other team NOT use, distribute, or modify this code 
* as per the terms of VEX honor code.
*
*---------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// *** CHECK ALL PORTS AND GEAR RATIOS AND REVERSE FLAGS ***

// VEXcode device constructors
motor frontMotorA = motor(PORT1, ratio6_1, true); //FrontLeft
motor frontMotorB = motor(PORT5, ratio6_1, false); //FrontRight 

motor backMotorA = motor(PORT2, ratio6_1, true); //BackLeft
motor backMotorB = motor(PORT3, ratio6_1, false); //BackRight

// motor groups
motor_group leftDrive = motor_group(frontMotorA, backMotorA);
motor_group rightDrive = motor_group(frontMotorB, backMotorB);

inertial isensor = inertial(PORT19);
smartdrive driveTrain = smartdrive(leftDrive, rightDrive, isensor, 12.566367061, /*trackWidth*/ 14.5, /*wheelBase*/ 11, distanceUnits::in, 1);
// smartdrive driveTrain = smartdrive(leftDrive, rightDrive, isensor, 12.566367061, 14.6457, 1.5748, distanceUnits::in, 0.42857142857142855);

encoder encLeft = encoder(Brain.ThreeWirePort.A); // and H
encoder encRight = encoder(Brain.ThreeWirePort.G); // and D
rotation rotFront = rotation(PORT12);

// ring scooper and stacker
motor ringScooper = motor(PORT6, ratio6_1, true);
motor RingCache = motor(PORT8, ratio36_1, true); 

// front mobile goal motor (front right)
motor tilter = motor(PORT16, ratio36_1, false); 

// four bar motor
motor fourBar = motor(PORT7, ratio36_1, true);
motor wp = motor(PORT13, ratio18_1, true);
// pneumatic pistons
pneumatics hook = pneumatics(Brain.ThreeWirePort.D);

distance dSensor = distance(PORT17);
distance dSensor_fwd = distance(PORT14);

controller Controller1 = controller(primary);


// VEXcode generated functions

/**
* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
* 
* This should be called at the start of your int main function.
*/
void vexcodeInit( void ) {
Brain.Screen.print("Device initialization...\n");
Brain.Screen.setCursor(2, 1);
// calibrate the drivetrain gyro
wait(200, msec);
// isensor.calibrate();
// wait for the gyro calibration process to finish
while (isensor.isCalibrating())
{
Brain.Screen.print("Calibrating Inertial Sensor for Drivetrain\n");
Controller1.Screen.print("Calibrating Inertial Sensor for Drivetrain\n");
wait(200, msec);
}

// reset the screen now that the calibration is complete
Brain.Screen.clearScreen();
Brain.Screen.setCursor(3,1);
}



// /*----------------------------------------------------------------------------
// *
// * Copyright (C) 2021 Team 10F - All Rights Reserved
// * Any other team NOT use, distribute, or modify this code 
// * as per the terms of VEX honor code.
// *
// *---------------------------------------------------------------------------*/

// #include "vex.h"

// using namespace vex;
// using signature = vision::signature;
// using code = vision::code;

// // A global instance of brain used for printing to the V5 Brain screen
// brain Brain;

// // *** CHECK ALL PORTS AND GEAR RATIOS AND REVERSE FLAGS ***

// // VEXcode device constructors
// motor frontMotorA = motor(PORT1, ratio6_1, true); //FrontLeft
// motor frontMotorB = motor(PORT5, ratio6_1, false); //FrontRight 

// motor backMotorA = motor(PORT2, ratio6_1, true); //BackLeft
// motor backMotorB = motor(PORT3, ratio6_1, false); //BackRight

// // motor groups
// motor_group leftDrive = motor_group(frontMotorA, backMotorA);
// motor_group rightDrive = motor_group(frontMotorB, backMotorB);

// inertial isensor = inertial(PORT19);
// smartdrive driveTrain = smartdrive(leftDrive, rightDrive, isensor, 12.566367061, /*trackWidth*/ 14.5, /*wheelBase*/ 11.0, distanceUnits::in, 1);

// encoder encLeft = encoder(Brain.ThreeWirePort.A); // and H
// encoder encRight = encoder(Brain.ThreeWirePort.G); // and D
// encoder encBack = encoder(Brain.ThreeWirePort.E); // and B
// rotation fmg = rotation(PORT16);

// // ring scooper and stacker
// motor RingScooper = motor(PORT15, ratio18_1, true);
// motor RingCache = motor(PORT8, ratio36_1, true); 

// // front mobile goal motor (front right)
// motor mogo = motor(PORT5, ratio36_1, false); 

// // four bar motor
// motor FourBar = motor(PORT14, ratio36_1, true);
// motor wp = motor(PORT7, ratio18_1, true);
// // pneumatic pistons
// pneumatics piston1 = pneumatics(Brain.ThreeWirePort.D);

// distance dSensor = distance(PORT3);

// controller Controller1 = controller(primary);


// // VEXcode generated functions

// /**
// * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
// * 
// * This should be called at the start of your int main function.
// */
// void vexcodeInit( void ) {
// Brain.Screen.print("Device initialization...\n");
// Brain.Screen.setCursor(2, 1);
// // calibrate the drivetrain gyro
// wait(200, msec);
// // isensor.calibrate();
// // wait for the gyro calibration process to finish
// while (isensor.isCalibrating())
// {
// Brain.Screen.print("Calibrating Inertial Sensor for Drivetrain\n");
// Controller1.Screen.print("Calibrating Inertial Sensor for Drivetrain\n");
// wait(200, msec);
// }

// // reset the screen now that the calibration is complete
// Brain.Screen.clearScreen();
// Brain.Screen.setCursor(3,1);
// }
