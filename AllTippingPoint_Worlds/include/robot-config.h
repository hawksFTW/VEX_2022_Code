using namespace vex;

extern brain Brain;

extern motor frontMotorA;
extern motor frontMotorB;
extern motor backMotorA;
extern motor backMotorB;

extern motor_group leftDrive;
extern motor_group rightDrive;

extern smartdrive driveTrain;
extern inertial isensor;

extern encoder encLeft;
extern encoder encRight;
extern encoder encBack;

extern rotation fmg;

extern motor RingScooper;
extern motor RingCache;

extern motor mogo;
extern motor FourBar;
extern motor wp;

extern pneumatics piston1;
extern pneumatics piston2;

extern distance dSensor;

extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
