using namespace vex;

extern brain Brain;

// All the SubModes
extern int autonType;
extern int ColorType;
extern int workaround;
// Other Variables
extern char *Auto;

// VEXcode devices

extern controller Controller1;

/* Motors */
  // Drivebase
  extern motor L1;
  extern motor L2;
  extern motor L3;

  extern motor R1;
  extern motor R2;
  extern motor R3;

  // Intake Motor
  extern motor Intake;

  // Lift Motor
  extern motor Lift;

/* Sensors */
  // Inertial Sensor
  extern inertial Angle;

  // Color Sensor
  extern optical Col;

  // Distance Sensors
  extern distance Dis;
  extern distance AutoClamp;

  extern distance BackDis;
  extern distance LeftDis;
  extern distance RightDis;
  extern distance FrontDis;

  extern timer AutoClampTimer;

/* Three Wire Ports */
  // Pneumatics
  extern pneumatics Clamp;
  extern pneumatics Doinker;

  extern limit DoinkerSwitch;
  extern bumper LBswitch;
/* -------------------------- */
/* Universally Used Functions */
/* -------------------------- */
// Math Functions
extern double AngleWrap(double LeAngle);
extern double Normalize(double LeAngle);
extern double ToRadians(double val);
extern double ToDegrees(double val);
extern double Sign(double val);

#ifndef MY_ENUM_H
#define MY_ENUM_H
enum Color_Sort {
  RED = 1,
  BLUE = 2
};
#endif // MY_ENUM_H

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );