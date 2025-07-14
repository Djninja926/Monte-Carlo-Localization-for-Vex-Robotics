#include "vex.h"

using namespace vex;
/* A global instance of brain used for printing to the V5 Brain screen */
brain Brain;
/* Auton Type */
int autonType = 7;
int ColorType = BLUE;
int workaround = 0;
/* Other Variables */
char *Auto = (char*) "Testing Auto";

/* VEXcode device constructors */
controller Controller1 = controller(primary);

/* Motors */
  // Drivebase
  motor L1 = motor(PORT16, ratio6_1, true);
  motor L2 = motor(PORT14, ratio6_1, true);
  motor L3 = motor(PORT17, ratio6_1, true);

  motor R1 = motor(PORT19, ratio6_1, false);
  motor R2 = motor(PORT20, ratio6_1, false);
  motor R3 = motor(PORT18, ratio6_1, false);

  // Intake Motor
  motor Intake = motor(PORT13, ratio6_1, false);

  // Lift Motor
  motor Lift = motor(PORT12, ratio18_1, false);

/* Sensors */
  // Inertial Sensor
  inertial Angle = inertial(PORT15);

  // Color Sensor
  optical Col = optical(PORT6);

  // Distance Sensors
  distance Dis = distance(PORT5);
  distance AutoClamp = distance(PORT9);

  // For Particle Filter
  distance BackDis = distance(PORT21);
  distance LeftDis = distance(PORT8);
  distance RightDis = distance(PORT7);
  distance FrontDis = distance(PORT10);

  timer AutoClampTimer;

/* Three Wire Ports */
  // Pneumatics
  pneumatics Clamp = pneumatics(Brain.ThreeWirePort.A);
  
  pneumatics Doinker = pneumatics(Brain.ThreeWirePort.C);
  limit DoinkerSwitch = limit(Brain.ThreeWirePort.D);
  bumper LBswitch = bumper(Brain.ThreeWirePort.H);

/* -------------------------- */
/* Universally Used Functions */
/* -------------------------- */
/* Math Functions */
// Degrees to Radians
double ToRadians(double val) {
  return val * (M_PI / 180.0);
}
// Radians to Degrees
double ToDegrees(double val) {
  return val * (180.0 / M_PI);
}
// Positve or Negative
double Sign(double val) {
  return (val > 0) ? 1 : ((val < 0) ? -1 : 0);
}

double AngleWrap(double LeAngle) {
  // Everything is in radians
  while(LeAngle < 0) {
    LeAngle += 2.0 * M_PI;
  }
  while (LeAngle > (2 * M_PI)) {
    LeAngle -= 2.0 * M_PI;
  }
  return LeAngle;
}

double Normalize(double LeAngle) {
  // Everything is in radians
  while(LeAngle < -M_PI) {
    LeAngle += 2.0 * M_PI;
  }
  while (LeAngle > M_PI) {
    LeAngle -= 2.0 * M_PI;
  }
  return LeAngle;
}
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