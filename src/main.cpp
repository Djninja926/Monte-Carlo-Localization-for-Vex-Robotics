/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "cmath"
#include "api.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
int IntakeState = 0;
bool ColorSortOn = true;

void ColorSorting(void) {
  // Within 5 millimeters
  while (true) {
    int val = Col.hue();
    if (ColorSortOn) { 
      if (ColorType == RED) {
        // Ejects BLUE Rings
        if (val >= 210 && val < 240 && Dis.value() < 50 && IntakeState == 0) {
          cout << "SORTING SORTING" << endl;
          thread([]{
            IntakeState = 1;
            Intake.spin(fwd, 12, volt);
            double startPos = Intake.position(rev);
            waitUntil(.56 - (Intake.position(rev) - startPos) < .06);

            Intake.spin(reverse, 12, volt);
            this_thread::sleep_for(300); // 600 is GOOD
            Intake.spin(fwd, 12, volt);
            IntakeState = 0;
          });
        }
      } else if (ColorType == BLUE) {
        // Ejects RED Rings
        if ((val > 340 || val < 20) && Dis.value() < 45 && IntakeState == 0) {
          thread([]{
            IntakeState = 1;
            Intake.spin(fwd, 12, volt);
            double startPos = Intake.position(rev);
            waitUntil(.56 - (Intake.position(rev) - startPos) < .1);


            Intake.spin(reverse, 12, volt);
            this_thread::sleep_for(300); // 600 is GOOD
            Intake.spin(fwd, 12, volt);
            IntakeState = 0;
          });
        }
      }
    }
    wait(1, msec);
  }
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Initialize the LB Rotation Sensor
  Lift.setPosition(10, deg);

  // Calibrate the Inertial Sensor
  Angle.calibrate();
  while (Angle.isCalibrating()) {
    wait(1, msec);
  }

  // Exrta Stuff
  MCL::StartMCL(0, -60, 0);
  Col.setLight(ledState::on);
  Col.setLightPower(100);
  Controller1.rumble("-");
  Col.integrationTime(5);

  /* Starting Threads */
  thread(ColorSorting).detach();
  thread(Draw::DrawField).detach();
  thread(MCL::MonteCarlo).detach();
  thread(PurePursuit::PurePursuitAlgo).detach();
  while (true) {
    thread(LadyBrown::LadyBrownProfile).detach();
    if (Competition.isAutonomous() && !PurePursuit::StartPurePursuit) {
      thread(DrivePID::DrivePid).detach();
      thread(TurnPID::TurnPid).detach();
      thread(Boom::Boomerang).detach();
    }

    if (LBswitch.pressing()) {
      Lift.setPosition(10, deg);
    }

    // cout << DoinkerSwitch.value() << endl;


    wait(5, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  Auton::RunAuton();
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

// void ScoreGoal(void) {
//   IntakeState = 1;
//   Intake.spin(reverse, 12, volt);
//   // wait(250, msec);
//   Clamp.close();
//   wait(250, msec);
//   IntakeState = 0;
//   Intake.spin(fwd, 12, volt);
// }

void usercontrol(void) {
  // Stopping threads from Auton
  // thread(PurePursuit::PurePursuitAlgo).interrupt();
  // thread(DrivePID::DrivePid).interrupt();
  // thread(TurnPID::TurnPid).interrupt();
  // thread(Ramsete::Ramsete).interrupt();
  // thread(Boom::Boomerang).interrupt();
  // thread(Odom::Odom).interrupt();

  // Drivebase Variables
  double RightJoystick = 0;
  double LeftJoystick = 0;
  double Angular = 0;
  double Linear = 0;

  // Booleans
  bool HasClamped = false;
  int ClampState = 0;
  bool ClampStateBool = false;
  int LadyBrownState = 0;
  
  bool ButtonRightState = false;
  bool ButtonYState = false;
  bool ButtonDownState = false;
  bool ButtonBState = false;
  bool ButtonLeftState = false;
  
  bool AutoClampState = false;
  bool CoastLB = false;

  IntakeState = 0;
  ColorSortOn = true;
  Col.setLight(ledState::on);

  // Reseting Brain timer
  Brain.resetTimer();
  while (true) {
    // Setting the values from the joysticks 
    LeftJoystick = Controller1.Axis3.position();
    RightJoystick = Controller1.Axis1.position();
    // Setting the Exponential Values
    Linear = pow(LeftJoystick, 3) / 10000;
    Angular = pow(RightJoystick, 3) / 10000;
    // Spinning the Wheels
    if (LeftJoystick > -5 && LeftJoystick < 5 && RightJoystick > -5 && RightJoystick < 5) {
      L1.stop(brake); R1.stop(brake);
      L2.stop(brake); R2.stop(brake);
      L3.stop(brake); R3.stop(brake);
    } else {
      L1.spin(fwd, Linear + Angular, pct); R1.spin(fwd, Linear - Angular, pct);
      L2.spin(fwd, Linear + Angular, pct); R2.spin(fwd, Linear - Angular, pct);
      L3.spin(fwd, Linear + Angular, pct); R3.spin(fwd, Linear - Angular, pct);
    }

    if (Controller1.ButtonRight.pressing() && !ButtonRightState) {
      if (LadyBrownState == 0) {
        LadyBrownState = 1;
        LadyBrown::LadyBrownTwo();
      } else {
        LadyBrownState = 0;
        LadyBrown::LadyBrownOne();
      }
      CoastLB = false;
      ButtonRightState = true;
     } else if (!Controller1.ButtonRight.pressing() && ButtonRightState) {
      ButtonRightState = false;
    }


    // Intake Controls
    if (IntakeState == 0) {
      if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) {
        if (!LadyBrownState) {
          LadyBrown::LadyBrownOne();
        } else {
          LadyBrown::LadyBrownTwo();
        }
        CoastLB = false;
        Intake.spin(fwd, 12, volt);
      } else if (Controller1.ButtonL1.pressing()) {
        Intake.spin(fwd, 12, volt);
      } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(reverse, 12, volt);
      } else {
        Intake.stop(coast);
      }
    }

    // Filthy MONKEY
    // Toggle Color Sort
    if (Controller1.ButtonLeft.pressing() && !ButtonLeftState) {
      if (ColorSortOn == 0) {
        ColorSortOn = 1;
        Col.setLight(ledState::on);
      } else {
        ColorSortOn = 0;
        Col.setLight(ledState::off);
      }
      ButtonLeftState = true;
     } else if (!Controller1.ButtonLeft.pressing() && ButtonLeftState) {
      ButtonLeftState = false;
    }

    // Lift Controls
    if (Controller1.ButtonR1.pressing()) {
      Lift.spin(fwd, 12, volt);
      LadyBrown::startLadyBrownMP = false;
      CoastLB = true;
    } else if (Controller1.ButtonR2.pressing()) {
      Lift.spin(reverse, 12, volt);
      LadyBrown::startLadyBrownMP = false;
      CoastLB = true;
    } else if (!Controller1.ButtonL1.pressing() || !Controller1.ButtonL2.pressing()) {
      if (CoastLB) Lift.stop(coast);
      // LadyBrown::startLadyBrownMP = false;
    }

    /* Pneumatics */
    // if (LBswitch.pressing()) {
    //   Lift.setPosition(10, deg);
    // }

    if (!Controller1.ButtonB.pressing()) {
      AutoClampTimer.reset();

      HasClamped = false;

      if (ClampState == 1) {
        Clamp.open();
        Controller1.rumble(".");
      } else if (ClampState == 2) {
        Clamp.close();
      }

      ClampState = 0;
    }

    if (AutoClampTimer.time() < 250) {
      // Normal Controls
      if ((Controller1.ButtonB.pressing() && !ButtonYState)) {
        if (Clamp.value() == 0) {
          ClampState = 1;
        } else {
          ClampState = 2;
        }
        ButtonYState = true;
      } else if (!Controller1.ButtonB.pressing() && ButtonYState) {
        ButtonYState = false;
      }
    } else {
      // Auto Clamp
      ClampState = 0;

      if (Clamp.value() == 1 && !HasClamped) {
        Clamp.close();
        HasClamped = true;
      }

      if (AutoClamp.value() < 90 && !AutoClampState && !ClampStateBool) {
        Clamp.open();
        Controller1.rumble(".");
        AutoClampState = true;
        HasClamped = true;
      }
    }

    if (AutoClamp.value() > 100 && AutoClampState) {
      AutoClampState = false;
    }

    if (AutoClamp.value() < 100) {
      ClampStateBool = false;
    } else {
      ClampStateBool = true;
    }
    
    
    /* CLAMP */


    if (Controller1.ButtonDown.pressing() && !ButtonDownState) {
      LadyBrown::LadyBrownHold();
      CoastLB = false;
      ButtonDownState = true;
     } else if (!Controller1.ButtonDown.pressing() && ButtonDownState) {
      ButtonDownState = false;
    }

    if (Controller1.ButtonY.pressing() && !ButtonBState) {
      if (Doinker.value() == 0) {
        Doinker.open();
      } else {
        Doinker.close();
      }
      ButtonBState = true;
     } else if (!Controller1.ButtonY.pressing() && ButtonBState) {
      ButtonBState = false;
    }

    wait(20, msec);
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
    wait(100, msec);
  }
}