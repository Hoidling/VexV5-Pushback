#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
controller Controller1 = controller(primary);
motor rightMotorA = motor(PORT4, ratio18_1, true);

motor rightMotorB = motor(PORT5, ratio18_1, true);

motor rightMotorC = motor(PORT6, ratio18_1, true);

motor leftMotorA = motor(PORT1, ratio18_1, false);

motor leftMotorB = motor(PORT2, ratio18_1, false);

motor leftMotorC = motor(PORT3, ratio18_1, false);

inertial TheMalfunctioner = inertial(PORT20);

motor Innity = motor(PORT7, ratio18_1, true);



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonL1/ButtonL2 status to control Innity
      if (Controller1.ButtonL1.pressing()) {
        Innity.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Innity.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Innity.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Conor Goodwill                                            */
/*    Created:      12/30/25                                                  */
/*    Description:  Winnity Comp Driving                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
////////////////
// BASE SETUP //
////////////////
#pragma region Base Setup

// Motor groups
motor_group LeftDriveSmart(leftMotorA, leftMotorB, leftMotorC);
motor_group RightDriveSmart(rightMotorA, rightMotorB, rightMotorC);

// Smartdrive
smartdrive Drivetrain(LeftDriveSmart, RightDriveSmart, TheMalfunctioner, 319.19, 320, 40, mm, 1);

competition Competition;
bool isreversed = false;
#pragma endregion Base Setup
/////////////
// DRIVING //
/////////////
#pragma region Driving
void usercontrol() {
    const int deadzone = 5;
    int leftRaw = 0;
    int rightRaw = 0;
    bool prevX = false; // track previous X button state (declare outside the loop!)

    while(true) {
        // Read the current state of the X button
        bool currX = Controller1.ButtonX.pressing();

        // Toggle isreversed only on rising edge (button just pressed)
        if(currX && !prevX) {
            isreversed = !isreversed;
        }

        // Save current button state for next iteration
        prevX = currX;

        // Assign joystick values based on reverse
        if(!isreversed){
            leftRaw  = Controller1.Axis3.position();
            rightRaw = Controller1.Axis2.position();
        } else {
            leftRaw  = Controller1.Axis2.position();
            rightRaw = Controller1.Axis3.position();
        }

        // Apply deadzone
        int leftPower  = (abs(leftRaw) > deadzone) ? leftRaw : 0;
        int rightPower = (abs(rightRaw) > deadzone) ? rightRaw : 0;

        // Reverse motor power if driving is reversed
        if(isreversed){
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        // Spin motors
        LeftDriveSmart.spin(forward, leftPower, percent);
        RightDriveSmart.spin(forward, rightPower, percent);

        wait(20, msec);
    }
}
#pragma endregion Driving
////////////////
// AUTONOMOUS //
////////////////
#pragma region Autonomous
void autonomous() {
  Drivetrain.setTurnVelocity(55, percent);
  if(autonSide == 0) { // 0 = left, 1 = right
    ///////////////
    // LEFT SIDE //
    ///////////////
    Drivetrain.driveFor(forward, 600, mm);
    Drivetrain.turnFor(left, 30, degrees);
    //start intake
    Drivetrain.driveFor(forward, 300, mm);
    //stop intake
    Drivetrain.turnFor(left, 105, degrees);
    Drivetrain.driveFor(reverse, 450, mm);
    //outtake back

    Drivetrain.driveFor(forward, 950, mm);
    Drivetrain.turnFor(left, 45, degrees);

    Drivetrain.driveFor(forward, 600, mm);
    //intake till full
    Drivetrain.driveFor(reverse, 600, mm);
    //outtake back till empty

      //repeat above till done with balls

    Drivetrain.driveFor(forward, 300, mm);
    Drivetrain.turnFor(right, 90, degrees);
    Drivetrain.driveFor(forward, 1200, mm);
    Drivetrain.turnFor(left, 45, degrees);
    //start intake
    Drivetrain.driveFor(forward, 848.53, mm);
    //stop intake
    Drivetrain.turnFor(left, 90, degrees);
    Drivetrain.driveFor(forward, 300, mm);
    //outtake front till empty

  } else {
    ////////////////
    // RIGHT SIDE //
    ////////////////
    Drivetrain.driveFor(forward, 600, mm);
    Drivetrain.turnFor(right, 30, degrees);
    //start intake
    Drivetrain.driveFor(forward, 400, mm);
    //stop intake
    Drivetrain.turnFor(left, 75, degrees);
    Drivetrain.driveFor(forward, 375, mm);
    //outtake front till empty
    Drivetrain.driveFor(reverse, 1050, mm);
    Drivetrain.turnFor(left, 150, degrees);
    Drivetrain.driveFor(forward, 150, mm);

    //intake till full
    Drivetrain.driveFor(reverse, 600, mm);
    //outtake till empty
    Drivetrain.driveFor(forward, 600, mm);

      //repeat above till done with balls

    Drivetrain.driveFor(reverse, 225, mm);
    Drivetrain.turnFor(right, 90, degrees);
    Drivetrain.driveFor(forward, 120, mm);
    Drivetrain.turnFor(right, 45, degrees);
    //start intaking
    Drivetrain.driveFor(forward, 848.53, mm);
    //stop intaking
    Drivetrain.turnFor(left, 90, degrees);
    Drivetrain.driveFor(reverse, 300, mm);
    //full outtake back
    Drivetrain.driveFor(forward, 950, mm);
    Drivetrain.turnFor(left, 45, degrees);

    Drivetrain.driveFor(forward, 600, mm);
    //intake till full
    Drivetrain.driveFor(reverse, 600, mm);
    //outtake till empty

      //repeat above till done with balls

    Drivetrain.driveFor(reverse, 600, mm);
    Drivetrain.turnFor(left, 90, degrees);
    Drivetrain.driveFor(forward, 500, mm);//should be 600 to corner tho
    Drivetrain.turnFor(right, 45, degrees);
    Drivetrain.driveFor(forward, 450, mm);//maybe a bit more bc the fwd 500 not 600
    Drivetrain.turnFor(left, 90, degrees);
    Drivetrain.drive(forward);
  }
}
void resetScreen(color Color){
  Brain.Screen.clearScreen();
  Brain.Screen.setPenWidth(0);
  Brain.Screen.setFillColor(Color);

  Brain.Screen.drawRectangle(0, 0, 480, 240);
}
void leftAutonSelected(){
  autonSide = 0; // left
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(red);
  Brain.Screen.setFont(prop40);

  Brain.Screen.drawRectangle(10, 10, 460, 220);
  Brain.Screen.printAt(170, 80, "Selection:");
  Brain.Screen.printAt(160, 155, "Left Auton");
}
void rightAutonSelected(){
  autonSide = 1; // right
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(blue);
  Brain.Screen.setFont(prop40);

  Brain.Screen.drawRectangle(10, 10, 460, 220);
  Brain.Screen.printAt(170, 80, "Selection:");
  Brain.Screen.printAt(150, 155, "Right Auton");
}
void preAutonSelect() {
 //Drawing
  //Background and Boxes
  resetScreen(black);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(red);
  Brain.Screen.drawRectangle(10, 10, 225, 220);
  Brain.Screen.setPenColor(blue);
  Brain.Screen.drawRectangle(245, 10, 225, 220);

  //Title
  Brain.Screen.setFont(prop30);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(orange);
  Brain.Screen.printAt(100,40,"Autonomous Selection");

  //Words
  Brain.Screen.setFont(prop30);
  Brain.Screen.setPenColor(red);
  Brain.Screen.printAt(90,130,"Left");
  Brain.Screen.setPenColor(blue);
  Brain.Screen.printAt(320,130,"Right");
 //Function
  
  autonomousSetup autonSetup;
  autonSetup.waitForRelease();
  while(true){
    if(Brain.Screen.pressing()) {
      resetScreen(black);
      if(Brain.Screen.xPosition()<240){
        leftAutonSelected();
      }else{
        rightAutonSelected();
      }
      autonSetup.waitForRelease();
      return;
    }
    wait(20,msec);
  }
}
#pragma endregion Autonomous
//////////
// MAIN //
//////////
#pragma region Main
int main() {
    vexcodeInit();

    Innity.setStopping(brake);
    Innity.setVelocity(100, percent);
    LeftDriveSmart.setVelocity(900, rpm);
    RightDriveSmart.setVelocity(900, rpm);
    Drivetrain.setStopping(brake);
    Drivetrain.setTurnVelocity(900, rpm);
    Drivetrain.setDriveVelocity(900, rpm);

    TheMalfunctioner.calibrate();

    while(TheMalfunctioner.isCalibrating()) {
      wait(20, msec);
    }

    preAutonSelect();

    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    while(true) {
        wait(100, msec);
    }
}
#pragma endregion Main
