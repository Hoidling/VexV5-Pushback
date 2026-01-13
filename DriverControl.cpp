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
/*    Description:  6 Wheel Double Cont                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
////////////////
// BASE SETUP //
////////////////
#pragma region Base Setup

//===== ROBOT PHYSICAL MEASUREMENTS =====//
 // TODO: Measure the distance between the CENTER of your left traction wheel 
 // and the CENTER of your right traction wheel in millimeters
 const double TRACK_WIDTH_MM = 320.0; // ← MEASURE AND UPDATE THIS VALUE!

 // Wheel specifications
 // NOTE: User has 2.75" wheels (smallest VEX wheels)
 // 2.75 inches = 69.85mm diameter
 const double WHEEL_DIAMETER_MM = 69.85;  // 2.75 inches = 69.85mm
 const double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * 3.14159265359;

 // Motor specifications (Green motor = 200 RPM motor with 18:1 ratio)
 const double GEAR_RATIO = 18.0 / 1.0;  // 18:1 gearing
 const bool DIRECT_DRIVE = true;        // Motors connect directly to wheels

 // IMPORTANT: Verify which motors are connected to your MIDDLE TRACTION wheels!
 // The odometry uses leftMotorB and rightMotorB
 // PORT1 = leftMotorA, PORT2 = leftMotorB, PORT3 = leftMotorC
 // PORT4 = rightMotorA, PORT5 = rightMotorB, PORT6 = rightMotorC
 // If your middle traction wheels are on different ports, update the Odometry class!

 //===== END ROBOT MEASUREMENTS =====//


//for auton/auton setup
int autonSide = 0; // 0 = left, 1 = right

bool robotConnected = false;
double wheelTravelInches = 2.75;  // VEX 2.75" wheels (converts to mm in smartdrive)
double trackWidthMM = 320.0;      // Distance between left/right wheel centers (measure with calipers)
double wheelBaseMM = 40.0;        // Distance between front/back wheels (⚠️ verify this value)

// Motor groups
motor_group LeftDriveSmart(leftMotorA, leftMotorB, leftMotorC);
motor_group RightDriveSmart(rightMotorA, rightMotorB, rightMotorC);

// Smartdrive Configuration
// wheelTravel: 2.75" wheels = 69.85mm diameter × π = 217mm (converted from wheelTravelInches)
// trackWidth: distance between left/right wheel centers (from trackWidthMM variable)
// wheelBase: distance between front/back wheels (from wheelBaseMM variable)
smartdrive Drivetrain(LeftDriveSmart, RightDriveSmart, TheMalfunctioner, wheelTravelInches * 3.14159265359 * 25.4, trackWidthMM, wheelBaseMM, mm, 1);

competition Competition;
bool isreversed = false;
#pragma endregion Base Setup

/////////////////
// PID & ODOM  //
/////////////////
#pragma region PID_and_Odometry

//===== PID CONTROLLER =====//
class PID {
private:
  double kP, kI, kD;
  double integral = 0;
  double previousError = 0;
  double maxIntegral = 50; // Prevent integral windup
  
public:
  PID(double p, double i, double d) : kP(p), kI(i), kD(d) {}
  
  double calculate(double error) {
    // Add to integral with anti-windup
    integral += error;
    if(integral > maxIntegral) integral = maxIntegral;
    if(integral < -maxIntegral) integral = -maxIntegral;
    
    double derivative = error - previousError;
    previousError = error;
    
    return (kP * error) + (kI * integral) + (kD * derivative);
  }
  
  void reset() {
    integral = 0;
    previousError = 0;
  }
  
  void setConstants(double p, double i, double d) {
    kP = p;
    kI = i;
    kD = d;
  }
};

//===== ODOMETRY TRACKER =====//
class Odometry {
private:
  double x = 0;
  double y = 0; 
  double heading = 0; // in degrees
  
  double lastLeftPos = 0;
  double lastRightPos = 0;
  
public:
  void update() {
    // Get current encoder positions from MIDDLE traction wheels (motorB)
    double leftPos = leftMotorB.position(degrees);
    double rightPos = rightMotorB.position(degrees);
    
    // Calculate change since last update
    double deltaLeft = leftPos - lastLeftPos;
    double deltaRight = rightPos - lastRightPos;
    
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
    
    // Convert encoder degrees to millimeters traveled
    double leftMM = (deltaLeft / 360.0) * WHEEL_CIRCUMFERENCE_MM;
    double rightMM = (deltaRight / 360.0) * WHEEL_CIRCUMFERENCE_MM;
    
    // Calculate change in heading (in radians)
    double deltaHeadingRad = (rightMM - leftMM) / TRACK_WIDTH_MM;
    heading += deltaHeadingRad * (180.0 / 3.14159265359); // Convert to degrees
    
    // Normalize heading to -180 to 180
    while(heading > 180) heading -= 360;
    while(heading < -180) heading += 360;
    
    // Calculate average distance traveled
    double deltaDistance = (leftMM + rightMM) / 2.0;
    
    // Update X and Y position
    double headingRad = heading * (3.14159265359 / 180.0);
    x += deltaDistance * cos(headingRad);
    y += deltaDistance * sin(headingRad);
  }
  
  // Getters
  double getX() { return x; }
  double getY() { return y; }
  double getHeading() { return heading; }
  
  // Setters
  void setPosition(double newX, double newY, double newHeading) {
    x = newX;
    y = newY;
    heading = newHeading;
  }
  
  void reset() {
    x = 0;
    y = 0;
    heading = 0;
    lastLeftPos = leftMotorB.position(degrees);
    lastRightPos = rightMotorB.position(degrees);
  }
};

// Create global instances
Odometry odom;

// PID Tuning Constants
// IMPORTANT: These are starting values - you WILL need to tune these for your robot!
// Start conservative (low values) and gradually increase
PID drivePID(0.3, 0.0001, 0.05);  // Conservative drive PID (lower kP to reduce overshoot)
PID turnPID(0.5, 0.002, 0.1);     // Conservative turn PID (added small kI for steady-state error)

// Background task to constantly update odometry
int odometryTask() {
  while(true) {
    odom.update();
    wait(10, msec); // Update every 10ms
  }
  return 0;
}

task odometry(odometryTask); // Start the task

//===== PID DRIVE FUNCTIONS =====//

// Drive straight for a distance in MM using PID
void drivePIDDistance(double targetMM, double maxSpeed = 100) {
  drivePID.reset();
  
  double startX = odom.getX();
  double startY = odom.getY();
  
  while(true) {
    // Calculate how far we've traveled
    double currentDistance = sqrt(pow(odom.getX() - startX, 2) + pow(odom.getY() - startY, 2));
    double error = targetMM - currentDistance;
    
    // Exit if close enough
    if(fabs(error) < 5) break;
    
    // Calculate power
    double power = drivePID.calculate(error);
    
    // Limit power
    if(power > maxSpeed) power = maxSpeed;
    if(power < -maxSpeed) power = -maxSpeed;
    
    // Apply to motors
    LeftDriveSmart.spin(forward, power, percent);
    RightDriveSmart.spin(forward, power, percent);
    
    wait(20, msec);
  }
  
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}

// Turn to an absolute heading using PID
void turnPIDToHeading(double targetHeading, double maxSpeed = 80) {
  turnPID.reset();
  
  while(true) {
    double error = targetHeading - odom.getHeading();
    
    // Normalize error to -180 to 180
    while(error > 180) error -= 360;
    while(error < -180) error += 360;
    
    // Exit if close enough
    if(fabs(error) < 2) break;
    
    // Calculate power
    double power = turnPID.calculate(error);
    
    // Limit power
    if(power > maxSpeed) power = maxSpeed;
    if(power < -maxSpeed) power = -maxSpeed;
    
    // Apply to motors (opposite directions for turning)
    LeftDriveSmart.spin(forward, power, percent);
    RightDriveSmart.spin(reverse, power, percent);
    
    wait(20, msec);
  }
  
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}

// Turn relative to current heading
void turnPIDRelative(double degrees, double maxSpeed = 80) {
  double targetHeading = odom.getHeading() + degrees;
  
  // Normalize to -180 to 180
  while(targetHeading > 180) targetHeading -= 360;
  while(targetHeading < -180) targetHeading += 360;
  
  turnPIDToHeading(targetHeading, maxSpeed);
}

// Drive to a specific point on the field (curved path - turns while driving)
void driveToPoint(double targetX, double targetY, double maxSpeed = 80) {
  drivePID.reset();
  turnPID.reset();
  
  while(true) {
    // Calculate distance and angle to target
    double deltaX = targetX - odom.getX();
    double deltaY = targetY - odom.getY();
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    
    // Exit if close enough
    if(distance < 10) break;
    
    // Calculate angle to target
    double angleToTarget = atan2(deltaY, deltaX) * (180.0 / 3.14159265359);
    double angleError = angleToTarget - odom.getHeading();
    
    // Normalize angle error to -180 to 180
    while(angleError > 180) angleError -= 360;
    while(angleError < -180) angleError += 360;
    
    // Calculate motor powers
    double drivePower = drivePID.calculate(distance);
    double turnPower = turnPID.calculate(angleError);
    
    // Limit powers
    if(drivePower > maxSpeed) drivePower = maxSpeed;
    if(drivePower < -maxSpeed) drivePower = -maxSpeed;
    
    // Apply to motors (with turning correction)
    LeftDriveSmart.spin(forward, drivePower - turnPower, percent);
    RightDriveSmart.spin(forward, drivePower + turnPower, percent);
    
    wait(20, msec);
  }
  
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}

// Drive straight to a point (turns to face it first, then drives straight)
void driveStraightToPoint(double targetX, double targetY, double maxSpeed = 80) {
  // Calculate angle and distance to target
  double deltaX = targetX - odom.getX();
  double deltaY = targetY - odom.getY();
  double angleToTarget = atan2(deltaY, deltaX) * (180.0 / 3.14159265359);
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  
  // First, turn to face the target
  turnPIDToHeading(angleToTarget, maxSpeed);
  
  // Then, drive straight to it
  drivePIDDistance(distance, maxSpeed);
}

// Drive to point with specific final heading (curved path to point, then turns to heading)
void driveToPointWithHeading(double targetX, double targetY, double finalHeading, double maxSpeed = 80) {
  // Drive to the point using curved path (fastest)
  driveToPoint(targetX, targetY, maxSpeed);
  
  // Then turn to the final heading
  turnPIDToHeading(finalHeading, maxSpeed);
}

#pragma endregion PID_and_Odometry

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
  // Reset odometry at the start
  odom.reset();
  
  Drivetrain.setTurnVelocity(55, percent);
  
  // ==========================================
  // IMPORTANT: This autonomous routine is a TEMPLATE
  // You need to:
  // 1. Replace hardcoded Drivetrain commands with PID functions
  // 2. Add actual intake/outtake motor control (currently just comments)
  // 3. Test and tune paths for your specific robot
  // 4. Consider using odometry point-to-point navigation instead of relative movements
  // ==========================================
  
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
    Drivetrain.driveFor(forward, 300, mm); // Fixed: was Drivetrain.drive(forward) - requires distance
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

void noAutonSelected(){
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFont(prop40);

  Brain.Screen.drawRectangle(10, 10, 460, 220);
  Brain.Screen.printAt(165, 80, "Selection:");
  Brain.Screen.printAt(105, 155, "No Autonomous");
}

void waitForRelease(){
  while(Brain.Screen.pressing()){
    wait(20, msec);
  }
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
  
  waitForRelease();
  while(true){
    if(Brain.Screen.pressing()) {
      resetScreen(black);
      if(Brain.Screen.xPosition()<240){
        leftAutonSelected();
      }else{
        rightAutonSelected();
      }
      waitForRelease();
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

    // Reset odometry after inertial sensor is calibrated
    odom.reset();

    preAutonSelect();

    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    while(true) {
        wait(100, msec);
    }
}
#pragma endregion Main