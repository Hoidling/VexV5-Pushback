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
//for auton/auton setup
bool testingMode = 1;
                                                                                              //TEST MODE ENABLED
int autonSide = 0; // 0 = left, 1 = right
enum moveOptions {
  None,
  Forward,
  Left
};
moveOptions lastMoveDirection = None;
float lastMoveDist = 0;
std::vector<std::string> finalDirectionNames;
std::vector<float> finalDirectionDistance;

const char* AUTON_EXPORT_PATH = "/usd/auton.txt";

float tile = 600;

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
void noAutonSelected(){
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFont(prop40);

  Brain.Screen.drawRectangle(10, 10, 460, 220);
  Brain.Screen.printAt(165, 80, "Selection:");
  Brain.Screen.printAt(105, 155, "No Autonomous");
}
class autonomousSetup{
public:
  void waitForRelease() {
    while (Brain.Screen.pressing()) {
      wait(10, msec);
    }
  }
  void input(float amount, moveOptions direction){
    if(direction == None || amount == 0) return;

    const char* dirName;

    if(direction == Forward){
      if(amount > 0){
        dirName = "Forward";
      } else {
        dirName = "Backward";
      }
    }
    else if(direction == Left){
      if(amount > 0){
        dirName = "Left";
      } else {
        dirName = "Right";    
      }
    }else{
      Brain.Screen.print("error");
      return;
    }

    finalDirectionNames.push_back(dirName);
    finalDirectionDistance.push_back(amount);       //hold
  }
  void record(float amount, moveOptions fwdOrLeft){//0 = Forward, 1 = Left
    if(amount == 0){
      return;
    }
    if(fwdOrLeft == Forward){
      if(lastMoveDirection==Forward){
        lastMoveDist = lastMoveDist+amount;
      }else{
        input(lastMoveDist, lastMoveDirection);
        lastMoveDist = amount;
        lastMoveDirection = fwdOrLeft;
      }
    }else if(fwdOrLeft == Left){
      if(lastMoveDirection==Left){
        lastMoveDist = lastMoveDist+amount;
      }else{
        input(lastMoveDist, lastMoveDirection);
        lastMoveDist = amount;
        lastMoveDirection = fwdOrLeft;
      }
    }else{
      Brain.Screen.printAt(10, 60, "Error: tried to save distance that was not forward or left (most likely left as value 'None')");
    }
  }
  void home(){
   //Drawing
    //Background and Boxes
    resetScreen(black);
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawRectangle(10, 10, 230, 110);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawRectangle(240, 10, 230, 110);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawRectangle(10, 130, 230, 100);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawRectangle(240, 130, 230, 100);

    //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(170,30,"Autonomous Setup");

    //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(70,80,"Forward");
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(295,80,"Backward");
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(95,190,"Left");
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(320,190,"Right");

    Brain.Screen.setFont(prop30);
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(160, 100, 160, 60);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(180,140,"View End");

   //Function
    waitForRelease();
    while(true){
      if(Brain.Screen.pressing()) {
        resetScreen(black);
        if((Brain.Screen.xPosition()>160 && Brain.Screen.xPosition()< 320)&&(Brain.Screen.yPosition()>100 && Brain.Screen.yPosition()< 160)){
          showDistances();
          break;
        }
        if(Brain.Screen.yPosition()<125){         //Top Level
          if(Brain.Screen.xPosition()<240){         //Left
            forwardOptions();
          } else {                                  //Right
            backwardOptions();
          }
        }else{                                   //Bottom Level
          if(Brain.Screen.xPosition()<240){         //Left
            leftOptions();
          } else {                                  //Right
            rightOptions();
          }
        }
        waitForRelease();
        home();
        return;
      }
      wait(20,msec);
    }
  }
  void forwardOptions(){
   //Drawing
    //Background and Boxes
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawRectangle(10, 10, (460/3), 105);
    Brain.Screen.drawRectangle(160, 10, (460/3), 105);
    Brain.Screen.drawRectangle(310, 10, (460/3), 105);

    Brain.Screen.drawRectangle(10, 125, (460/3), 105);
    Brain.Screen.drawRectangle(160, 125, (460/3), 105);
    Brain.Screen.drawRectangle(310, 125, (460/3), 105);

    //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(155,30," Forward Distance ");
    Brain.Screen.printAt(155,50,"   1 tile = 600mm     ");

    //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(50,80,"10mm");
    Brain.Screen.printAt(190,85,"50mm");
    Brain.Screen.printAt(335,80,"100mm");

    Brain.Screen.printAt(35,190,"1/2 Tile");
    Brain.Screen.printAt(200,190,"1 Tile");
    Brain.Screen.printAt(345,190,"2 Tiles");
   //Function
    waitForRelease();
    while(true){
      if(Brain.Screen.pressing()) {
        if(Brain.Screen.yPosition()<120){                //Top Level
          if(Brain.Screen.xPosition()<163){              //Left
            Drivetrain.driveFor(forward, 10, mm);
            record(10, Forward);
          } else if(Brain.Screen.xPosition()<316) {      //Middle
            Drivetrain.driveFor(forward, 50, mm);
            record(50, Forward);
          }else{                                         //Right
            Drivetrain.driveFor(forward, 100, mm);
            record(100, Forward);
          }
        }else{                                           //Bottom Level
          if(Brain.Screen.xPosition()<163){              //Left
            Drivetrain.driveFor(forward, 0.5*tile, mm);
            record(0.5*tile, Forward);
          } else if(Brain.Screen.xPosition()<316) {      //Middle
            Drivetrain.driveFor(forward, tile, mm);
            record(tile, Forward);
          }else{                                         //Right
            Drivetrain.driveFor(forward, 2*tile, mm);
            record(2*tile, Forward);
          }
        }
        waitForRelease();
        home();
        return;
      }
      wait(20,msec);
    }
  }
  void backwardOptions(){
   //Drawing
    //Background and Boxes
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawRectangle(10, 10, (460/3), 105);
    Brain.Screen.drawRectangle(160, 10, (460/3), 105);
    Brain.Screen.drawRectangle(310, 10, (460/3), 105);

    Brain.Screen.drawRectangle(10, 125, (460/3), 105);
    Brain.Screen.drawRectangle(160, 125, (460/3), 105);
    Brain.Screen.drawRectangle(310, 125, (460/3), 105);

    //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(155,30," Backward Distance ");
    Brain.Screen.printAt(155,50,"   1 tile = 600mm     ");

    //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(50,80,"10mm");
    Brain.Screen.printAt(190,85,"50mm");
    Brain.Screen.printAt(335,80,"100mm");

    Brain.Screen.printAt(35,190,"1/2 Tile");
    Brain.Screen.printAt(200,190,"1 Tile");
    Brain.Screen.printAt(345,190,"2 Tiles");
   //Function
    waitForRelease();
    while(true){
      if(Brain.Screen.pressing()) {
        if(Brain.Screen.yPosition()<120){                //Top Level
          if(Brain.Screen.xPosition()<163){              //Left
            Drivetrain.driveFor(reverse, 10, mm);
            record(-10, Forward);
          } else if(Brain.Screen.xPosition()<316) {      //Middle
            Drivetrain.driveFor(reverse, 50, mm);
            record(-50, Forward);
          }else{                                         //Right
            Drivetrain.driveFor(reverse, 100, mm);
            record(-100, Forward);
          }
        }else{                                           //Bottom Level
          if(Brain.Screen.xPosition()<163){              //Left
            Drivetrain.driveFor(reverse, 300, mm);
            record(-0.5*tile, Forward);
          } else if(Brain.Screen.xPosition()<316) {      //Middle
            Drivetrain.driveFor(reverse, 600, mm);
            record(-1*tile, Forward);
          }else{                                         //Right
            Drivetrain.driveFor(reverse, 1200, mm);
            record(-2*tile, Forward);
          }
        }
        waitForRelease();
        home();
        return;
      }
      wait(20,msec);
    }
  }
  void leftOptions(){
   //Drawing
    //Background and Boxes
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawRectangle(10, 10, (460/3), 105);
    Brain.Screen.drawRectangle((460/3)+10, 10, (460/3), 105);
    Brain.Screen.drawRectangle(((460/3)*2)+10, 10, (460/3), 105);

    Brain.Screen.drawRectangle(10, 130, 230, 100);
    Brain.Screen.drawRectangle(240, 130, 230, 100);

    //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(185,30,"Left Angling");

    //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(75,80,"5°");
    Brain.Screen.printAt(220,85,"10°");
    Brain.Screen.printAt(370,80,"45°");

    Brain.Screen.printAt(65,190,"Half Turn");
    Brain.Screen.printAt(300,190,"Full Turn");
   //Function
    
    waitForRelease();
    while(true){
      if(Brain.Screen.pressing()) {
        if(Brain.Screen.yPosition()<125){           //Top Level
          if(Brain.Screen.xPosition()<(460/3)+10){  //Left
            Drivetrain.turnFor(left, 5, degrees);
            record(5, Left);
          } else if(Brain.Screen.xPosition()<((460/3)*2)+10){                //Middle
            Drivetrain.turnFor(left, 10, degrees);
            record(10, Left);
          }else{                                    //Right
            Drivetrain.turnFor(left, 45, degrees);
            record(45, Left);
          }
        }else{                                      //Bottom Level
          if(Brain.Screen.xPosition()<240){         //Left
            Drivetrain.turnFor(left, 90, degrees);
            record(90, Left);
          } else {                                  //Right
            Drivetrain.turnFor(left, 180, degrees);
            record(180, Left);
          }
        }
        waitForRelease();
        home();
        return;
      }
      wait(20,msec);
    }
  }
  void rightOptions(){
   //Drawing
    //Background and Boxes
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawRectangle(10, 10, (460/3), 105);
    Brain.Screen.drawRectangle((460/3)+10, 10, (460/3), 105);
    Brain.Screen.drawRectangle(((460/3)*2)+10, 10, (460/3), 105);

    Brain.Screen.drawRectangle(10, 130, 230, 100);
    Brain.Screen.drawRectangle(240, 130, 230, 100);

    //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(180,30,"Right Angling");

    //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(75,80,"5°");
    Brain.Screen.printAt(220,85,"10°");
    Brain.Screen.printAt(370,80,"45°");

    Brain.Screen.printAt(65,190,"Half Turn");
    Brain.Screen.printAt(300,190,"Full Turn");
   //Function
    
    waitForRelease(); 
    while(true){
      if(Brain.Screen.pressing()) {
        if(Brain.Screen.yPosition()<125){           //Top Level
          if(Brain.Screen.xPosition()<(460/3)+10){  //Left
            Drivetrain.turnFor(right, 5, degrees);
            record(-5, Left);
          } else if(Brain.Screen.xPosition()<((460/3)*2)+10){                //Middle
            Drivetrain.turnFor(right, 10, degrees);
            record(-10, Left);
          }else{                                    //Right
            Drivetrain.turnFor(right, 45, degrees);
            record(-45, Left);
          }
        }else{                                      //Bottom Level
          if(Brain.Screen.xPosition()<240){         //Left
            Drivetrain.turnFor(right, 90, degrees);
            record(-90, Left);
          } else {                                  //Right
            Drivetrain.turnFor(right, 180, degrees);
            record(-180, Left);
          }
        }
        waitForRelease();
        home();
        return;
      }
      wait(20,msec);
    }
  }
  void showDistances(){
   //Drawing
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(240, 0, 240, 240);
    Brain.Screen.setPenColor(red);
    Brain.Screen.setFont(prop40);
    Brain.Screen.printAt(305, 110,"Return");
    Brain.Screen.printAt(310, 175,"Home");

    Brain.Screen.setFillColor(white);
    Brain.Screen.setFont(prop20);
    Brain.Screen.setPenColor(black);
   //Function
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setFont(mono20);
    // print stored moves (first to last)
    
    for (int i = 0; i < finalDirectionNames.size(); i++) {
      Brain.Screen.print("%s %.2f",finalDirectionNames.at(i).c_str(),finalDirectionDistance.at(i));
    }

    if(lastMoveDirection == Forward){
      Brain.Screen.print("%s %.2f",
        lastMoveDist > 0 ? "Forward, " : "Backwards, ",
        fabs(lastMoveDist)
      );
    }else if(lastMoveDirection == Left){
      Brain.Screen.print("%s %.2f",
        lastMoveDist > 0 ? "Left, " : "Right, ",
        fabs(lastMoveDist)
      );
    }


    // wait for user to tap Return/Home
    waitForRelease();
    while (true) {
      if (Brain.Screen.pressing() && Brain.Screen.xPosition() > 240) {
        waitForRelease();
        home();
        return;
      }
      wait(20, msec);
    }
  }
  void exportAutonToSD() {
    std::ofstream file(AUTON_EXPORT_PATH);   // use the global path

    if (!file.is_open()) {
      Brain.Screen.clearScreen();
      Brain.Screen.print("SD CARD ERROR");
      return;
    }

    file << "// ===== AUTON EXPORT =====\n\n";

    // ----- finalized moves -----
    for (int i = 0; i < finalDirectionNames.size(); i++) {
      const std::string& name = finalDirectionNames[i];
      float value = finalDirectionDistance[i];

      if (name == "Forward") {
        file << "Drivetrain.driveFor(forward, " << value << ", mm);\n";
      }
      else if (name == "Backward") {
        file << "Drivetrain.driveFor(reverse, " << fabs(value) << ", mm);\n";
      }
      else if (name == "Left") {
        file << "Drivetrain.turnFor(left, " << value << ", degrees);\n";
      }
      else if (name == "Right") {
        file << "Drivetrain.turnFor(right, " << fabs(value) << ", degrees);\n";
      }
    }

    // ----- buffered (last) move -----
    if (lastMoveDirection != None && lastMoveDist != 0) {
      file << "\n// --- last buffered move ---\n";

      if (lastMoveDirection == Forward) {
        file << "Drivetrain.driveFor(" 
            << (lastMoveDist > 0 ? "forward" : "reverse") 
            << ", " << fabs(lastMoveDist) << ", mm);\n";
      }
      else if (lastMoveDirection == Left) {
        file << "Drivetrain.turnFor("
            << (lastMoveDist > 0 ? "left" : "right") 
            << ", " << fabs(lastMoveDist) << ", degrees);\n";
      }
    }

    file << "\n// ===== END EXPORT =====\n";
    file.close();

    // feedback to user
    Brain.Screen.clearScreen();
    Brain.Screen.print("Exported to SD:");
    Brain.Screen.newLine();
    Brain.Screen.print(AUTON_EXPORT_PATH);
  }
};
void preAutonSelect() {
  if(testingMode){
   //Drawing
    //Background and Boxes
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawRectangle(10, 10, 225, 110);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawRectangle(245, 10, 225, 110);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(10, 130, 225, 100);
    Brain.Screen.setPenColor(green);
    Brain.Screen.drawRectangle(245, 130, 225, 100);

    //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(150,30,"Autonomous Selection");

    //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(100,80,"Left");
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(320,80,"Right");
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(65,190,"No Auton");
    Brain.Screen.setPenColor(green);
    Brain.Screen.printAt(280,190,"Setup Auton");
   //Function
    autonomousSetup autonSetup;
    autonSetup.waitForRelease();
    while(true){
      if(Brain.Screen.pressing()) {
        resetScreen(black);
        if(Brain.Screen.yPosition()<125){         //Top Level
          if(Brain.Screen.xPosition()<240){         //Left
            leftAutonSelected();
            autonomous();
          } else {                                  //Right
            rightAutonSelected();
            autonomous();
          }
        }else{                                   //Bottom Level
          if(Brain.Screen.xPosition()<240){         //Left
            noAutonSelected();
          } else {                                  //Right
            autonSetup.home();
          }
        }
        autonSetup.waitForRelease();
        return;
      }
      wait(20,msec);
    }
  }else{
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
