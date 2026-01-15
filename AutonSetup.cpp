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
#include "vex_competition.h"
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

//CHANGE IF U GOT A ROBOT

bool robotConnected = false;
double wheelTravelInches = 2.75;  // VEX 2.75" wheels (converts to mm in smartdrive)
double trackWidthMM = 320.0;      // Distance between left/right wheel centers (measure with calipers)
double wheelBaseMM = 40.0;        // Distance between front/back wheels (⚠️ verify this value)

#pragma region Base Setup
//for auton/auton setup
int autonSide = 0; // 0 = left, 1 = right

enum moveOptions {
  None,
  Forward,
  Left
};
moveOptions lastMoveDirection = None;

enum screenShown {
  BaseHome,
  SetupHome,
  LeftAuton,
  RightAuton,
  NoAuton,
  SetupFront,
  SetupBack,
  SetupLeft,
  SetupRight,
  ShowDistance
};
screenShown currentScreen = BaseHome;

enum Selection {
  NA,
  TL,
  TM,
  TR,
  MM,
  BL,
  BM,
  BR
};
Selection currentSelection;

int topButtonNumber = 2;
int bottomButtonNumber = 2;
bool isMiddleButton = 0;
bool autonChosen = false;

float lastMoveDist = 0;
std::vector<std::string> finalDirectionNames;
std::vector<float> finalDirectionDistance;

const char* AUTON_EXPORT_PATH = "A:/auton.txt";

float tile = 600;

bool exportSuccessShown = false;  // Track if export message has been shown

// Motor groups
motor_group LeftDriveSmart(leftMotorA, leftMotorB, leftMotorC);
motor_group RightDriveSmart(rightMotorA, rightMotorB, rightMotorC);

// Smartdrive Configuration
// wheelTravel: 2.75" wheels = 69.85mm diameter × π = 217mm (converted from wheelTravelInches)
// trackWidth: distance between left/right wheel centers (from trackWidthMM variable)
// wheelBase: distance between front/back wheels (from wheelBaseMM variable)
smartdrive Drivetrain(LeftDriveSmart, RightDriveSmart, TheMalfunctioner, wheelTravelInches * 3.14159265359 * 25.4, trackWidthMM, wheelBaseMM, mm, 1);

competition Competition;
#pragma endregion Base Setup
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
  } else {
    ////////////////
    // RIGHT SIDE //
    ////////////////
  }
}
void resetScreen(color Color){
  Brain.Screen.setPenColor(Color);
  Brain.Screen.setFillColor(Color);
  Brain.Screen.clearScreen();
  Brain.Screen.drawRectangle(0, 0, 480, 240);
}
void waitForRelease() {
  while (Brain.Screen.pressing()) {
    wait(10, msec);
  }
}
void waitForPress() {
  while (!Brain.Screen.pressing()) {
    wait(10, msec);
  }
}
void startLoop(){
  Brain.Screen.printAt(0,100,"Loop Entered");
  while (true){
    wait(100, msec);
  }
}
void input(float amount, moveOptions direction){
  if(direction == None || amount == 0) return;

  std::string dirName;

  if(direction == Forward){
    if(amount > 0){
      dirName = "Forward";
    } else {
      dirName = "Backward";
    }
  }else if(direction == Left){
    if(amount > 0){
      dirName = "Left";
    } else {
      dirName = "Right";
    }
  }else{
    Brain.Screen.print("Error: Invalid Direction In 'void input()'");
    startLoop();
    return;
  }

  finalDirectionNames.push_back(dirName);
  finalDirectionDistance.push_back(amount);
}
void record(float amount, moveOptions fwdOrLeft){
  if(amount == 0){
    return;
  }
  if(fwdOrLeft == Forward){
    // If last move was also Forward type (including backward), combine them
    if(lastMoveDirection==Forward){
      lastMoveDist = lastMoveDist+amount;
      // If they cancel out to zero, reset
      if(lastMoveDist == 0){
        lastMoveDirection = None;
        lastMoveDist = 0;
      }
    }else{
      // Save previous move (if any) and start new buffer
      input(lastMoveDist, lastMoveDirection);
      lastMoveDist = amount;
      lastMoveDirection = fwdOrLeft;
    }
  }else if(fwdOrLeft == Left){
    // If last move was also Left type (including right), combine them
    if(lastMoveDirection==Left){
      lastMoveDist = lastMoveDist+amount;
      // If they cancel out to zero, reset
      if(lastMoveDist == 0){
        lastMoveDirection = None;
        lastMoveDist = 0;
      }
    }else{
      // Save previous move (if any) and start new buffer
      input(lastMoveDist, lastMoveDirection);
      lastMoveDist = amount;
      lastMoveDirection = fwdOrLeft;
    }
  }else{
    Brain.Screen.printAt(10, 60, "Error: tried to save distance that was not forward or left (most likely left as value 'None')");
  }
}
class DrawButton{
 public:
  void t2b2(color colortl, color colortr, color colorbl, color colorbr){
    // For 2x2 layout:
    // Screen width: 480px, 10px border each side = 460px available
    // 2 buttons horizontally with 10px gap between them
    // Each button width: (460 - 10) / 2 = 225px
    // Positions: Left: 10, Right: 10 + 225 + 10 = 245
    
    // Screen height: 240px, 10px border top/bottom = 220px available
    // Split into top/bottom sections
    // Top section height: 110px (for text)
    // Bottom section height: 100px
    // Gap: 240 - 10 - 110 - 10 - 100 - 10 = 0 (perfect!)
    
   //top
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(colortl);
    Brain.Screen.drawRectangle(10, 10, 225, 110);
    Brain.Screen.setPenColor(colortr);
    Brain.Screen.drawRectangle(245, 10, 225, 110);
    
   //bottom
    Brain.Screen.setPenColor(colorbl);
    Brain.Screen.drawRectangle(10, 130, 225, 100);
    Brain.Screen.setPenColor(colorbr);
    Brain.Screen.drawRectangle(245, 130, 225, 100);
  }
  
  void t3b2(color colortl, color colortm, color colortr, color colorbl, color colorbr){
   //top - using 147px width
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(colortl);
    Brain.Screen.drawRectangle(10, 10, 147, 110);
    Brain.Screen.setPenColor(colortm);
    Brain.Screen.drawRectangle(167, 10, 147, 110);
    Brain.Screen.setPenColor(colortr);
    Brain.Screen.drawRectangle(324, 10, 147, 110);
    
   //bottom - 2 buttons, 225px each
    Brain.Screen.setPenColor(colorbl);
    Brain.Screen.drawRectangle(10, 130, 225, 100);
    Brain.Screen.setPenColor(colorbr);
    Brain.Screen.drawRectangle(245, 130, 225, 100);
  }
  
  void t2b3(color colortl, color colortr, color colorbl, color colorbm, color colorbr){
    // For 2 top buttons, 3 bottom buttons:
    
   //top - 2 buttons
    resetScreen(black);
    Brain.Screen.setPenColor(colortl);
    Brain.Screen.drawRectangle(10, 10, 225, 110);
    Brain.Screen.setPenColor(colortr);
    Brain.Screen.drawRectangle(245, 10, 225, 110);
    
   //bottom - 3 buttons
    Brain.Screen.setPenWidth(2);
    
    // Bottom button width: (460 - 20) / 3 = 440/3 ≈ 146.67px ≈ 147px
    Brain.Screen.setPenColor(colorbl);
    Brain.Screen.drawRectangle(10, 130, 147, 100);
    Brain.Screen.setPenColor(colorbm);
    Brain.Screen.drawRectangle(167, 130, 147, 100);
    Brain.Screen.setPenColor(colorbr);
    Brain.Screen.drawRectangle(324, 130, 147, 100);
  }
  
  void t3b3(color colortl, color colortm, color colortr, color colorbl, color colorbm, color colorbr){
    // For 3x3 layout:
    // Available width: 480 - 20 = 460px
    // 3 buttons with 2 gaps of 10px each = 460 - 20 = 440px
    // Each button width: 440 / 3 ≈ 146.67px ≈ 147px
    
    // Positions:
    // Left: 10
    // Middle: 10 + 147 + 10 = 167
    // Right: 167 + 147 + 10 = 324
    
    //top
    resetScreen(black);
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setPenColor(colortl);
    Brain.Screen.drawRectangle(10, 10, 147, 110);   // Left: 10-157
    Brain.Screen.setPenColor(colortm);
    Brain.Screen.drawRectangle(167, 10, 147, 110);  // Middle: 167-314
    Brain.Screen.setPenColor(colortr);
    Brain.Screen.drawRectangle(324, 10, 147, 110);  // Right: 324-471
    
    //bottom
    Brain.Screen.setPenColor(colorbl);
    Brain.Screen.drawRectangle(10, 130, 147, 100);   // Left: 10-157
    Brain.Screen.setPenColor(colorbm);
    Brain.Screen.drawRectangle(167, 130, 147, 100);  // Middle: 167-314
    Brain.Screen.setPenColor(colorbr);
    Brain.Screen.drawRectangle(324, 130, 147, 100);  // Right: 324-471
  }
};
DrawButton drawButton;
// Forward declare exportAutonToSD
void exportAutonToSD();

class DrawScreen{
 public:
  void BaseHome(){
    resetScreen(black);
    topButtonNumber = 2;
    bottomButtonNumber = 2;
    isMiddleButton = true;
      drawButton.t2b2(red, blue, white, green);
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
  }
  void SetupHome(){
    resetScreen(black);
    topButtonNumber = 2;
    bottomButtonNumber = 2;
    isMiddleButton = true;
    drawButton.t2b2(red, red, blue, blue);

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
    
    // Middle button for "View Log" - black fill with white text
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(160, 90, 160, 70);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(prop30);
    Brain.Screen.printAt(185,135,"View Log");
  }
  void leftAutonomous(){
    resetScreen(black);
    autonChosen = true;
    autonSide = 0; // left
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(red);
    Brain.Screen.setFont(prop40);

    Brain.Screen.drawRectangle(10, 10, 460, 220);
    Brain.Screen.printAt(170, 80, "Selection:");
    Brain.Screen.printAt(160, 155, "Left Auton");
  }
  void rightAutonomous(){
    resetScreen(black);
    autonChosen = true;
    autonSide = 1; // right
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.setFont(prop40);

    Brain.Screen.drawRectangle(10, 10, 460, 220);
    Brain.Screen.printAt(170, 80, "Selection:");
    Brain.Screen.printAt(150, 155, "Right Auton");
  }
  void noAutonomous(){
    resetScreen(black);
    autonChosen = true;
    Brain.Screen.setPenWidth(2);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(prop40);

    Brain.Screen.drawRectangle(10, 10, 460, 220);
    Brain.Screen.printAt(165, 80, "Selection:");
    Brain.Screen.printAt(105, 155, "No Autonomous");
  }
  void forwardSetup(){
    resetScreen(black);
    topButtonNumber = 3;
    bottomButtonNumber = 3;
    isMiddleButton = false;
    drawButton.t3b3(red, red, red, red, red, red);
   //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(155,30," Forward Distance ");
    Brain.Screen.printAt(155,50,"   1 tile = 600mm     ");

   //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(55,80,"10mm");      // Center in left button (10-157)
    Brain.Screen.printAt(205,85,"50mm");     // Center in middle button (167-314)
    Brain.Screen.printAt(355,80,"100mm");    // Center in right button (324-471)

    Brain.Screen.printAt(50,190,"1/2 Tile"); // Center in left bottom button
    Brain.Screen.printAt(205,190,"1 Tile");  // Center in middle bottom button
    Brain.Screen.printAt(360,190,"2 Tiles"); // Center in right bottom button
  }
  void backwardSetup(){
    resetScreen(black);
    topButtonNumber = 3;
    bottomButtonNumber = 3;
    isMiddleButton = false;
    drawButton.t3b3(red, red, red, red, red, red);
   //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(155,30," Backward Distance ");
    Brain.Screen.printAt(155,50,"   1 tile = 600mm     ");

   //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(55,80,"10mm");      // Center in left button (10-157)
    Brain.Screen.printAt(205,85,"50mm");     // Center in middle button (167-314)
    Brain.Screen.printAt(355,80,"100mm");    // Center in right button (324-471)

    Brain.Screen.printAt(50,190,"1/2 Tile"); // Center in left bottom button
    Brain.Screen.printAt(205,190,"1 Tile");  // Center in middle bottom button
    Brain.Screen.printAt(360,190,"2 Tiles"); // Center in right bottom button
  }
  void leftSetup(){
    resetScreen(black);
    topButtonNumber = 3;
    bottomButtonNumber = 2;
    isMiddleButton = false;
    drawButton.t3b2(blue, blue, blue, blue, blue);
   //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(185,30,"Left Angling");

   //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(blue);
    // Top buttons (147px wide):
    Brain.Screen.printAt(75,80,"5°");     // Was 75, should be ~80 to center in 147px
    Brain.Screen.printAt(220,85,"10°");   // Was 220, should be ~225 to center
    Brain.Screen.printAt(370,80,"45°");   // Was 370, should be ~375 to center

    // Bottom buttons (225px wide):
    Brain.Screen.printAt(80,190,"Half Turn");   // Center in 225px
    Brain.Screen.printAt(300,190,"Full Turn");  // Center in 225px
  }
  void rightSetup(){
    resetScreen(black);
    topButtonNumber = 3;
    bottomButtonNumber = 2;
    isMiddleButton = false;
    drawButton.t3b2(blue, blue, blue, blue, blue);
   //Title
    Brain.Screen.setFont(prop20);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(180,30,"Right Angling");

   //Words
    Brain.Screen.setFont(prop30);
    Brain.Screen.setPenColor(blue);
    // Top buttons (147px wide):
    Brain.Screen.printAt(75,80,"5°");     // Was 75, should be ~80 to center in 147px
    Brain.Screen.printAt(220,85,"10°");   // Was 220, should be ~225 to center
    Brain.Screen.printAt(370,80,"45°");   // Was 370, should be ~375 to center

    // Bottom buttons (225px wide):
    Brain.Screen.printAt(80,190,"Half Turn");   // Center in 225px
    Brain.Screen.printAt(300,190,"Full Turn");  // Center in 225px
  }
  void showDistance(){
    resetScreen(white);
    isMiddleButton = false;

    Brain.Screen.setPenWidth(2);
    
    //big button
    Brain.Screen.setPenColor(red);
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(240, 1, 238, 237);
    Brain.Screen.setFont(prop30);
    Brain.Screen.printAt(275, 75, "Return Home");
    //export button
    Brain.Screen.setPenColor(blue);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.drawRectangle(252, 130, 214, 96);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.setFont(prop30);
    Brain.Screen.printAt(315, 190, "Export");

    Brain.Screen.setFillColor(white);
    Brain.Screen.setFont(prop20);
    Brain.Screen.setPenColor(black);
    
    // Function - display the movement log
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setFont(mono20);
    
    // Print stored moves (first to last)
    for (int i = 0; i < finalDirectionNames.size(); i++) {
      Brain.Screen.print("%d: %s %.1f", 
        i+1,
        finalDirectionNames.at(i).c_str(),
        fabs(finalDirectionDistance.at(i))
      );
      Brain.Screen.newLine();
    }

    // Print the buffered (last) move if it exists
    if(lastMoveDirection != None && lastMoveDist != 0){
      Brain.Screen.print("%d: %s %.1f",
        finalDirectionNames.size() + 1,
        lastMoveDist > 0 ? (lastMoveDirection == Forward ? "Forward" : "Left") 
                         : (lastMoveDirection == Forward ? "Backward" : "Right"),
        fabs(lastMoveDist)
      );
      Brain.Screen.newLine();
    }
    
    // Show success message if export was successful
    if(exportSuccessShown){
      Brain.Screen.setPenColor(green);
      Brain.Screen.setFont(mono15);
      Brain.Screen.printAt(10, 230, "Successfully exported");
    }
  }
};
void exportAutonToSD() {
  std::ofstream file(AUTON_EXPORT_PATH);

  if (!file.is_open()) {
    Brain.Screen.clearScreen();
    Brain.Screen.print("SD CARD ERROR");
    wait(2, seconds);
    exportSuccessShown = false;
    return;
  }

  file << "// ===== AUTONOMOUS ROUTINE EXPORT =====\n";
  file << "// Copy and paste into your autonomous() function\n\n";

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
    if (lastMoveDirection == Forward) {
      if(lastMoveDist > 0){
        file << "Drivetrain.driveFor(forward, " << lastMoveDist << ", mm);\n";
      } else {
        file << "Drivetrain.driveFor(reverse, " << fabs(lastMoveDist) << ", mm);\n";
      }
    }
    else if (lastMoveDirection == Left) {
      if(lastMoveDist > 0){
        file << "Drivetrain.turnFor(left, " << lastMoveDist << ", degrees);\n";
      } else {
        file << "Drivetrain.turnFor(right, " << fabs(lastMoveDist) << ", degrees);\n";
      }
    }
  }

  file << "\n// ===== END EXPORT =====\n";
  file.close();

  // Set flag to show success message
  exportSuccessShown = true;
}



void autonSetup() {
  DrawScreen drawScreen;
  while(!autonChosen){
    resetScreen(black);
    switch (currentScreen) {
      case BaseHome:
        drawScreen.BaseHome();
        break;
      case SetupHome:
        drawScreen.SetupHome();
        break;
      case LeftAuton:
        drawScreen.leftAutonomous();
        break;
      case RightAuton:
        drawScreen.rightAutonomous();
        break;
      case NoAuton:
        drawScreen.noAutonomous();
        break;
      case SetupFront:
        drawScreen.forwardSetup();
        break;
      case SetupBack:
        drawScreen.backwardSetup();
        break;
      case SetupLeft:
        drawScreen.leftSetup();
        break;
      case SetupRight:
        drawScreen.rightSetup();
        break;
      case ShowDistance:
        drawScreen.showDistance();
        break;
      default:
        Brain.Screen.print("Error: No Screen Selected In 'void autonSetup()'");
        startLoop();
    }
    currentSelection = NA;
    waitForRelease();
    bool loopIsDone = false;
    while (!loopIsDone){
      if(Brain.Screen.pressing()) {
        // Special handling for ShowDistance screen (has floating export button)
        if(currentScreen == ShowDistance){
          int x = Brain.Screen.xPosition();
          int y = Brain.Screen.yPosition();

          // Check if Export button was pressed
          if(x >= 252 && x <= 466 && y >= 130 && y <= 226){
            exportAutonToSD();
            waitForRelease();
            // Redraw the screen to show success message
            drawScreen.showDistance();
            loopIsDone = false;  // Stay in the loop
            continue;
          }
          // Check if Return Home button was pressed
          else if(x >= 240){
            exportSuccessShown = false;  // Reset flag when leaving screen
            currentSelection = MM;  // This will trigger the screen change
            loopIsDone = true;
            continue;
          }
        }
        
        if(isMiddleButton && currentSelection == NA){
          if((Brain.Screen.xPosition()>160 && Brain.Screen.xPosition()< 320)&&(Brain.Screen.yPosition()>100 && Brain.Screen.yPosition()< 160)){
            currentSelection = MM;
          }
        }
        if(topButtonNumber == 2 && currentSelection == NA){
          if(Brain.Screen.yPosition()<125){
            if(Brain.Screen.xPosition()<240){         //Left
              currentSelection = TL;
            } else {                                  //Right
              currentSelection = TR;
            }
          }
        }
        if(topButtonNumber == 3 && currentSelection == NA){
          if(Brain.Screen.yPosition()<125){
            if(Brain.Screen.xPosition()<167){              //Left: 0-167 (10-157 button)
              currentSelection = TL;
            } else if(Brain.Screen.xPosition()<324) {      //Middle: 167-324 (167-314 button)
              currentSelection = TM;
            }else{                                         //Right: 324-480 (324-471 button)
              currentSelection = TR;
            }
          }
        }
        if(bottomButtonNumber == 2 && currentSelection == NA){
          if(Brain.Screen.yPosition()>=125){
            if(Brain.Screen.xPosition()<240){         //Left
              currentSelection = BL;
            } else {                                  //Right
              currentSelection = BR;
            }
          }
        }
        if(bottomButtonNumber == 3 && currentSelection == NA){ 
          if(Brain.Screen.yPosition()>=125){
            if(Brain.Screen.xPosition()<167){              //Left: 0-167 (10-157 button)
              currentSelection = BL;
            }else if(Brain.Screen.xPosition()<324) {       //Middle: 167-324 (167-314 button)
              currentSelection = BM;
            }else{                                         //Right: 324-480 (324-471 button)
              currentSelection = BR;
            }
          }
        }
        waitForRelease();
        loopIsDone = true;
      }
    }
    switch (currentSelection){
     case TL:
      switch (currentScreen) {
       case BaseHome:
        drawScreen.leftAutonomous();
        break;

       case SetupHome:
        currentScreen = SetupFront;
        break;

       case SetupFront:
        if(robotConnected){
          Drivetrain.driveFor(forward, 10, mm);
        }
        record(10, Forward);
        currentScreen = SetupHome;
        break;

       case SetupBack:
        if(robotConnected){
          Drivetrain.driveFor(reverse, 10, mm);
        }
        record(-10, Forward);
        currentScreen = SetupHome;
        break;

       case SetupLeft:
        if(robotConnected){
          Drivetrain.turnFor(left, 5, degrees);
        }
        record(5, Left);
        currentScreen = SetupHome;
        break;
       case SetupRight:
        if(robotConnected){
          Drivetrain.turnFor(right, 5, degrees);
        }
        record(-5, Left);
        currentScreen = SetupHome;
        break;

       default:
        Brain.Screen.print("Error: check position 1");
        startLoop();
        break;
      }
      break;

     case TM:
      switch (currentScreen) {
       case SetupFront:
        if(robotConnected){
          Drivetrain.driveFor(forward, 50, mm);
        }
        record(50, Forward);
        currentScreen = SetupHome;
        break;

       case SetupBack:
        if(robotConnected){
          Drivetrain.driveFor(reverse, 50, mm);
        }
        record(-50, Forward);
        currentScreen = SetupHome;
        break;

       case SetupLeft:
        if(robotConnected){
          Drivetrain.turnFor(left, 10, degrees);
        }
        record(10, Left);
        currentScreen = SetupHome;
        break;

       case SetupRight:
        if(robotConnected){
          Drivetrain.turnFor(right, 10, degrees);
        }
        record(-10, Left);
        currentScreen = SetupHome;
        break;

       default:
        Brain.Screen.print("Error: check position 2");
        startLoop();
        break;
      }
      break;
     case TR:
      switch (currentScreen) {
       case BaseHome:
        drawScreen.rightAutonomous();
        break;

       case SetupHome:
        currentScreen = SetupBack;
        break;

       case SetupFront:
        if(robotConnected){
          Drivetrain.driveFor(forward, 100, mm);
        }
        record(100, Forward);
        currentScreen = SetupHome;
        break;

       case SetupBack:
        if(robotConnected){
          Drivetrain.driveFor(reverse, 100, mm);
        }
        record(-100, Forward);
        currentScreen = SetupHome;
        break;

       case SetupLeft:
        if(robotConnected){
          Drivetrain.turnFor(left, 45, degrees);
        }
        record(45, Left);
        currentScreen = SetupHome;
        break;

       case SetupRight:
        if(robotConnected){
          Drivetrain.turnFor(right, 45, degrees);
        }
        record(-45, Left);
        currentScreen = SetupHome;
        break;

       default:
        Brain.Screen.print("Error: check position 3");
        startLoop();
        break;
      }
      break;
     case MM:
      switch (currentScreen) {
       case SetupHome:
        exportSuccessShown = false;  // Reset flag when entering log screen
        currentScreen = ShowDistance;
        break;
       case ShowDistance:
        exportSuccessShown = false;  // Reset flag when leaving log screen
        currentScreen = SetupHome;
        break;

       default:
        Brain.Screen.print("Error: check position 4");
        startLoop();
        break;
      }

      break;
     case BL:
      switch (currentScreen) {
       case BaseHome:
        drawScreen.noAutonomous();
        break;

       case SetupHome:
        currentScreen = SetupLeft;
        break;

       case SetupFront:
        if(robotConnected){
          Drivetrain.driveFor(forward, tile/2, mm);
        }
        record(tile/2, Forward);
        currentScreen = SetupHome;
        break;

       case SetupBack:
        if(robotConnected){
          Drivetrain.driveFor(reverse, tile/2, mm);
        }
        record(-1*(tile/2), Forward);
        currentScreen = SetupHome;
        break;

       case SetupLeft:
        if(robotConnected){
          Drivetrain.turnFor(left, 90, degrees);
        }
        record(90, Left);
        currentScreen = SetupHome;
        break;

       case SetupRight:
        if(robotConnected){
          Drivetrain.turnFor(right, 90, degrees);
        }
        record(-90, Left);
        currentScreen = SetupHome;
        break;

       default:
        Brain.Screen.print("Error: check position 5");
        startLoop();
        break;
      }
      break;
     case BM:
      switch (currentScreen) {
      case SetupFront:
       if(robotConnected){
         Drivetrain.driveFor(forward, tile, mm);
       }
       record(tile, Forward);
       currentScreen = SetupHome;
       break;
    
      case SetupBack:
       if(robotConnected){
         Drivetrain.driveFor(reverse, tile, mm);
       }
       record(-1*tile, Forward);
       currentScreen = SetupHome;
       break;
    
      default:
       Brain.Screen.print("Error: check position 6");
       startLoop();
       break;
      }
      break;
     case BR:
      switch (currentScreen) {
       case BaseHome:
        currentScreen = SetupHome;
        break;

       case SetupHome:
        currentScreen = SetupRight;
        break;

       case SetupFront:
        if(robotConnected){
          Drivetrain.driveFor(forward, 2*tile, mm);
        }
        record(2*tile, Forward);
        currentScreen = SetupHome;
        break;

       case SetupBack:
        if(robotConnected){
          Drivetrain.driveFor(reverse, 2*tile, mm);
        }
        record(-1*(2*tile), Forward);
        currentScreen = SetupHome;
        break;

       case SetupLeft:
        if(robotConnected){
          Drivetrain.turnFor(left, 180, degrees);
        }
        record(180, Left);
        currentScreen = SetupHome;
        break;

       case SetupRight:
        if(robotConnected){
          Drivetrain.turnFor(right, 180, degrees);
        }
        record(-180, Left);
        currentScreen = SetupHome;
        break;

       default:
        Brain.Screen.print("Error: check position 7");
        startLoop();
        break;
      }
      break;
     default:
      Brain.Screen.print("Error no selection recorded/made in 'void autonSetup()'");
      break;
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

    LeftDriveSmart.setVelocity(900, rpm);
    RightDriveSmart.setVelocity(900, rpm);
    Drivetrain.setStopping(brake);
    Drivetrain.setTurnVelocity(900, rpm);
    Drivetrain.setDriveVelocity(900, rpm);

    TheMalfunctioner.calibrate();
    
    while(TheMalfunctioner.isCalibrating()) {
        wait(20, msec);
    }

    autonSetup();  // Run the setup interface
    while(true) {
        wait(100, msec);
    }
}
#pragma endregion Main