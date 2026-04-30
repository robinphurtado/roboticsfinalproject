#include <Pololu3piPlus32U4.h>
#include "printOLED.h"
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"
#include "odometry.h"
#include "printOLED.h"


using namespace Pololu3piPlus32U4;

//Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

//Update kp and kd based on your testing
#define minOutput -100
#define maxOutput 100
#define kp 10 // starting with 20 which was decent for P controller
#define kd 2 // starting with 0 as a base state
#define base_speed 100
#define calibrationSpeed 50

//phase 3 constants
#define BLACK_THRESHOLD 900
#define NUM_BINS 3
#define CENTER_IR 2 //

// state constants
#define WALL_FOLLOWING  0
#define RETURN_TO_DOCK  2
#define PICK_SERVICE 4

//maze navigation 
#define CELL_SIZE 20
#define ROWS 4
#define COLS 9
#define MAXMOVES 200

// object instances
LineSensors lineSensors;
Motors motors;
Servo servo;
Encoders encoders;
Sonar sonar(4);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
PrintOLED oled;
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);
// ***********to test beep when done after 3 bins collected*************
// PoluluBuzzer for beep when done
//PololuBuzzer buzzer;

//phase 3 variables
int binCount = 0;
//display.clear();
bool isOnBlack;

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = -10.0;
float y = 10.0; 
float theta = 0.0;  //to I need to set initial x & y to 10?  ****

// array to hold the 5 line sensor values for location
unsigned int lineSensorValues[5];
// array to hold the 5 line sensor values for amount of reflection
unsigned int lineDetectionValues[5];  

//line Following
int lineCenter = 2000;
int16_t robotPosition;
bool justTurned = false;

// wall following variables
const double goalWallDist = 14.0; // Goal distance from wall (cm), greater than 10 since using 135degree angle
double actualWallDist;
int state = WALL_FOLLOWING;   // start in WALL_FOLLOWING state

// cell and movement tracking
int currentRow = 0;  
int currentCol = 0;
int prevRow = 0;
int prevCol = 0;
int currentMove = 0;  
char visitedCells[ROWS][COLS];
char movementLog[MAXMOVES];
int returnIndex = -1;  //CHANGED THIS IN RETURN BRANCH

// total timekeeping 
unsigned long startTime, endTime;

void setup() {
  Serial.begin(9600);
  servo.attach(5);   
  calibrateSensors();
  delay(40);
  startTime = millis(); //record start time after calibration  
  servo.write(150); //Move Sonar to 150 degress to detect forward and to the left
  delay(200);
  initializeArray(); 
  //motors.setSpeeds(base_speed, base_speed);
  // mark starting cell as visited
  visitedCells[0][0] = 'V';
  //delay(1000);  // drive off start square
}

void loop() {
  //for troubleshooting. if values are duplicate, remove this one 4/27
  Serial.print("At top of loop: ");
  
  //*************break this out to a method
  // odometry
  // store and then reset current encoder counts
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  // Increment total encoder count
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  // update x,y, and theta 
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);  
  //************************************

  // WALL FOLLOW
  if (state == WALL_FOLLOWING) {

    if(detectBlackBin()) {
      motors.setSpeeds(0,0);
      state = PICK_SERVICE;
      return;

    } else {  
      Serial.print("State: Wall Following");

      wallFollowing(); 

      //read for line
      //*****************************************************************************
      // lineSensors.read(lineSensorValues); <- this was incorrect, we were reading raw
      // values again. Instead this call should have been readCalibrated, which is now
      // embedded in detectBlackBin()
      /*  changing so isOnBlack is only true when black is detected
      if (lineSensorValues[2] > BLACK_THRESHOLD){
        isOnBlack = true;
      } 


      isOnBlack = lineSensorValues[2] > BLACK_THRESHOLD;

      */
      //********************************************************************

      // if just turned but still on the black square, continue wall following
      if (justTurned && isOnBlack){
        return;    
      }

      // if just turned and no longer on black square, reset justTurned 
      if (justTurned && !isOnBlack){
        justTurned = false;        
        return;    
      }

      // if not just turned and no longer on black square, 
      if (!justTurned && isOnBlack){
        state = PICK_SERVICE;        
        return;    
      }
    
//**********************break this out to a cell tracking method***********************
      //cell tracking
      // using int division to track current cell
      //starting of maze is lower right, x will be negative the whole time
      currentRow = (y-10)/CELL_SIZE;
      currentCol = (-x-10)/CELL_SIZE; //reversing sign for positive col#s recorded/ reversing pos direction of x axis
      

      // break this out to a function ?
      // if current row or column has changed
      if(currentRow != prevRow || currentCol != prevCol) {
      
        //maybe add bounds checking
        //if current cell has not been visited
        if (visitedCells[currentRow][currentCol] != 'V') { 
          // mark cell as visited
          visitedCells[currentRow][currentCol] = 'V';
          // print new cell row and col visited
          Serial.print("cell [");
          Serial.print(currentRow);
          Serial.print("][");
          Serial.print(currentCol);
          Serial.println("] visited");
        }
        // MOVE TRACKING
        // if current column# is greater than prev column#
        if (currentCol > prevCol) {
          // robot came from right (x axis is reversed)
          movementLog[currentMove] = 'R';
          Serial.print(currentMove);
          Serial.println(" R");
          currentMove++;
          // if current column# is less than prev column#
        } else if (currentCol < prevCol) {
          // robot came from left (x axis is reversed)
          movementLog[currentMove] = 'L';        
          Serial.print(currentMove);
          Serial.println(" L");
          currentMove++;
          // if current row is greater than previous row
        } else if (currentRow > prevRow) {
          // robot traveled up
          movementLog[currentMove] = 'U';
          Serial.print(currentMove);
          Serial.println(" U");
          currentMove++;
          // if current row is less than previous row
        } else if (currentRow < prevRow) {
          // robot traveled down 
          movementLog[currentMove] = 'D';
          Serial.print(currentMove);
          Serial.println(" D");
          currentMove++;
        }
        // for troubleshooting REMOVE LATER 
        Serial.print("Current Move: ");
        Serial.println(currentMove);
      }    
      //set current row and column to previous for next loop
      prevRow = currentRow;
      prevCol = currentCol;
//*********************************************************************************     
  }

  // STATE PICK SERVICE
  } else if (state == PICK_SERVICE) {
    Serial.print("State: Pick Service");

    serviceBin();
    //once all bins are collected 
    if (binCount >= NUM_BINS) {
      //return to dock
      state = RETURN_TO_DOCK;
    }
    else {
      //otherwise record just turned and return to WALL_FOLLOWING state
      justTurned = true;
      state = WALL_FOLLOWING;
    }

  // STATE RETURN TO DOCK
  } else if (state == RETURN_TO_DOCK) {
    Serial.print("State: Return to Dock");

    // for now just stop, later implement code for returning
    motors.setSpeeds(0, 0);
    //*******************************************************
    //buzzer.play("!L16 V8 fgab");

  }  
}
//HELPER FUNCTIONS

/*initializeArray function initializes all cells the visitedCells Array with 'N' for 
not visited, then marks the unreachable cells occupied by the pallet as 'V'
for visited so that the robot does not require those cells to be visited before
moving to RETURN_TO_DOCK. does not return a value */
void initializeArray()
{
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      visitedCells[r][c] = 'N';
    }
  }
  // block the location of the pallet
  visitedCells[0][2] = 'V';
  visitedCells[0][3] = 'V';
}

/*wallFollowing function pings sonar, calculates distance from nearest obstacle,
then adjusts the wheel speeds to turn to avoid obstacle, constraining min speed
and max speed at -400 and 400 respectively to not damage motor. does not return a value */
void wallFollowing () {

  actualWallDist = sonar.readDist();
  //calculate control signal
  double PDout = PDcontroller.update(actualWallDist, goalWallDist); //uncomment if using PDcontroller 

  // adjust speeds and set motors
  ///*
  int16_t leftSpeed = constrain(base_speed - PDout, -400, 400);
  int16_t rightSpeed = constrain(base_speed + PDout, -400, 400);
  //*/
  /* switch 
  int16_t leftSpeed = constrain(base_speed + PDout, -400, 400);
  int16_t rightSpeed = constrain(base_speed - PDout, -400, 400);
  */

  motors.setSpeeds(leftSpeed, rightSpeed);
  
  //Also print outputs to serial monitor for testing purposes
  Serial.print("Goal: ");
  Serial.print(goalWallDist);
  Serial.print(" Actual: ");
  Serial.print(actualWallDist);
  Serial.print(" PDout: ");
  Serial.println(PDout);
}

/*serviceBin function stops robot, increments bin count, and does a 360 degree
turn to signify collecting the bin ADD SOUND AND FIX DISPLAY. does not return a value */
void serviceBin() {

  motors.setSpeeds(0, 0);
  delay(200);
  binCount++;
  Serial.print("bin: ");
  Serial.print(binCount);
  Serial.print(" pick-confirmed at t=");
  Serial.println(millis());
  //display.clear();  //this is insdie the print_bins function now
  //display.print(F("Bins: "));
  //display.print(binCount);
  oled.print_bins(binCount);
  unsigned long spinStart = millis();
  // refine or try goToAngle ? 
  while (millis() - spinStart < 3600) {
    motors.setSpeeds(-80, 80);
}
 motors.setSpeeds(0, 0);
 delay(300);
}

/*calibrateSensors function calibrates the sensors for line detection.
turns on the IR sensors and rotates the robots left and right in place 
to calibrate the IR sensors when placed over a calibration square */
void calibrateSensors()
{
    // loop to control robot back and forth wiggle
  for (int i = 0; i < 80; i++){
    if(i < 20 || i >= 60)
      // rotate left over line for 20 interations
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
     else
      // rotate right over the line for 20 iterations
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    //calibrate with sensors on
    lineSensors.calibrate();
    delay(20);
  }
  // stop robot when calibration is complete
  motors.setSpeeds(0,0);
}

/*detectBlackBin function  */
bool detectBlackBin(){
  // read the calibrated values
  lineSensors.readCalibrated(lineSensorValues);
  // returns true if center IR detects value over black threshold
  return lineSensorValues[CENTER_IR] > BLACK_THRESHOLD; 

}