#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"
#include "printOLED.h"
#include "odometry.h"
#include "BrownLineFollower.h"
 
using namespace Pololu3piPlus32U4;
 
// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
LineSensors lineSensors;
Motors motors;
Encoders encoders;
Servo servo;
Sonar sonar(4);
PrintOLED oled;
 
// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
 
// Calibration
#define CALIBRATION_SPEED 50

//Odometry
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

//maze navigation 
#define CELL_SIZE 20
#define ROWS 4
#define COLS 9
#define MAXMOVES 200
 
// Wall following PD
#define BASE_SPEED     120
#define KP             12
#define KD             2
#define KP_LINE        10 //best from lab 9
#define KD_LINE        6  //best from lab 9
#define MIN_OUTPUT    -100
#define MAX_OUTPUT     100
#define GOAL_WALL_DIST 20.0   // cm
 
// Phase 3: Bin detection
#define BLACK_THRESHOLD 900   // calibrated IR value for black square (0-1000)
#define NUM_BINS        3     // total bins to collect
#define SPIN_TIME       4200  // ms for full 360 degree spin
#define FORWARD_TIME    600   // ms to drive forward off bin square
 
// Phase 2: Return to dock
#define LIGHT_BROWN_MIN 180   // lower bound for light brown dock square
#define LIGHT_BROWN_MAX 300   // upper bound for light brown dock square

// Phase #: Fast Track
#define DARK_BROWN_MIN 320   // lower bound for dark brown dock square
#define DARK_BROWN_MAX 650   // upper bound for dark brown dock square
 
// ---------------------------------------------------------------------------
// Robot States
// ---------------------------------------------------------------------------
#define WALL_FOLLOWING  0
#define PICK_SERVICE    1
#define RETURN_TO_DOCK  2
#define DOCKED          3
#define FAST_TRACK      4
 
int state = WALL_FOLLOWING;
 
// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------
unsigned int lineSensorValues[5];
double actualWallDist;
int binCount = 0;
bool leftStartZone = false;
PololuBuzzer buzzer;
bool returning = false; //**

//odometry **
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0; 
float x = -10.0;
float y = 10.0;
float theta = 0.0;

// movement tracking **
char visitedCells[ROWS][COLS];
char movementLog[MAXMOVES];

int currentRow = 0;  //should these be local? 
int currentCol = 0;
int prevRow = 0;
int prevCol = 0;
int currentMove = 0;
int returnIndex = 0;  //pretty sure change this to a bool and it should start at 0

//line following
unsigned int lineDetectionValues[5];
int16_t lineCenter = 2000;  //do a define instead?
int16_t robotPosition;  //maybe dont need this
uint16_t brownPosition;
bool brownDetected = false;
 
// timekeeping
unsigned long startTime, endTime;

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING); //** 
PDcontroller wallController(KP, KD, MIN_OUTPUT, MAX_OUTPUT);
PDcontroller pd_brown(KP_LINE, KD_LINE, MIN_OUTPUT, MAX_OUTPUT);
BrownLineFollower linefollower(lineSensors, DARK_BROWN_MIN, DARK_BROWN_MAX);
 
// Function Prototypes
void calibrateSensors();
void wallFollowing();
void serviceBin();
 
// setup()
void setup() {
  Serial.begin(9600);
  servo.attach(5);
  delay(40);

  Serial.println("Starting in 3 seconds...");
  delay(3000); 
  Serial.println("Calibrating...");  
  calibrateSensors();
  Serial.println("Calibration done.");
  servo.write(150);
  motors.setSpeeds(0, 0);
  delay(500);

  //record start time
  startTime = millis();

  
  //drive off calibration cell - split out to a 'clear cell' state? odom is not accruing here
  // or split out to the beginning of wall following
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(1000);  // drive off start square 

  // mark starting cell as visited***
  visitedCells[0][0] = 'V';
  
  Serial.println("Starting wall following.");
}
 
// loop()
void loop() {

    //update odometryxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  // ODOMETRY
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    // Increment total encoder count
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    // update x,y, and theta 
    //prints odom to OLED, theta in rad, calls printserial(), theta in degrees 
    odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);  

  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 
  if (state == WALL_FOLLOWING) {

        // read IR sensors & handle readoutxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    //**BROWN FOLLOWING uncomment this and comment below to switch to brownlinefollower instead of reading the lineSensors directly
    linefollower.readCalibrated(lineSensorValues);    
    brownDetected = linefollower.computeLinePosition(lineSensorValues, brownPosition);

    //**BROWN FOLLOWING comment out next line
    //lineSensors.readCalibrated(lineSensorValues);

    unsigned int centerVal = lineSensorValues[2];
 
    Serial.print("Center IR: ");
    Serial.println(centerVal);

    // dont detect bins until robot has seen white floor after start or last bin
    if (centerVal < BLACK_THRESHOLD - 200) {
      leftStartZone = true;
    }
 
    // detect black square bin — only if bins still needed
    if (leftStartZone && centerVal > BLACK_THRESHOLD && binCount < NUM_BINS) {
      state = PICK_SERVICE;
    }

    //**BROWN FOLLOWING uncomment this 
    if (brownDetected){
      state = FAST_TRACK;
    }
 
    wallFollowing();
 
    // // read IR sensors
    // lineSensors.readCalibrated(lineSensorValues);
    // unsigned int centerVal = lineSensorValues[2];
 
    // Serial.print("Center IR: ");
    // Serial.println(centerVal);
 
    // // dont detect bins until robot has seen white floor after start or last bin
    // if (centerVal < BLACK_THRESHOLD - 200) {
    //   leftStartZone = true;
    // }
 
    // // detect black square bin — only if bins still needed
    // if (leftStartZone && centerVal > BLACK_THRESHOLD && binCount < NUM_BINS) {
    //   state = PICK_SERVICE;
    // }
 
  } else if (state == PICK_SERVICE) {
 
    serviceBin();
 
    if (binCount >= NUM_BINS) {
      // all bins collected — switch to return to dock
      Serial.println("All bins collected. Returning to dock.");

      // robot ends after last bin pickup for navigated to last cell and in theory is facing forward
      // turn 180 degrees to face the other direction. SPIN_TIME is calc for 360, SPIN_TIME/2 should be ~180. wall following should straighten robot
      unsigned long spinStart = millis();
      while (millis() - spinStart < SPIN_TIME/2) {
        motors.setSpeeds(-80, 80);
      }
      motors.setSpeeds(0, 0); 
      // pause to let sonar stabilize after spin
      delay(800);

      //drive off last bin before going to return to doc so we dont sense for light brown early - split out to a 'clear cell' state?
      //but the fact that it is sensing the last black bin as light brown means light brown is not working properly
      motors.setSpeeds(BASE_SPEED, BASE_SPEED);
      delay(1000);  // drive off start square  
      state = RETURN_TO_DOCK;
    } else {
      state = WALL_FOLLOWING;
    }

    
 
  } else if (state == RETURN_TO_DOCK) {

    //***********************
    returning = true;

     //IF THIS WORKS, GOOD ENOUGH!! 
    // continue wall following back to dock and detect for light brown square
    wallFollowing();

    // read IR sensors to detect dock
    lineSensors.readCalibrated(lineSensorValues);
    unsigned int centerVal = lineSensorValues[2];
 
    Serial.print("Return IR: ");
    Serial.println(centerVal);

///*//******************************Added 5/2**************************************************
    // dont detect dock until robot has seen white floor after last bin
    if (centerVal < BLACK_THRESHOLD - 200) {
      leftStartZone = true;
    }
//*/*********************************************************************************************
 
    // detect light brown dock square
    if (centerVal >= LIGHT_BROWN_MIN && centerVal <= LIGHT_BROWN_MAX) {
      state = DOCKED;
    }
 
  } else if (state == DOCKED) {
 
    // stop and signal completion
    motors.setSpeeds(0, 0);
    endTime = millis();
    buzzer.playNote(NOTE_F(4), 2000, 10);
 
    Serial.println("Docked.");
    Serial.print("Start time: ");
    Serial.println(startTime);
    Serial.print("End time: ");
    Serial.println(endTime);
    Serial.print("Total time (ms): ");
    Serial.println(endTime - startTime);
 
    while (true) {} // halt
  }

   //CELL TRACKINGxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

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
    }
    
    //set current row and column to previous for next loop
    prevRow = currentRow;
    prevCol = currentCol;

  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
}
 
// calibrateSensors()
void calibrateSensors() {
  for (int i = 0; i < 80; i++) {
    if (i < 20 || i >= 60)
      motors.setSpeeds(-CALIBRATION_SPEED, CALIBRATION_SPEED);
    else
      motors.setSpeeds(CALIBRATION_SPEED, -CALIBRATION_SPEED);
    lineSensors.calibrate();
    delay(20);
  }
  motors.setSpeeds(0, 0);
}
 
// wallFollowing()
void wallFollowing() {
  actualWallDist = sonar.readDist();
 
  Serial.print("Wall dist: ");
  Serial.println(actualWallDist);
 
  // on bad read drive straight
  if (actualWallDist <= 0 || actualWallDist > 100) {
    motors.setSpeeds(BASE_SPEED, BASE_SPEED);
    return;
  }
 
  double PDout = wallController.update(actualWallDist, GOAL_WALL_DIST);
 
   int16_t leftSpeed  = constrain(BASE_SPEED + PDout, -400, 400);
   int16_t rightSpeed = constrain(BASE_SPEED - PDout, -400, 400);
//reversing to test 
  // int16_t leftSpeed  = constrain(BASE_SPEED - PDout, -400, 400);
  // int16_t rightSpeed = constrain(BASE_SPEED + PDout, -400, 400);

  motors.setSpeeds(leftSpeed, rightSpeed);
 
  Serial.print("PDout: ");
  Serial.println(PDout);
}
 
// serviceBin()
void serviceBin() {
  motors.setSpeeds(0, 0);
  delay(300);
 
  binCount++;
  Serial.print("bin: ");
  Serial.print(binCount);
  Serial.print(" pick-confirmed at t=");
  Serial.println(millis());
 
  // spin 360 degrees
  unsigned long spinStart = millis();
  while (millis() - spinStart < SPIN_TIME) {
    motors.setSpeeds(-80, 80);
  }
  motors.setSpeeds(0, 0);
 
  // pause to let sonar stabilize after spin
  delay(800);
 
  // drive forward off bin square to prevent re-detection
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(FORWARD_TIME);
  motors.setSpeeds(0, 0);
  delay(200);
 
  // reset so robot must see white floor before detecting next bin
  leftStartZone = false;
}
