#include <Pololu3piPlus32U4.h>
#include "printOLED.h"
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"
#include "odometry.h"


using namespace Pololu3piPlus32U4;

//Calibration
int calibrationSpeed = 50;

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

//phase 3 constants

#define BLACK_THRESHOLD 900
#define NUM_BINS 3

#define WALL_FOLLOWING  0
#define RETURN_TO_DOCK  2
#define PICK_SERVICE 4

//maze navigation 
#define CELL_SIZE 20
#define ROWS 4
#define COLS 9
#define MAXMOVES 200

LineSensors lineSensors;
Motors motors;
Servo servo;
Encoders encoders;
Sonar sonar(4);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
//OLED display;

//phase 3 variables
int binCount = 0;
//display.clear();



//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10;
float y = 10; 
float theta;  //to I need to set initial x & y to 10?  ****

// array to hold the 5 line sensor values
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];  //what is this? 

//line Following
int lineCenter = 2000;
int16_t robotPosition;
bool justTurned = false;


bool isOnBlack; //where to use this?



int state = WALL_FOLLOWING;


PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);

const double goalWallDist=10.0; // Goal distance from wall (cm)

double actualWallDist;

// cell and movement tracking
int currentRow = 0;  //should these be local? 
int currentCol = 0;
int prevRow = 0;
int prevCol = 0;
int currentMove = 0;
char visitedCells[ROWS][COLS];
char movementLog[MAXMOVES];
int returnIndex = -1;  //initialize with -1 so we know return to dock hasnt started yet

// total timekeeping 
unsigned long startTime, endTime;

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  delay(40);
  startTime = millis();
  calibrateSensors();
  //Move Sonar to desired direction using Servo
  servo.write(135);
  delay(200);
  motors.setSpeeds(base_speed, base_speed);
  delay(1000);

  initializeArray();
  // mark starting cell as visited
  visitedCells[0][0] = 'V';
}

void loop() {

  odometry.printSerial();
  //DO NOTE DELETE CODE AFTER EACH TASK, COMMENT OUT INSTEAD

  // WALL FOLLOW STATE
  if (state == WALL_FOLLOWING) {
    Serial.print("State: Wall Following");
    
    wallFollowing();

    //read for line
    lineSensors.read(lineSensorValues);
    /*  changing so isOnBlack is only true when black is detected
    if (lineSensorValues[2] > BLACK_THRESHOLD){
      isOnBlack = true;
    }
      */
    isOnBlack = lineSensorValues[2] > BLACK_THRESHOLD;

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
   

    // ODOMETRY
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    // Increment total encoder count
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    // update x,y, and theta 
    odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);

    //CELL TRACKING
    currentCol = (x-10)/CELL_SIZE; 
    currentRow = (y-10)/CELL_SIZE;

    // break this out to a function ?
    if(currentRow != prevRow || currentCol != prevCol) {
    
      //maybe add bounds checking
      if (visitedCells[currentRow][currentCol] != 'V') { 
        visitedCells[currentRow][currentCol] = 'V';
        Serial.print("cell [");
        Serial.print(currentRow);
        Serial.print("][");
        Serial.print(currentCol);
        Serial.println("] visited");
      }
      if (currentCol > prevCol) {
        movementLog[currentMove] = 'R';
        Serial.print(currentMove);
        Serial.println(" R");
        currentMove++;
      } else if (currentCol < prevCol) {
        movementLog[currentMove] = 'L';
        Serial.print(currentMove);
        Serial.println(" L");
        currentMove++;
      } else if (currentRow > prevRow) {
        movementLog[currentMove] = 'U';
        Serial.print(currentMove);
        Serial.println(" U");
        currentMove++;
      } else if (currentRow < prevRow) {
        movementLog[currentMove] = 'D';
        Serial.print(currentMove);
        Serial.println(" D");
        currentMove++;
      }
    }
    
    //set current row and column to previous for next loop
    prevRow = currentRow;
    prevCol = currentCol;

  } else if (state == PICK_SERVICE) {
     serviceBin();
     //should this be somewhere else too? 
     if (binCount >= NUM_BINS) {
      state = RETURN_TO_DOCK;
     }
     else {
      //removed this because If we come out of spin crooked, it will crash into wall. going with boolean instead
      /*
      motors.setSpeeds(base_speed, base_speed);
      //put in a delay in millis
      delay(1000);
      */
      justTurned = true;
      state = WALL_FOLLOWING;
     }

  } else if (state == RETURN_TO_DOCK) {
    motors.setSpeeds(0, 0);

  }
  
}

//initialize the array with 'N'
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
//wall Following
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
  Serial.println(" PDout: ");
  Serial.print(PDout);
}


void serviceBin() {
  motors.setSpeeds(0, 0);
  delay(200);
  binCount++;
  Serial.print("bin: ");
  Serial.print(binCount);
  Serial.print(" pick-confirmed at t=");
  Serial.println(millis());
  //display.clear();
  //display.print(F("Bins: "));
  //display.print(binCount);
  unsigned long spinStart = millis();
  // refine or try goToAngle ? 
  while (millis() - spinStart < 3600) {
    motors.setSpeeds(-80, 80);
}
 motors.setSpeeds(0, 0);
 delay(300);
}

void calibrateSensors()
{
  //Copy your calibrateSensors() function from lab 8
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