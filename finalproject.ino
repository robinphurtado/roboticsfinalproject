#include <Pololu3piPlus32U4.h>
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
#define kp 20 // starting with 20 which was decent for P controller
#define kd 0 // starting with 0 as a base state
#define base_speed 100

//phase 3 constants
#define PICK_SERVICE 4
#define BLACK_THRESHOLD 900
#define NUM_BINS 3

#define WALL_FOLLOWING  0

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

//phase 3 variables
int binCount = 0;
// display.clear(); is this oled?

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;  //to I need to set initial x & y to 10?  ****

//calibration
int calibrationSpeed = 50;
// array to hold the 5 line sensor values
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];  //what is this? 

//line Following
int lineCenter = 2000;
int16_t robotPosition;

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
  //calibrate
  //Move Sonar to desired direction using Servo
  servo.write(180);
  delay(2000);

  initializeArray();
  // mark starting cell as visited
  visitedCells[0][0] = 'V';
}

void loop() {

  odometry.printSerial();
  //DO NOTE DELETE CODE AFTER EACH TASK, COMMENT OUT INSTEAD

  // WALL FOLLOW
  if (state == WALL_FOLLOWING) {
    
    wallFollowing();

    //read for line
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[2] > BLACK_THRESHOLD) {
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
    currentCol = x/CELL_SIZE; 
    currentRow = y/CELL_SIZE;

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
     if (binCount >= NUM_BINS) {
    //  state = RETURN_TO_DOCK;
     }
     else {
      state = WALL_FOLLOWING;
     }
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
  int16_t leftSpeed = constrain(base_speed + PDout, -400, 400);
  int16_t rightSpeed = constrain(base_speed - PDout, -400, 400);
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
  Serial.print("bin");
  Serial.print(binCount);
  Serial.print(" pick-confirmed at t=");
  Serial.println(millis());
  //display.clear();
  //display.print(F("Bins: "));
  //display.print(binCount);
  unsigned long spinStart = millis();
  while (millis() - spinStart < 1800) {
    motors.setSpeeds(-80, 80);
}
 motors.setSpeeds(0, 0);
 delay(300);
}
