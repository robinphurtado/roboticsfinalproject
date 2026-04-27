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

#define WALL_FOLLOWING  0
#define RETURN_TO_DOCK 2

//maze navigation 
#define CELL_SIZE 20
#define ROWS 4
#define COLS 9
#define MAXMOVES 200

Motors motors;
Servo servo;
Encoders encoders;
Sonar sonar(4);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
//float x, y, theta;  do I need to set initial x & y to 10?  
float x = -10.0;
float y = 10.0;
float theta = 0.0;

// wall following variables
const double goalWallDist=14.0; // Goal distance from wall (cm), greater than 10 since using 135degree angle
double actualWallDist;
int state = WALL_FOLLOWING;

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
  initializeArray();
  startTime = millis();
  //calibrate
  //Move Sonar to desired direction using Servo
  servo.write(135);
  delay(2000);
  
  // mark starting cell as visited
  visitedCells[0][0] = 'V';
}

void loop() {
  //for troubleshooting. if values are duplicate, remove this one 4/27
  Serial.print("At top of loop: ");
  odometry.printSerial(x, y, theta);  //prints odometry, theta in degrees

  // WALL FOLLOW
  if (state == WALL_FOLLOWING) {
    
    wallFollowing();

    // ODOMETRY
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    // Increment total encoder count
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    // update x,y, and theta 
    odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);  //prints odom to OLED, theta in rad, calls printserial(), theta in degrees 

    //CELL TRACKING
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
  }
  



  
}

//HELPER FUNCTIONS

/*initializeArray initializes all cells the visitedCells Array with 'N' for 
not visited, then marks the unreachable cells occupied by the pallet as 'V'
for visited so that the robot does not require those cells to be visited before
moving to RETURN_TO_DOCK. does not return a value */

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

/*wallFollowing function pings sonar, calculates distance from nearest obstacle,
then adjusts the wheel speeds to turn to avoid obstacle, constraining min speed
and max speed at -400 and 400 respectively to not damage motor. does not return a value */
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
  Serial.print(" PDout: ");
  Serial.println(PDout);
}
