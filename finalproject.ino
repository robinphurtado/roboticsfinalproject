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
#define DONE 5


//maze navigation 
#define CELL_SIZE 20
#define ROWS 4
#define COLS 9
#define MAXMOVES 200

Motors motors;
Servo servo;
Encoders encoders;
Sonar sonar(4);
// Odometry instance
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
// PDController for obstacle avoidance
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);
// PoluluBuzzer for beep when done
PololuBuzzer buzzer;

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;  //to I need to set initial x & y to 10?  ****

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
int returnIndex = 0;  
bool returnStarted = false; 

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

  odometry.printSerial();

  // WALL FOLLOW
  if (state == WALL_FOLLOWING) {
    
    wallFollowing();
    Serial.print("State: Wall Following");

    // odometry
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    // Increment total encoder count
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    // update x,y, and theta 
    odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);

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

        //check if all are visited and advance
    bool allVisited = true;
    for (int r = 0; r < ROWS; r++) {
      for (int c = 0; c < COLS; c++) {
        // if current cell is not visited
        if (visitedCells[r][c] == 'N') {
          // set all visited to false, break out and continue
          allVisited = false;
          break;
        }
      }
      // if allVisited is false, break out and continue
      if (!allVisited) {
        break;
      }
    }
    // if allVisited is true, advance to return to dock
    if (allVisited) {
      state = RETURN_TO_DOCK;
    }
  } 

///////////////////////////////THIS IS SUPER BUGGY!////////////////////////////////////////////
  
  else if (state == RETURN_TO_DOCK) {
    Serial.println("State: Return to Dock");

    // check for light brown dock
    // if dock found, stop, beep, advance to done

    // does return to dock need a delay coming out of the turn?
    // no since it isnt using wall following? 

    // check if return to dock has started
    if (!returnStarted) {
      // stop robot
      motors.setSpeeds(0, 0);
      // slight delay to settle
      delay(200);
      // set returnIndex to the last stored move
      returnIndex = currentMove - 1;
      // set returnStarted to true since we are starting the state
      returnStarted = true;      
      return;

    // finished stepping backwards through all moves
    } else if (returnIndex < 0) {
      motors.setSpeeds(0, 0);
      //#TODO turn this into a function //
      buzzer.play("!L16 V8 fgab");
      endTime = millis();
      servo.write(90);
      // need to make sure his position matches original
      //does that mean turn around?
      Serial.println("Docked");
      Serial.print("Start Time: " );
      Serial.println(startTime);
      Serial.print("End Time: ");
      Serial.println(endTime);

      state = DONE;
      return;
    }

    char move = reverseMove(movementLog[returnIndex]);

    executeMove(move);

    returnIndex--;
  } 
///////////////////////////////////////////////////////////////////////////////////////////////
  else if (state == DONE) {
    Serial.print("State: DONE");
    halt();
    // nothing happens here, do not pass go, do not advance to any more states
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
  Serial.println(" PDout: ");
  Serial.print(PDout);
}

/////////////////////NOT SURE ABOU THESE EITHER////////////////////////////////////////////////////

/* write description for reversemove*/
char reverseMove(char move) {
  if (move == 'R') return 'L';
  if (move == 'L') return 'R';
  if (move == 'U') return 'D';
  if (move == 'D') return 'U';
  return 'X';
}

//helper normalizeAngle to help avoid wrap around of theta
// write more 
float normalizeAngle(float angle) {
  while (angle > PI)  angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

/* write description for execute move*/
void executeMove(char move) {

  if (move == 'R') {
    turnToAngle(0);      // east
  } else if (move == 'L') {
    turnToAngle(PI);     // west
  } else if (move == 'U') {
    turnToAngle(PI/2);   // north
  } else if (move == 'D') {
    turnToAngle(-PI/2);  // south
  }

  driveForwardOneCell();
}

/* write description for driveForwardOneCell */
void driveForwardOneCell() {

  float startX = x;
  float startY = y;

  while (true) {

    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

    // calcluate distance 
    float dist = sqrt(pow(x - startX, 2) + pow(y - startY, 2));

    if (dist >= CELL_SIZE) {
      break;
    }

    motors.setSpeeds(base_speed, base_speed);
  }

  motors.setSpeeds(0, 0);
}

void turnToAngle(float targetTheta) {
  while (fabs(normalizeAngle(theta - targetTheta)) > 0.1) {

    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

    motors.setSpeeds(80, -80);
  }

  motors.setSpeeds(0, 0);
}

//////////////////////////////////////////////////////////////////////////////////////

void halt() {
  motors.setSpeeds(0,0);
}