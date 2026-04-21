#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h" //Uncomment after importing your PDcontroller files
#include "odometry.h"     //needed for controlled turn for wall following

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;
Encoders encoders;

Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 10
#define kd_line 6
#define kp_obs 8  //not sure what will be best starting 5
#define kd_obs 0  //not sure what will be best starting 2

//Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

//constants for identifying states
#define WALL_FOLLOWING    0
#define STOP_AND_PREP     1
#define TURNING   2
//#define TURN_AROUND    3
#define RETURN_TO_DOCK   3


//maze navigation 
#define CELL_SIZE 20
#define ROWS 4
#define COLS 9
#define MAXMOVES 200

// Odometry instance 
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
// PdController for line following
PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
// PdController for obstacle avoidance
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

//Recommended Variables

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;  //to I need to set initial x & y to 10?  ****
 

//Calibration
int calibrationSpeed = 50;
// array to hold the 5 line sensor values
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];  //what is this? 

//Line Following
int lineCenter = 2000;
int16_t robotPosition;

bool isOnBlack; //where to use this?

//Wall Following
int PDout;
float wallDist;
int distFromWall = 7;
int detectionDist = 10;  

int task = WALL_FOLLOWING;  //initial state is WALL FOLLOWING
float theta_start = 0;  //do I need this? 
float goal_theta;  //180 degrees 
//started with Pi/2, was not good, robot turned around and found line going the opposite direction
//float goal_theta = PI/4.0;  //second try 45 degrees

// cell and movement tracking
int currentRow = 0;  //should these be local? 
int currentCol = 0;
int prevRow = 0;
int prevCol = 0;
int currentMove = 0;
char visitedCells[ROWS][COLS];
char movementLog[MAXMOVES];
int returnIndex;

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

//NEED TO ADD THE BROWN LINE CALIBRATION WHEN ON STEP 2
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

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(180); // turn servo left for wall following
  delay(2000);

  initializeArray();
  // mark starting cell as visited
  visitedCells[0][0] = 'V';

//  calibrateSensors();  //uncomment when we move on to include line follow  
}

void loop(){
    // pint odom values for debugging
  odometry.printSerial();

// navigation
  if (task == WALL_FOLLOWING) {
    //print state for debugging
    Serial.println("State: Wall Following");
    wallFollowing();
    
    // ODOMETRY
    // get counts from encoders for this loop
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    // Increment total encoder count
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    // update x,y, and theta 
    odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
    // update cells visited
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
        Serial.print("] visited");
      }
      if (currentCol > prevCol) {
        movementLog[currentMove] = 'R';
        Serial.print(currentMove);
        Serial.print("R");
        currentMove++;
      } else if (currentCol < prevCol) {
        movementLog[currentMove] = 'L';
        Serial.print(currentMove);
        Serial.print("L");
        currentMove++;
      } else if (currentRow > prevRow) {
        movementLog[currentMove] = 'U';
        Serial.print(currentMove);
        Serial.print("U");
        currentMove++;
      } else if (currentRow < prevRow) {
        movementLog[currentMove] = 'D';
        Serial.print(currentMove);
        Serial.print("D");
        currentMove++;
      }
    }
    
    //set current row and column to previous for next loop
    prevRow = currentRow;
    prevCol = currentCol;

    // check if the cell matches a dead end cell
    if (isDeadEndCell(currentRow, currentCol)) {
      //stop and prep if this is a known dead end
      goal_theta = PI/2;
      task = STOP_AND_PREP;
      return;   
    }
    
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
      // if allVisisted is false, break out and continue
      if (!allVisited) {
        break;
      }
    }
    // if allvistied is true, advance to return to dock
    if (allVisited) {
      task = RETURN_TO_DOCK;
    } 

  /* //THIS WILL COME IN LATER 
  // start line following
  if (task == LINE_FOLLOWING) {
    // print state
    //Serial.println("State: Line Following");
    float dist = sonar.readDist();
    // if object detected, switch states
    if (dist > 0 && dist <= detectionDist) {
        //stop and move to next state  
        motors.setSpeeds(0, 0);
        Serial.println("State: Stop & Prep");
        task = STOP_AND_PREP;
    } else {
        lineFollowing();
    }  */


///*
  //} else if (task == STOP_AND_PREP) {
  else if (task == STOP_AND_PREP) {
    Serial.println("State: Stop and Prep");
    // stop robot
    motors.setSpeeds(0, 0);
    // record starting theta to turn from 
    theta_start = theta;  
    task = TURNING;       // TURN goal_theta degrees
//*/
///*

  } else if (task == TURNING) {
    Serial.println("State: Turning");

    // update encoder counts and odometry
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

    // calculate angle turned calculating normalized angle 
    float angleTurned = fabs(normalizeAngle(theta - theta_start));

    Serial.print("Angle turned: ");
    Serial.println(angleTurned);

    if (angleTurned < goal_theta) {
      motors.setSpeeds(-baseSpeed, baseSpeed);  //**** CHANGE THIS IS ROBOT IS TURNING 'LEFT' THE WRONG WAY
    } else {
      motors.setSpeeds(0, 0);
      // slight delay to settle if needed
      //delay(50);
      task = WALL_FOLLOWING;
    }
  } else if (task == RETURN_TO_DOCK) {

    Serial.println("State: Return to Dock");

    if (returnIndex < 0) {
      returnIndex = currentMove - 1;
      motors.setSpeeds(0, 0);
      // beep here later
      return;
    }

    char move = reverseMove(movementLog[returnIndex]);

    executeMove(move);

    returnIndex--;
  } 
}
    /*
    // for using detectBlackLine() instead of robotPosition to avoid gray tile false positives
    if (detectBlackLine()) {
        motors.setSpeeds(0, 0);
        task = RESET_PREP;
    }
    */

     /* uncomment this if using the detectBlackLine()
    // when the line is found again, but before it hits center
    if (robotPosition > 500 && robotPosition < 2000) {  
      //stop
      motors.setSpeeds(0, 0);
      //move to next state
      //Serial.println("State: Reset Prep");
      task = RESET_PREP;
    }
     */
/*
  // reset prep state stops and turns servo and robot back forward for line followng
  } else if (task == RESET_PREP) {
    // prep to turn left back to forward
    //Serial.println("State: Reset Prep");
    motors.setSpeeds(0, 0);
    //turn servo forward again
    servo.write(90); 
    theta_start = theta;
    delay(100);
    // return back to line following
    task = LINE_FOLLOWING;
  } 

  */
    
}

// helper functions

///*
void lineFollowing()
{
  //From lab 8
  //read in robots current position over line, store the returned value in robotPosition
  robotPosition = lineSensors.readLineBlack(lineSensorValues);

  // pass values to PD controller to calculate pd_lineOut signal
  float pd_lineOut = pd_line.update(robotPosition, lineCenter);

  // pass pd_lineOut signal to motors to adjust location
  int16_t leftSpeed = constrain(baseSpeed - pd_lineOut, -400, 400);
  int16_t rightSpeed = constrain(baseSpeed + pd_lineOut, -400, 400);
  motors.setSpeeds(leftSpeed, rightSpeed);  
}

//*/
///*
void wallFollowing()
{
  // using sonar to calculate distance
  wallDist = sonar.readDist();

    // Check distance for greater than wall follow distance (fine tune) 
    if (wallDist > 20) {  // once this is turned, set variable
      goal_theta = PI/2; 
      task = STOP_AND_PREP; 
      return; 
    }

  // call PD Controller to calculate control signal
  PDout = pd_obs.update(wallDist, distFromWall); 

  int leftSpeed  = constrain(baseSpeed - PDout, -400, 400);
  int rightSpeed = constrain(baseSpeed + PDout, -400, 400);

  // adjusting to maintain distance
  motors.setSpeeds(leftSpeed, rightSpeed);

  //Also print outputs to serial monitor for testing purposes
  Serial.print("Wall Dist: "); Serial.print(wallDist);
  Serial.print(" PD Out: "); Serial.println(PDout);

  // sensing for line  ADD FOR BROWN LINE LATER PHASE
  //read in robots current position over line, store the returned value in robotPosition

  // robotPosition = lineSensors.readLineBlack(lineSensorValues);
}
//*/

//helper normalizeAngle to help avoid wrap around of theta
float normalizeAngle(float angle) {
  while (angle > PI)  angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

// bool isDeadEndCell from known map to get out of dead end
bool isDeadEndCell(int row, int col) {
  return (row == 0 && col == 1) ||
         (row == 2 && col == 7);
}

char reverseMove(char move) {
  if (move == 'R') return 'L';
  if (move == 'L') return 'R';
  if (move == 'U') return 'D';
  if (move == 'D') return 'U';
  return 'X';
}

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

void driveForwardOneCell() {

  float startX = x;
  float startY = y;

  while (true) {

    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

    float dist = sqrt(pow(x - startX, 2) + pow(y - startY, 2));

    if (dist >= CELL_SIZE) {
      break;
    }

    motors.setSpeeds(100, 100);
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
/*
void detectBlackLine()
{
  lineSensors.read(lineDetectionValues);

    // Threshold value to detect black (adjust based on calibration)
    const int blackThreshold = 1500; 

    // Check if the robot is on a black square
    for (int i = 0; i < 5; i++) {
        Serial.println(lineDetectionValues[i]);
        if (lineDetectionValues[i] > blackThreshold) {
          return true;  // at least one sensor sees black
        }
    }

    return false;  // sensors are seeing colors other than blank
}
*/
