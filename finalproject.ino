#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"
#include "printOLED.h"
 
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
 
// Wall following PD
#define BASE_SPEED     120
#define KP             12
#define KD             2
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
 
// ---------------------------------------------------------------------------
// Robot States
// ---------------------------------------------------------------------------
#define WALL_FOLLOWING  0
#define PICK_SERVICE    1
#define RETURN_TO_DOCK  2
#define DOCKED          3
 
int state = WALL_FOLLOWING;
 
// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------
unsigned int lineSensorValues[5];
double actualWallDist;
int binCount = 0;
bool leftStartZone = false;
PololuBuzzer buzzer;
 
// timekeeping
unsigned long startTime, endTime;
 
PDcontroller PDcontroller(KP, KD, MIN_OUTPUT, MAX_OUTPUT);
 
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

  motors.setSpeeds(0, 0);
  delay(500);

  //record start time
  startTime = millis();

  servo.write(150);
  //drive off calibration cell - split out to a 'clear cell' state? odom is not accruing here
  // or split out to the beginning of wall following
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(1000);  // drive off start square 
  
  Serial.println("Starting wall following.");
}
 
// loop()
void loop() {
 
  if (state == WALL_FOLLOWING) {
 
    wallFollowing();
 
    // read IR sensors
    lineSensors.readCalibrated(lineSensorValues);
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
 
  } else if (state == PICK_SERVICE) {
 
    serviceBin();
 
    if (binCount >= NUM_BINS) {
      // all bins collected — switch to return to dock
      Serial.println("All bins collected. Returning to dock.");

    //drive off last bin before going to return to doc so we dont sense for light brown early - split out to a 'clear cell' state?
    //but the fact that it is sensing the last black bin as light brown means light brown is not working properly
    motors.setSpeeds(BASE_SPEED, BASE_SPEED);
    delay(1000);  // drive off start square 
      state = RETURN_TO_DOCK;
    } else {
      state = WALL_FOLLOWING;
    }
 
  } else if (state == RETURN_TO_DOCK) {
 
    // continue wall following back to dock
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
 
  double PDout = PDcontroller.update(actualWallDist, GOAL_WALL_DIST);
 
  // int16_t leftSpeed  = constrain(BASE_SPEED + PDout, -400, 400);
  // int16_t rightSpeed = constrain(BASE_SPEED - PDout, -400, 400);
//reversing to test 
  int16_t leftSpeed  = constrain(BASE_SPEED - PDout, -400, 400);
  int16_t rightSpeed = constrain(BASE_SPEED + PDout, -400, 400);

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
