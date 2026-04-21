#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include "odometry.h"
#include "printOLED.h"

using namespace Pololu3piPlus32U4;

#define PI 3.14159

PrintOLED printOLED;

Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning){
  _diaL = diaL;
  _diaR = diaR;
  _w = w;
  _nL = nL;
  _nR = nR;
  _gearRatio = gearRatio;
  _deadreckoning = dead_reckoning;

  _x = 0; 
  _y = 0;
  _theta = 0;

  _left_encoder_counts_prev = 0;
  _right_encoder_counts_prev = 0;

  if(_deadreckoning){ // if using dead reckoning, initialize and calibrate IMU
    // initialize wire library and join the IC2 bus as controller or peripheral
    Wire.begin();
    // initializes the interial sensors and detects their type
    _imu.init();
    // enables all the inertial sensors with a default configuration
    _imu.enableDefault();
    
    //calibrate IMU
    int total = 0;
    for (int i = 0; i < 100; i++)
    {
      // take a reading from the gyro and makes its measurements availble in g
      _imu.readGyro();
      // increment the z value
      total += _imu.g.z;
      // wait one millisecond before next loop
      delay(1);
    }
    // calculate the average of the 100 readings collected in above for loop
    _IMUavg_error = total / 100;  
  }
}

// convert rads to degrees to output to OLED
//accepts parameter theta in radians, returns angle in degrees
float Odometry::convertToDegrees(float theta) { 
  float angle_degrees = theta*(180.0/PI);
  return angle_degrees;
}

// convert degrees to rads
//accepts angle in degress, returns angle in radian
float Odometry::convertToRadian(float angle) { 
  float angle_theta = (angle/180)*PI;
  return angle_theta;
}

void Odometry::printSerial(){
  float angle = convertToDegrees(_theta);
  Serial.print("["); 
  Serial.print(_x, 3);
  Serial.print(", ");
  Serial.print(_y, 3);
  Serial.print(", ");
  Serial.print(angle, 3); 
  Serial.println("]");
}

//overloaded version
void Odometry::printSerial(int16_t encCountsLeft, int16_t encCountsRight){
  Serial.print(encCountsLeft);
  Serial.print(", ");
  Serial.print(encCountsRight);
}

// USE ODOMETRY FORMULAS TO CALCULATE ROBOT'S NEW POSITION AND ORIENTATION
void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta){
///* Uncomment after Steps 1 &  2	
  float L = left_encoder_counts - _left_encoder_counts_prev;
  float R = right_encoder_counts - _right_encoder_counts_prev;
  
  float NL = _nL * _gearRatio;
  float NR = _nR * _gearRatio;  
  float deltaL = (PI *_diaL * L) / NL;  
  float deltaR = (PI *_diaR * R) / NR;  
  float delta_d = (deltaR+deltaL)/2;


  if(_deadreckoning){ // IF USING dead reckoning, GET THE ANGLE _theta FROM IMU
    _imu.readGyro();
    float angleRate= (_imu.g.z - _IMUavg_error);
    _theta += angleRate * 0.0001;
  }else{// OTHERWISE, CALCULATE THE ANGLE _theta FROM ENCODERS
    _theta += (deltaR-deltaL)/_w;
  }  

  // CALCULATE _x BASED ON THE FORMULA FROM THE LECTURES
  _x += delta_d * (cos(_theta));
  
  // CALCULATE _y BASED ON THE FORMULA FROM THE LECTURES
  _y += delta_d * (sin(_theta));

  // CALCULATE CUMULATIVE x, AND CUMULATIVE y. 
  //AKA UPDATE THE VALUE OF &x AND &y (THE PARAMETERS OF THE update_odom FUNCTIONS, WHICH ARE PASSED BY REFERENCE)
  x = _x;
  y = _y;

  //REMINDER: CUMULATIVE theta IS EQUAL TO _theta.
  theta = _theta;

  // PRINT THE x, y, theta VALUES ON OLED
  //convert theta in radians to angle in degrees
  float angle_degrees = convertToDegrees(theta);
  printOLED.print_odom(x, y, angle_degrees);

  // PRINT THE x, y, theta VALUES ON SERIAL MONITOR
  Odometry::printSerial();

  // Save the current encoder values as the "previous" values, so you can use it in the next iteration
  _left_encoder_counts_prev = left_encoder_counts;
  _right_encoder_counts_prev = right_encoder_counts;

//*/

}
