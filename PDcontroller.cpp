#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(float kp, float kd, double minOutput, double maxOutput) {
  	_kp =kp;
    _kd = kd;
    _prevTime = 0;
    _derivative = 0.00;
    _minOutput = minOutput;
    _maxOutput = maxOutput;
    _error = 0.0;
    _prevError = 0.0;
    _output = 0.0;
    _clampOut = 0.0;
}

double PDcontroller::update(double value, double target_value){

  // get current time
  unsigned long currTime = millis();

  // calculate the error
  _error =  target_value - value; //switched in Isas
  //_error = value - target_value;
  

  // if prevTime == 0.00, this is the first run since it is
  // still in its intial state. 
  if(_prevTime == 0){

    // multiply error by gain
    _output = _kp * _error;
    //call contstrain to clamp within minOutput to maxOutput range
    _clampOut = constrain(_output, _minOutput, _maxOutput);

    _prevTime = currTime;
    //_prevError = _error;
    _prevError = 0.0;  //switched to IsasCode
  
    // return clamped output
    return _clampOut;
  
  // else this is not the first run, and we have P,D, and previous values
  } else {
    
    //calculate delta time. 
    //convert time to seconds because times are in milliseconds because we used millis() 
    double dt = (currTime - _prevTime)/1000.0;

    /* tried bounds checking for 0, robot stopped turning around the wall
    if (dt <= 0) {
      dt = 0.001;
    } */

    //calculate the derivative 
    _derivative = (_error -_prevError)/(dt);

    // add error multiplied by error gain to derivative multiplied by derivative gain
    _output = (_kp * _error) + (_kd * _derivative);

    //call contstrain to clamp within minOutput to maxOutput range
    _clampOut = constrain(_output, _minOutput, _maxOutput);

    // store error as _prevError for next loop
    _prevError = _error;
    //store currTime as prevTime for next loop
    _prevTime = currTime;

    //return clamoed output
    return _clampOut;
  }

}
