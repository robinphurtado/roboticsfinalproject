#ifndef PDcontroller_h
#define PDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PDcontroller{
  public:
    PDcontroller(float kp, float kd, double minOutput, double maxOutput);
    double update(double value, double target_value); //may need to update with additional variables passed in this function
    
  private:
    //Add private variables here
    //Hint: You are adding 4 more variables to the
    //      ones from Pcontroller
	  float _kp;
    float _kd;
    unsigned long _prevTime;
    double _derivative;
    double _minOutput;
    double _maxOutput;
    double _error;
    double _prevError;
    double _output;
    double _clampOut;	
};

#endif
