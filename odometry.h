#ifndef Odometry_h
#define Odometry_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class Odometry{
  public:
    Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning);

    // helper function covertToDegrees converts theta to degrees
    // calle dby update_odom
    float convertToDegrees(float theta);

    float convertToRadian(float angle);

    //function printSerial() prints out the tuple [x, y, theta] to the serial monitor
    //called by update_odom
    void printSerial();

    // overloaded function to print out the encoderCounts to the Serial Monitor
    void printSerial(int16_t encCountsLeft, int16_t encCountsRight);

    void update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta);

    
  private:
    float _diaL;
    float _diaR;
    float _w;
    int _nL;
    int _nR;
    int _gearRatio;
    IMU _imu;
    float _IMUavg_error;
    bool _deadreckoning;
    int _left_encoder_counts_prev;
    int _right_encoder_counts_prev;

    double _x;
    double _y;
    double _theta;
};

#endif
