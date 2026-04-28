#ifndef BROWN_LINE_FOLLOWER_H
#define BROWN_LINE_FOLLOWER_H

#include <Arduino.h>
#include <Pololu3piPlus32U4.h>

// =============================================================================
// BrownLineFollower.h
// Sensor utility for brown line detection on the Pololu 3pi+ 32U4.
//
// Provides:
//   - Calibrated sensor reads
//   - Weighted-average line position estimation across all 5 sensors
//
// Calibration, motor control, PD/PID correction, threshold values, and state
// decisions are intentionally left to the student's sketch.
//
// This class borrows a reference to the student's already-calibrated
// LineSensors instance, so no duplicate sensor objects or separate
// calibration step is needed here.
//
// Scale reference:
//   readCalibrated() returns values per sensor on a 0..1000 scale:
//     0    = brightest surface seen during calibration (e.g. white vinyl floor)
//     1000 = darkest surface seen during calibration (e.g. black tape)
//
//   computeLinePosition() returns a position on a 0..4000 scale:
//     This is NOT a sensor reading — it is the estimated location of the
//     line across the width of the robot's sensor array.
//     0    = line is under the far left sensor  (cal[0])
//     2000 = line is centered under the robot   (cal[2])
//     4000 = line is under the far right sensor (cal[4])
//
// Threshold reference (White Vinyl floor, current lab materials):
//   All threshold values are on the 0..1000 calibrated sensor scale.
//   White Vinyl (floor)        :   0 –  20
//   Black tape / squares       : 900 – 1000
//   Dark brown crafting foam   : 320 –  650  <- line color
//   Light brown crafting foam  : 130 –  300
// =============================================================================

class BrownLineFollower {
public:
    // -------------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   lineSensors  - Reference to the LineSensors instance the student has
    //                  already calibrated in their setup(). The class borrows
    //                  this reference and does not take ownership.
    //   darkBrownMin - Lower bound of the dark brown band (0..1000 scale)
    //   darkBrownMax - Upper bound of the dark brown band (0..1000 scale)
    // -------------------------------------------------------------------------
    BrownLineFollower(Pololu3piPlus32U4::LineSensors &lineSensors,
                      uint16_t darkBrownMin,
                      uint16_t darkBrownMax);

    // -------------------------------------------------------------------------
    // readCalibrated()
    // Fills a 5-element array with the current calibrated sensor readings.
    // Call each loop iteration to get fresh values before computing position.
    //
    // Parameters:
    //   cal - uint16_t array of length 5, populated by this call
    //
    // Each value in cal[] is on the 0..1000 scale:
    //   0    = brightest surface seen during calibration
    //   1000 = darkest surface seen during calibration
    //
    // Sensor index reference:
    //   cal[0] = far left sensor
    //   cal[2] = center sensor
    //   cal[4] = far right sensor
    // -------------------------------------------------------------------------
    void readCalibrated(uint16_t cal[5]);

    // -------------------------------------------------------------------------
    // computeLinePosition()
    // Estimates where the dark brown line is under the robot using a
    // weighted average across all 5 sensors.
    //
    // Only sensors whose reading (0..1000) falls within [darkBrownMin, darkBrownMax]
    // contribute, so edge-sensor drift is handled correctly even when the
    // center sensor reads floor.
    //
    // Parameters:
    //   cal      - Array of 5 calibrated readings in 0..1000 scale (from readCalibrated)
    //   position - Output: estimated line position on 0..4000 scale
    //                0    = line at far left sensor
    //                2000 = line centered under robot
    //                4000 = line at far right sensor
    //
    // Returns true  if the line was detected by at least one sensor.
    // Returns false if no sensor fell within the dark brown band.
    // -------------------------------------------------------------------------
    bool computeLinePosition(const uint16_t cal[5], uint16_t &position);

private:
    Pololu3piPlus32U4::LineSensors &_lineSensors; // borrowed reference, not owned
    uint16_t _darkBrownMin; // 0..1000 calibrated scale
    uint16_t _darkBrownMax; // 0..1000 calibrated scale
};

#endif // BROWN_LINE_FOLLOWER_H
