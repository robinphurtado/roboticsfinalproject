#include "BrownLineFollower.h"

// =============================================================================
// BrownLineFollower.cpp
// See BrownLineFollower.h for full documentation.
// =============================================================================

// ---------------------------------------------------------------------------
// Constructor
// Stores the borrowed LineSensors reference and the student-defined thresholds.
// ---------------------------------------------------------------------------
BrownLineFollower::BrownLineFollower(Pololu3piPlus32U4::LineSensors &lineSensors,
                                     uint16_t darkBrownMin,
                                     uint16_t darkBrownMax)
    : _lineSensors(lineSensors),
      _darkBrownMin(darkBrownMin),
      _darkBrownMax(darkBrownMax)
{
}

// ---------------------------------------------------------------------------
// readCalibrated()
// Delegates directly to the student's shared LineSensors instance.
// Each value in cal[] is on the 0..1000 calibrated scale.
// ---------------------------------------------------------------------------
void BrownLineFollower::readCalibrated(uint16_t cal[5])
{
    _lineSensors.readCalibrated(cal);
}

// ---------------------------------------------------------------------------
// computeLinePosition()
//
// Input:  cal[] values are each on the 0..1000 calibrated sensor scale.
// Output: position is on the 0..4000 scale representing where the line
//         sits across the physical width of the robot's sensor array.
//
// Using all 5 sensors (not just the center 3) ensures the robot can still
// detect and correct when the line has drifted all the way to the outermost
// sensors — cases where checking only the center sensor would miss the line.
// ---------------------------------------------------------------------------
bool BrownLineFollower::computeLinePosition(const uint16_t cal[5], uint16_t &position)
{
    uint32_t weightedSum = 0;
    uint32_t total       = 0;

    for (int i = 0; i < 5; i++) {
        // val is a calibrated sensor reading on the 0..1000 scale
        uint16_t val = cal[i];

        if (val >= _darkBrownMin && val <= _darkBrownMax) {
            // Activation strength — how strongly this sensor sees the brown line.
            // Zero at band minimum, larger toward band maximum (still 0..1000 scale).
            uint16_t activation = val - _darkBrownMin;

            // sensorPos is this sensor's location on the 0..4000 position scale.
            // This is NOT a sensor reading — it maps sensor index to robot width:
            //   i=0 -> 0    (far left)
            //   i=1 -> 1000
            //   i=2 -> 2000 (center)
            //   i=3 -> 3000
            //   i=4 -> 4000 (far right)
            uint16_t sensorPos = i * 1000;

            weightedSum += (uint32_t)activation * sensorPos;
            total       += activation;
        }
    }

    // If no sensor fell within the dark brown band, the line is not detected
    if (total == 0) {
        return false;
    }

    // Weighted average gives the estimated center of the line on the 0..4000 scale
    position = (uint16_t)(weightedSum / total);
    return true;
}
