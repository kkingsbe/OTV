#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <DistanceSensor.h>

//Configure sensor pins here
#define FRONT_TRIG 13
#define FRONT_ECHO 12
#define LEFT_TRIG 11
#define LEFT_ECHO 10
#define RIGHT_TRIG 9
#define RIGHT_ECHO 8

struct RangeData {
    float front;
    float left;
    float right;
};

class SensorManager {
public:
    SensorManager(); // Constructor
    float getWeight(); // Function to get the weight determined by the loadcells
    RangeData getRange(); // Function to get the range determined by the ultrasonic sensor

private:
    DistanceSensor left_ultra;
    DistanceSensor right_ultra;
    DistanceSensor front_ultra;
};

#endif // SENSOR_MANAGER_H