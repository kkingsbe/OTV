#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

//Configure sensor pins here
#define FRONT_L_TRIG 13
#define FRONT_L_ECHO 12
#define FRONT_R_TRIG 11
#define FRONT_R_ECHO 10
#define LEFT_TRIG 9
#define LEFT_ECHO 8
#define RIGHT_TRIG 7
#define RIGHT_ECHO 6

struct RangeData {
    float front;
    float left;
    float right;
};

enum UltrasonicSensor {
    FRONT,
    LEFT,
    RIGHT
};

class SensorManager {
public:
    SensorManager(); // Constructor
    void init();
    long takeReading(UltrasonicSensor sensor);
    float durationToCm(long duration); // Function to convert duration to cm (for ultrasonic sensors
    float getWeight(); // Function to get the weight determined by the loadcells
    RangeData getRange(); // Function to get the range determined by the ultrasonic sensor
};

#endif // SENSOR_MANAGER_H