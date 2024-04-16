#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

//Configure sensor pins here
#define FRONT_L_TRIG 10 //working
#define FRONT_L_ECHO 12 //working
#define FRONT_R_TRIG 13
#define FRONT_R_ECHO 11
#define LEFT_TRIG 13 //working
#define LEFT_ECHO 9 //working
#define RIGHT_TRIG 6
#define RIGHT_ECHO 8

struct RangeData {
    float front_l;
    float front_r;
    float left;
    float right;
};

enum UltrasonicSensor {
    FRONT_L,
    FRONT_R,
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