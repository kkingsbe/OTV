#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

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