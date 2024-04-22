#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

//Configure sensor pins here
#define FRONT_L_TRIG 7  //Working
#define FRONT_L_ECHO 9  //Working
#define FRONT_R_TRIG 11 //Working
#define FRONT_R_ECHO 12 //Working
#define LEFT_TRIG 6     //Working
#define LEFT_ECHO 8     //Working
#define RIGHT_TRIG 10   //Working
#define RIGHT_ECHO 13   //Working

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