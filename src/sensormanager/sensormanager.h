#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "../scalemanager/scalemanager.h"

//Configure sensor pins here
#define FRONT_L_TRIG A4//A3 //Working
#define FRONT_L_ECHO A4//9  //Working
#define FRONT_R_TRIG A1 //Working
#define FRONT_R_ECHO 12 //Working
#define LEFT_TRIG A5    //Working
#define LEFT_ECHO 8     //Working
#define RIGHT_TRIG A2   //Working
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
    void tick();
private:
    ScaleManager scale;
};

#endif // SENSOR_MANAGER_H