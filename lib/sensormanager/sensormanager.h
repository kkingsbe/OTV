#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

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
};

#endif // SENSOR_MANAGER_H