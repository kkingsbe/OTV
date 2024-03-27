#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

#define M1 4
#define M2 7
#define E1 5
#define E2 6

struct Motors {
    float left;
    float right;
    float driveSpeed;
};

class MotorController {
public:
    MotorController(); // Constructor
    void setDriveSpeed(float speed); // Function to set the drive speed
    void setSteerBias(float steerAngle); // Function to set the steer angle
    void init(); // Function to initialize the motor controller
    void tick();
    
private:
    Motors motors;
    void commandMotors(); // Function to command the motors
};

#endif // SENSOR_MANAGER_H
