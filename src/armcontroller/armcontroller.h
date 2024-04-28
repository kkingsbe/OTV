#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>

#define ARM_PWM 5
#define ARM_DIR 4
#define LIMIT_SWITCH 0//TBD

enum ArmState {
    MOVING,
    STOPPED
};

class ArmController {
public:
    ArmController(float speed); // Constructor
    void stop();
    void updateSpeed(float speed, bool forwards);
    void init(); // Function to initialize the motor controller
    void tick();
    void spin();
private:
    float speed;
    float currentSpeed;
    ArmState state;
    void commandMotors(); // Function to command the motors
    bool limitSwitchPressed();
};

#endif // ARM_CONTROLLER_H