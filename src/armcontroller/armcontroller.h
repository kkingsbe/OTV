#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>

#define ARM_PWM 11
#define ARM_DIR 10
#define LIMIT_SWITCH A0
#define BURST_SPEED 255.0
#define BURST_LENGTH 50     //How many ms to burst the motor for to get it moving
#define ARM_SPEED 100.0
#define RECENTER_DELAY 800  //How many ms to wait before stopping the arm when recentering (after pressing limit switch)
#define SPIN_DELAY 500      //How many ms to wait before stopping the arm when bringing in the block

enum ArmState {
    MOVING,
    RESETTING,
    STOPPED,
    COASTING
};

enum ResetState {
    NONE,
    APPROACHING_LIMIT_SWITCH, //Waiting until it rotates to hit the limit switch
    CENTERING                 //Continue moving past the limit switch for some distance until at resting position
};

enum ArmDirection {
    FORWARD = LOW,
    BACKWARD = HIGH
};

class ArmController {
public:
    ArmController(); // Constructor
    void stop();
    void updateSpeed(float speed, bool forwards);
    void init(); // Function to initialize the motor controller
    void tick();
    void spin();
    void resetPosition();
private:
    float speed;
    float currentSpeed;
    ArmState state;
    void commandMotors(); // Function to command the motors
    bool limitSwitchPressed();
    ResetState resetState;
    long limitSwitchTime;
    long burstStartTime;
};

#endif // ARM_CONTROLLER_H