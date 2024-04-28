#include "armcontroller.h"

ArmController::ArmController(float _speed): 
    speed(_speed),
    currentSpeed(0.0),
    state(STOPPED) 
{}

void ArmController::init() {
    // Initialize the motor controller
    pinMode(ARM_PWM, OUTPUT);
    pinMode(ARM_DIR, OUTPUT);
    pinMode(LIMIT_SWITCH, OUTPUT);
    digitalWrite(LIMIT_SWITCH, HIGH);
}
 
void ArmController::stop() {
    currentSpeed = 0.0;
    state = STOPPED;
}

void ArmController::updateSpeed(float _speed, bool forwards) {
    if(forwards) {
        currentSpeed = _speed;
    } else {
        currentSpeed = -_speed;
    }
    state = MOVING;
}

void ArmController::tick() {
    if(limitSwitchPressed()) {
        stop();
    } else {
        this->commandMotors();
    }
}

void ArmController::commandMotors() {
    if(state == MOVING) {
        digitalWrite(ARM_DIR, currentSpeed > 0 ? HIGH : LOW);
        analogWrite(ARM_PWM, abs(currentSpeed));
    } else {
        digitalWrite(ARM_DIR, LOW);
        analogWrite(ARM_PWM, 0);
    }
}

bool ArmController::limitSwitchPressed() {
    return digitalRead(LIMIT_SWITCH) == LOW;
}

void ArmController::spin() {
    updateSpeed(speed, true);
    state = MOVING;
    tick();
}