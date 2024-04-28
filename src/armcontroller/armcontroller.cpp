#include "armcontroller.h"

ArmController::ArmController(): 
    speed(ARM_SPEED),
    currentSpeed(0.0),
    state(STOPPED),
    resetState(NONE),
    burstStartTime(0),
    limitSwitchTime(0)
{}

void ArmController::init() {
    // Initialize the motor controller
    pinMode(ARM_PWM, OUTPUT);
    pinMode(ARM_DIR, OUTPUT);
    pinMode(LIMIT_SWITCH, OUTPUT);
    digitalWrite(LIMIT_SWITCH, HIGH);

    digitalWrite(ARM_DIR, ArmDirection::FORWARD);
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
}

void ArmController::tick() {
    if(limitSwitchPressed()) {
        if(state == RESETTING) {
            resetState = CENTERING;
            limitSwitchTime = millis();
        } else {
            stop();
        }
    } else {
        if(millis() - burstStartTime < BURST_LENGTH) {
            currentSpeed = BURST_SPEED;
        } else {
            currentSpeed = speed;
        }

        this->commandMotors();
    }

    if(resetState == CENTERING && millis() - limitSwitchTime > RECENTER_DELAY) {
        stop();
        resetState = NONE;
    }
}

void ArmController::commandMotors() {
    Serial.println("State: " + String(state) + " Speed: " + String(currentSpeed));
    if(state == MOVING || state == RESETTING) {
        digitalWrite(ARM_DIR, ArmDirection::FORWARD);
        analogWrite(ARM_PWM, abs(currentSpeed));
    } else {
        digitalWrite(ARM_PWM, LOW);
    }
}

bool ArmController::limitSwitchPressed() {
    return digitalRead(LIMIT_SWITCH) == LOW;
}

void ArmController::spin() {
    burstStartTime = millis();
    updateSpeed(speed, true);
    state = MOVING;
    tick();
}

void ArmController::resetPosition() {
    burstStartTime = millis();
    state = RESETTING;
    resetState = APPROACHING_LIMIT_SWITCH;
    updateSpeed(speed, true);
    tick();
}