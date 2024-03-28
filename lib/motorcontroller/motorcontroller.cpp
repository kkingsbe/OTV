#include "motorcontroller.h"

MotorController::MotorController() {
    // Initialize the motor controller class
}

void MotorController::init() {
    // Initialize the motor controller
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
}
 
void MotorController::setDriveSpeed(float speed) {
    // Set the drive speed
    motors.driveSpeed = speed;
}

/**
 * @brief Set the steer angle
 * @var steerAngle The steer angle to set. Ranges from -1 (full left) to 1 (full right).
*/
void MotorController::setSteerBias(float steerAngle) {
    if(steerAngle < 0) {
        this->motors.left = 0.5 - (-1.5 * steerAngle);
        this->motors.right = 1.0;
    } else if(steerAngle > 0) {
        this->motors.left = 1.0;
        this->motors.right = 0.5 - (1.5 * steerAngle);
    } else {
        this->motors.left = 1.0;
        this->motors.right = 1.0;
    }
    Serial.println("Left: " + String(this->motors.left));
}

void MotorController::commandMotors() {
    digitalWrite(M1, this->motors.left > 0 ? HIGH : LOW);
    digitalWrite(M2, this->motors.right > 0 ? HIGH : LOW);

    float maxSpeed = this->motors.driveSpeed * 255;

    //Serial.println("Max Speed: " + String(maxSpeed) + "Left: " + String(abs(this->motors.left) * maxSpeed));

    analogWrite(E1, abs(this->motors.left) * maxSpeed);
    analogWrite(E2, abs(this->motors.right) * maxSpeed);
}

void MotorController::tick() {
    this->commandMotors();
}