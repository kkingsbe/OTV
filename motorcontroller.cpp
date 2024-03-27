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
    this->motors.left = 1.0 - steerAngle;
    this->motors.right = 1.0 + steerAngle;
}

void MotorController::commandMotors() {
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);

    float maxSpeed = this->motors.driveSpeed * 255;

    Serial.println("Max Speed: " + String(maxSpeed) + "Left: " + String(abs(this->motors.left) * maxSpeed) + "Right: " + String(abs(this->motors.right) * maxSpeed));

    analogWrite(E1, abs(this->motors.left) * maxSpeed);
    analogWrite(E2, abs(this->motors.right) * maxSpeed);
}

void MotorController::tick() {
    this->commandMotors();
}
