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
    float clippedAngle = constrain(steerAngle, -1.0, 1.0);

    if(clippedAngle <= 0) {
        this->motors.left = 2.0 * clippedAngle + 1.0;
        this->motors.right = 1.0;
    } else if(clippedAngle >= 0) {
        this->motors.left = 1.0;
        this->motors.right = -2.0 * clippedAngle + 1.0;
    }
}

void MotorController::commandMotors() {
    digitalWrite(M1, this->motors.left > 0 ? HIGH : LOW);
    digitalWrite(M2, this->motors.right > 0 ? LOW : HIGH);

    float maxSpeed = this->motors.driveSpeed * 255;
    float speedLeft = round(abs(this->motors.left) * maxSpeed);
    float speedRight = round(abs(this->motors.right) * maxSpeed);

    Serial.println("Left: " + String(this->motors.left ? "Forwards" : "Reverse") + " " + String(speedLeft) + " Right: " + String(this->motors.right ? "Forwards" : "Reverse") + " " + String(speedRight));

    analogWrite(E1, speedLeft);
    analogWrite(E2, speedRight);
}

void MotorController::tick() {
    this->commandMotors();
}