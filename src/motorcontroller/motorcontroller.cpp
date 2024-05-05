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
    setDriveSpeed(0.0);
}
 
//Speed is between 0 and 1
void MotorController::setDriveSpeed(float speed) {
    // Set the drive speed
    motors.driveSpeed = speed;
}

/**
 * @brief Set the steer angle
 * @var steerAngle The steer angle to set. Ranges from -1 (full left) to 1 (full right).
*/
void MotorController::setSteerBias(float steerAngle) {
    if(steerAngle <= 0) { //Turn left
        this->motors.left = (2.0 * steerAngle) + 1.0;
        this->motors.right = 1.0;

        //Serial.println("Left: " + String(this->motors.left) + " Right: " + String(this->motors.right));
    } else { //Turn right
        this->motors.left = 1.0;
        this->motors.right = (-2.0 * steerAngle) + 1.0;

        //Serial.println("Left: " + String(this->motors.left) + " Right: " + String(this->motors.right));
    }
}

void MotorController::commandMotors() {
    digitalWrite(M1, this->motors.left > 0 ? HIGH : LOW);
    digitalWrite(M2, this->motors.right > 0 ? LOW : HIGH);

    float maxSpeed = this->motors.driveSpeed * 255;

    int speedLeft = round(this->motors.left * maxSpeed);
    int speedRight = round(this->motors.right * maxSpeed);

    if(speedLeft < 0) speedLeft *= -1;
    if(speedRight < 0) speedRight *= -1;
 
    //Serial.println("Left: " + String(speedLeft) + " Right: " + String(speedRight));

    analogWrite(E1, speedLeft);
    analogWrite(E2, speedRight);
}

void MotorController::tick() {
    this->commandMotors();
}