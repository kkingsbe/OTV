#include "motorcontroller.h"

MotorController* motorController = new MotorController();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motorController->init();
  motorController->setDriveSpeed(1.0);
}

float i = -1.0;

void loop() {
  motorController->setSteerBias(i);
  if(i < 1.0) {
    i += 0.05;
  } else {
    i = -1.0;
  }
  motorController->tick();
  delay(30);
}
