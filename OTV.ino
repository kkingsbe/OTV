#include "motorcontroller.h"

MotorController* motorController = new MotorController();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motorController->init();
  motorController->setDriveSpeed(1.0);
}

float i = -1.0;
bool increasing = true;

void loop() {
  motorController->setSteerBias(i);
  
  if(i >= 1.0) {
    increasing = false;
  }

  if(i <= -1.0) {
    increasing = true;
  }

  if(increasing) {
    i += 0.1;
  } else {
    i -= 0.1;
  }
  motorController->tick();
  delay(30);
}
