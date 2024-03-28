#include "motorcontroller.h"
#include "sensormanager.h"
#include <DistanceSensor.h>

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motorController->init();
  motorController->setDriveSpeed(1.0);
}

float i = -1.0;
bool increasing = true;

void loop() {
  /*
  motorController->setSteerBias(i);
  
  if(i >= 1.0) {
    increasing = false;
  }

  if(i <= -1.0) {
    increasing = true;
  }

  if(increasing) {
    i += 0.05;
  } else {
    i -= 0.05;
  }
  motorController->tick();
  */

  RangeData range = sensorManager->getRange();
  
  // Write values to serial port
  Serial.print("Distance: ");
  Serial.print(range.front);
  Serial.println("cm");
  //delay();
}
