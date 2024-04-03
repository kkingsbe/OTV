#include "src/motorcontroller/motorcontroller.h"
#include "src/sensormanager/sensormanager.h"
#include "src/guidancemanager/guidancemanager.h"
#include <DistanceSensor.h>

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();
GuidanceManager* guidanceManager = new GuidanceManager();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motorController->init();
  motorController->setDriveSpeed(1.0);
  guidanceManager->setPidConfig(0.3, 0.008, 0.14);

  guidanceManager->addWaypoint(0.5, 0.5);
  guidanceManager->addWaypoint(1.75, 1.0);
  guidanceManager->addWaypoint(0.5, 1.5);
}

long last_time = millis();
float waypoint_distance_threshold = 0.25; //m

void loop() {
  float dt = (millis() - last_time) / 1000.0;
  last_time = millis();
  guidanceManager->tick();
  float setpoint = guidanceManager->getUpdatedSteerBias();
  motorController->setSteerBias(setpoint);  
  motorController->tick();

  if(guidanceManager->getDistanceError() < waypoint_distance_threshold) {
    guidanceManager->nextWaypoint();
  }

  delay(30);
}
