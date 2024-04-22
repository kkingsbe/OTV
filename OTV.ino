#include "src/motorcontroller/motorcontroller.h"
#include "src/sensormanager/sensormanager.h"
#include "src/guidancemanager/guidancemanager.h"

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();
GuidanceManager* guidanceManager = new GuidanceManager();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initializing...");

  sensorManager->init();

  motorController->init();
  motorController->setDriveSpeed(1.0);

  Serial.println("Skipping guidance manager config");

  /*
  Serial.println("Setting up guidance manager");

  guidanceManager->init();
  guidanceManager->setPidConfig(0.2, 0.01, 0.15);

  guidanceManager->addWaypoint(0.5, 0.5);
  guidanceManager->addWaypoint(1.75, 1.0);
  guidanceManager->addWaypoint(0.5, 1.0);
  */
}

long last_time = millis();
float waypoint_distance_threshold = 0.25; //m

void loop() {
  /*
  float dt = (millis() - last_time) / 1000.0;
  last_time = millis();
  
  */
  //guidanceManager->tick();
  //float setpoint = guidanceManager->getUpdatedSteerBias();
  //motorController->setSteerBias(setpoint);  
  //motorController->tick();

  /*
  float distance_error = guidanceManager->getDistanceError();

  if(guidanceManager->getDistanceError() < waypoint_distance_threshold) {
    guidanceManager->nextWaypoint();
  }
  */
  
  RangeData rd = sensorManager->getRange();
  Serial.println("Front Left: " + String(rd.front_l) + " | Front Right: " + String(rd.front_r) + " | Left: " + String(rd.left) + " | Right: " + String(rd.right));

  delay(30);
}
