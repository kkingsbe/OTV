 #include "src/motorcontroller/motorcontroller.h"
#include "src/sensormanager/sensormanager.h"
#include "src/guidancemanager/guidancemanager.h"
#include "src/armcontroller/armcontroller.h"

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();
GuidanceManager* guidanceManager = new GuidanceManager();
ArmController* armController = new ArmController();

float speed = 0.5;
float columnOneX = 1.25;
float columnTwoX = 2.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initializing...");
  delay(500);

  sensorManager->init();
  motorController->init();
  motorController->setDriveSpeed(speed);

  //Serial.println("Setting up guidance manager");

  guidanceManager->init();
  //Becomes unstable at kp=0.8
  //guidanceManager->setPidConfig(0.8, 0.01, 0.2);
  guidanceManager->setPidConfig(0.8, 0.01, 0.4);
  guidanceManager->setZeroPointPidConfig(0.6, 0.15, 0.05);
  guidanceManager->setCirclePidConfig(2.0, 0, 0);

  //Potential Starts  
  guidanceManager->addWaypoint(0.25, 0.43, 0);
  guidanceManager->addWaypoint(0.25, 1.57, 2);
  //guidanceManager->addWaypoint(3.3, 0.54, 0);
  //guidanceManager->addWaypoint(3.3, 1.49, 1);

  //Potential scan points
  //guidanceManager->addWaypoint(3.3, 0.54, 2, true, -PI/2.0);
  //guidanceManager->addWaypoint(3.3, 1.49, 3, true, PI/2.0);

  //guidanceManager->addWaypoint(0.25, 0.54, 2, true, -PI/2.0);
  //guidanceManager->addWaypoint(0.25, 1.49, 3, true, PI/2.0);

  //Run into the wall after the mission site
  guidanceManager->addWaypoint(0.25, 0.1, 1);
  guidanceManager->addWaypoint(0.25, 1.9, 3);
  /*
  guidanceManager->addWaypoint(3.3, 0.2, 4);
  guidanceManager->addWaypoint(3.3, 1.8, 5);
  */

  //Navigation waypoints
  guidanceManager->addWaypoint(1.3, 0.6, 4, false, 0, true, 0, 0);
  guidanceManager->addWaypoint(1.3, 1.3, 5, false, 0, true, 1, 0);
  guidanceManager->addWaypoint(1.3, 1.7, 6, false, 0, true, 2, 0);

  guidanceManager->addWaypoint(1.9, 0.6, 9, false, 0, true, 0, 1);
  guidanceManager->addWaypoint(1.9, 1.3, 8, false, 0, true, 1, 1);
  guidanceManager->addWaypoint(1.9, 1.7, 7, false, 0, true, 2, 1);

  //Clear last column
  guidanceManager->addWaypoint(3.0, 0.6, 10);
  guidanceManager->addWaypoint(3.0, 1.0, 11);
  guidanceManager->addWaypoint(3.0, 1.4, 12);

  guidanceManager->addWaypoint(3.0, 1.5, 13);  //Line up with limbo
  guidanceManager->addWaypoint(3.5, 1.5, 14);  //Pass limbo

  //guidanceManager->setActiveWaypoint(2);
  guidanceManager->setActiveWaypoint(0);

  armController->init();

  motorController->tick();
  armController->resetPosition();
}

long last_time = millis();

void loop() {
  //Determine starting waypoint
  VehiclePosition* pos = guidanceManager->getPosition();

  RangeData rd = sensorManager->getRange();
  GuidanceInfo gi = guidanceManager->tick(&rd);

  motorController->setDriveSpeed(gi.driveSpeed);
  motorController->setSteerBias(gi.steerBias);
  
  //Serial.println("Front Left: " + String(rd.front_l) + " | Front Right: " + String(rd.front_r) + " | Left: " + String(rd.left) + " | Right: " + String(rd.right));

  motorController->tick();
  armController->tick();
  //sensorManager->tick();

  delay(30);
}
