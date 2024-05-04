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

  Serial.println("Setting up guidance manager");

  guidanceManager->init();
  //Becomes unstable at kp=0.8
  //guidanceManager->setPidConfig(0.8, 0.01, 0.2);
  guidanceManager->setPidConfig(0.8, 0.01, 0.4);

  //Potential Starts
  //guidanceManager->addWaypoint(0.42, 0.54, 0);
  //guidanceManager->addWaypoint(0.42, 1.49, 1);

  //Navigation waypoints
  guidanceManager->addWaypoint(1.1, 0.6, 2, false, 0, true, 0, 0);
  guidanceManager->addWaypoint(1.1, 1.1, 3, false, 0, true, 1, 0);
  guidanceManager->addWaypoint(1.1, 1.4, 4, false, 0, true, 2, 0);

  guidanceManager->addWaypoint(2.1, 0.6, 7, false, 0, true, 0, 1);
  guidanceManager->addWaypoint(2.1, 1.1, 6, false, 0, true, 1, 1);
  guidanceManager->addWaypoint(2.1, 1.4, 5, false, 0, true, 2, 1);

  //Clear last column
  guidanceManager->addWaypoint(3.0, 0.6, 8);
  guidanceManager->addWaypoint(3.0, 1.0, 9);
  guidanceManager->addWaypoint(3.0, 1.4, 10);

  guidanceManager->addWaypoint(3.0, 1.5, 11);  //Line up with limbo
  guidanceManager->addWaypoint(3.5, 1.5, 12);  //Pass limbo

  guidanceManager->setActiveWaypoint(2);
  //guidanceManager->setActiveWaypoint(0);

  armController->init();

  motorController->tick();
  armController->resetPosition();
}

long last_time = millis();
//bool has_init = false;
bool has_init = true;

//Determines if the vehicle started at waypoint 0 or 1
void determineStartPoint() {
  VehiclePosition* pos = guidanceManager->getPosition();

  //The two possible starting positions
  Waypoint* waypoint0 = guidanceManager->getWaypoint(0);
  Waypoint* waypoint1 = guidanceManager->getWaypoint(1);
  
  //If vehicle did not start at waypoint 0
  if(guidanceManager->getDistanceError() > 0.25) {
    //Swap the two waypoints
    guidanceManager->getWaypoint(0)->index = 1;
    guidanceManager->getWaypoint(1)->index = 0;
  }
}

void loop() {
  //Determine starting waypoint
  if(!has_init) {
    VehiclePosition* pos = guidanceManager->getPosition();
    Serial.println("x: " + String(pos->x) + ", y: " + String(pos->y) + ", Valid: " + String(pos->valid));
    if(guidanceManager->getPosition()->valid == true) {
      Serial.println("Valid position. Determining start waypoint");
      determineStartPoint();
      has_init = true;
    } else {
      Serial.println("Waiting for valid position");
    }
  }

  VehiclePosition* pos = guidanceManager->getPosition();

  if(!pos->valid) {
    //Serial.println("Invalid position. Waiting");
    motorController->setDriveSpeed(0.0);
  } else {
    //Serial.println("Valid position. Moving");
    motorController->setDriveSpeed(speed);
  }

  RangeData rd = sensorManager->getRange();
  GuidanceInfo gi = guidanceManager->tick(&rd);

  motorController->setDriveSpeed(gi.driveSpeed);
  motorController->setSteerBias(gi.steerBias);
  
  //Serial.println("Front Left: " + String(rd.front_l) + " | Front Right: " + String(rd.front_r) + " | Left: " + String(rd.left) + " | Right: " + String(rd.right));

  motorController->tick();
  armController->tick();

  delay(30);
}
 
  
  
