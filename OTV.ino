#include "src/motorcontroller/motorcontroller.h"
#include "src/sensormanager/sensormanager.h"
#include "src/guidancemanager/guidancemanager.h"
#include "src/armcontroller/armcontroller.h"

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();
GuidanceManager* guidanceManager = new GuidanceManager();
ArmController* armController = new ArmController(255.0);

float speed = 0.8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initializing...");
  delay(500);

  /*
  //sensorManager->init();

  motorController->init();
  motorController->setDriveSpeed(speed);

  Serial.println("Setting up guidance manager");

  guidanceManager->init();
  //Becomes unstable at kp=0.8
  guidanceManager->setPidConfig(0.8, 0.01, 0.2);
  //guidanceManager->setPidConfig(0.2, 0.01, 0.15);
  //guidanceManager->setPidConfig(5.0, 0.0, 0.0);

  //guidanceManager->addWaypoint(0.52, 0.54);
  //guidanceManager->addWaypoint(0.53, 1.49);

  //Potential Starts
  guidanceManager->addWaypoint(1.0, 0.54);
  guidanceManager->addWaypoint(1.0, 1.49);

  guidanceManager->addWaypoint(1.7, 0.42);
  guidanceManager->addWaypoint(1.7, 1.3);
  guidanceManager->addWaypoint(3.0, 1.3);
  guidanceManager->addWaypoint(3.81, 1.3);

  guidanceManager->setActiveWaypoint(0);

  */

  armController->init();
}

long last_time = millis();
float waypoint_distance_threshold = 0.25; //m
bool has_init = false;

//Determines if the vehicle started at waypoint 0 or 1
void determineStartPoint() {
  VehiclePosition* pos = guidanceManager->getPosition();

  //The two possible starting positions
  Waypoint* waypoint0 = guidanceManager->getWaypoint(0);
  Waypoint* waypoint1 = guidanceManager->getWaypoint(1);
  
  //If vehicle did not start at waypoint 0
  Serial.println("Distance to waypoint 1: " + String(guidanceManager->getDistanceError()));
  if(guidanceManager->getDistanceError() > waypoint_distance_threshold) {
    guidanceManager->setActiveWaypoint(1);
  }
}

void loop() {
  /*
  guidanceManager->tick();

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
    Serial.println("Invalid position. Waiting");
    motorController->setDriveSpeed(0.0);
  } else {
    Serial.println("Valid position. Moving");
    motorController->setDriveSpeed(speed);
  }

  float setpoint = guidanceManager->getUpdatedSteerBias();
  motorController->setSteerBias(setpoint);
  
  float distance_error = guidanceManager->getDistanceError();

  if(guidanceManager->getDistanceError() < waypoint_distance_threshold) {
    guidanceManager->nextWaypoint();
  }
  
  //RangeData rd = sensorManager->getRange();
  //Serial.println("Front Left: " + String(rd.front_l) + " | Front Right: " + String(rd.front_r) + " | Left: " + String(rd.left) + " | Right: " + String(rd.right));

  motorController->tick();
  delay(30);
  */

  armController->spin();
  armController->tick();
}
