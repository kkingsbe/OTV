#include "src/motorcontroller/motorcontroller.h"
#include "src/sensormanager/sensormanager.h"
#include "src/guidancemanager/guidancemanager.h"
#include "src/armcontroller/armcontroller.h"

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();
GuidanceManager* guidanceManager = new GuidanceManager();
ArmController* armController = new ArmController();

float speed = 0.7;
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
  guidanceManager->setPidConfig(0.8, 0.01, 0.2);

  //Potential Starts
  guidanceManager->addWaypoint(0.42, 0.54, 0);
  guidanceManager->addWaypoint(0.42, 1.49, 1);

  //Navigation waypoints
  guidanceManager->addWaypoint(1.25, 0.5, 2, true, 0, 0);
  guidanceManager->addWaypoint(1.25, 1.0, 3, true, 1, 0);
  guidanceManager->addWaypoint(1.25, 1.5, 4, true, 2, 0);

  guidanceManager->addWaypoint(2.0, 1.5, 5, true, 0, 1);
  guidanceManager->addWaypoint(2.0, 1.0, 6, true, 1, 1);
  guidanceManager->addWaypoint(2.0, 0.5, 7, true, 2, 1);

  guidanceManager->addWaypoint(3.0, 1.5, 8);  //Line up with limbo
  guidanceManager->addWaypoint(3.5, 1.5, 9);  //Pass limbo

  /*
  guidanceManager->addWaypoint(0.42, 0.3, POTENTIAL_OBSTACLE_COL1); //Line up with gap between obstacles and wall
  guidanceManager->addWaypoint(3.0, 0.3, POTENTIAL_OBSTACLE);  //Pass both columns
  guidanceManager->addWaypoint(3.0, 1.5, POTENTIAL_OBSTACLE);  //Line up with limbo
  guidanceManager->addWaypoint(3.5, 1.5, POTENTIAL_OBSTACLE);  //Pass limbo
  */

  guidanceManager->setActiveWaypoint(0);

  armController->init();

  motorController->tick();
  armController->resetPosition();
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
  if(guidanceManager->getDistanceError() > waypoint_distance_threshold) {
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

  float setpoint = guidanceManager->getUpdatedSteerBias();
  motorController->setSteerBias(setpoint);
  
  RangeData rd = sensorManager->getRange();

  float distance_error = guidanceManager->getDistanceError();
  if(guidanceManager->getDistanceError() < waypoint_distance_threshold) {
    if(guidanceManager->isActiveWaypointGrid()) {
      //Determine if obstacle exists
      if(rd.front_l < 20 || rd.front_r < 20) {
        guidanceManager->nextRow();
      } else {
        guidanceManager->nextCol(); //No obstacle, move to next column
      }
    } else {
      bool nextWaypointRes = guidanceManager->nextWaypoint();

      if (!nextWaypointRes) {
        motorController->setDriveSpeed(0.0); //Stop driving
      }
    }
  }

  guidanceManager->tick();
  Serial.println("Front Left: " + String(rd.front_l) + " | Front Right: " + String(rd.front_r) + " | Left: " + String(rd.left) + " | Right: " + String(rd.right));

  motorController->tick();
  armController->tick();

  delay(30);
}
 
