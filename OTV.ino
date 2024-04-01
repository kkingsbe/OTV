#include "src/motorcontroller/motorcontroller.h"
#include "src/sensormanager/sensormanager.h"
#include "src/visionsystem/Enes100.h"
#include <DistanceSensor.h>

MotorController* motorController = new MotorController();
SensorManager* sensorManager = new SensorManager();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motorController->init();
  motorController->setDriveSpeed(0.7);
  Enes100.begin("MATTerials", MATERIAL, 247, 3,2);
}

float target_x = 0.5;
float target_y = 0.5;

void loop() {
  Enes100.updateLocation();
  float x = Enes100.getX();
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
  while(theta < 0) {
    theta += 2*3.14159;
  }
  while(theta > 2*3.14159) {
    theta -= 2*3.14159;
  }

  bool v = Enes100.isVisible();

  float ex = target_x - x;
  float ey = target_y - y;

  float desired_heading = atan2(ey, ex);
  while(desired_heading < 0) {
    desired_heading += 2*3.14159;
  }
  while(desired_heading > 2*3.14159) {
    desired_heading -= 2*3.14159;
  }

  //Left-handed rotation for some reason
  float heading_error = theta - desired_heading;

  //Serial.println("ex: " + String(ex) + " ey: " + String(ey) + " theta: " + String(theta) + " desired_heading: " + String(desired_heading) + " heading_error: " + String(heading_error));
  Enes100.println(heading_error);

  float k = 0.02;
  motorController->setSteerBias(k*heading_error);
  motorController->tick();
  
  delay(30);
}
