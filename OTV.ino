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

float target_1_x = 1.0;
float target_1_y = 0.5;
float target_2_x = 1.0;
float target_2_y = 1.5;

bool target1 = true;

float prev_err = 0.0;
float integral = 0.0;
long last_time = millis();
long target_start_time = millis();
long max_target_time = 10000;

float precise_drive_speed = 0.4;
float waypoint_distance_threshold = 0.1; //m

void loop() {
  float dt = (millis() - last_time) / 1000.0;
  last_time = millis();
  
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

  float target_x = target1 ? target_1_x : target_2_x;
  float target_y = target1 ? target_1_y : target_2_y;

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

  float kp = 0.1;
  float ki = 0.0;//0.005;
  float kd = 0.01;

  integral += heading_error * dt;
  float derivative = (heading_error - prev_err) / dt;
  float setpoint = (kp * heading_error) + (ki * integral) + (kd * derivative);
  float new_speed = 1.0 - (2.0 * abs(heading_error));
  if(new_speed < precise_drive_speed) {
    new_speed = precise_drive_speed;
  }

  //motorController->setDriveSpeed(new_speed);

  //Serial.println("ex: " + String(ex) + " ey: " + String(ey) + " theta: " + String(theta) + " desired_heading: " + String(desired_heading) + " heading_error: " + String(heading_error));
  Enes100.println("Target 1?: " + String(target1 ? "yes" : "no") + " Heading error: " + String(heading_error) + "rad. Steer bias: " + String(setpoint) + " Drive Speed: " + String(new_speed));

  motorController->setSteerBias(setpoint);
  motorController->tick();

  Serial.println("Target time: " + String(millis() - target_start_time) + "ms");

  if(sqrt(ex*ex + ey*ey) < waypoint_distance_threshold) {
    if(target1) {
      target1 = false;
      target_start_time = millis();
    } else {
      target1 = true;
      target_start_time = millis();
    }
  }

  delay(30);
}
