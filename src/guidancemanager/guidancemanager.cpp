#include "guidancemanager.h"
#include "Arduino.h"
#include "../visionsystem/Enes100.h"

GuidanceManager::GuidanceManager():
    total_waypoints(0),
    integral(0),
    last_time(millis()),
    prev_err(0),
    pid_config(new PIDConfig{0.0, 0.0, 0.0}),
    vehicle_position(new VehiclePosition{0.0, 0.0, 0.0})
{
    Enes100.begin("MATTerials", MATERIAL, 247, 3,2);
}

void GuidanceManager::tick() {
    updateLocation();
}

void GuidanceManager::setPidConfig(float kp, float ki, float kd) {
    pid_config->kp = kp;
    pid_config->ki = ki;
    pid_config->kd = kd;
}

void GuidanceManager::addWaypoint(float x, float y) {
    waypoints[total_waypoints].x = x;
    waypoints[total_waypoints].y = y;
    total_waypoints++;
}

void GuidanceManager::setActiveWaypoint(int index) {
    if(index < 0 || index >= total_waypoints) {
        return;
    }

    active_waypoint = index;
}

void GuidanceManager::updateLocation() {
    Enes100.updateLocation();
    vehicle_position->x = Enes100.getX();
    vehicle_position->y = Enes100.getY();
    vehicle_position->theta = Enes100.getTheta();

    while(vehicle_position->theta < 0) {
        vehicle_position->theta += 2*3.14159;
    }
    while(vehicle_position->theta > 2*3.14159) {
        vehicle_position->theta -= 2*3.14159;
    }
}

float GuidanceManager::getUpdatedSteerBias() {
    float heading_error = getHeadingError();
    long dt = millis() - last_time;
    last_time = millis();

    integral += heading_error * dt;
    float derivative = (heading_error - prev_err) / dt;
    float setpoint = (pid_config->kd * heading_error) + (pid_config->ki * integral) + (pid_config->kd * derivative);
}

void GuidanceManager::nextWaypoint() {
    if(active_waypoint < total_waypoints - 1) {
        active_waypoint++;
    } else {
        active_waypoint = 0;
    }
}

float GuidanceManager::getHeadingError() {
    float ex = waypoints[active_waypoint].x - vehicle_position->x;
    float ey = waypoints[active_waypoint].y - vehicle_position->y;

    float desired_heading = atan2(ey, ex);
    while(desired_heading < 0) {
        desired_heading += 2*3.14159;
    }
    while(desired_heading > 2*3.14159) {
        desired_heading -= 2*3.14159;
    }

    float heading_error = vehicle_position->theta - desired_heading;
    if (heading_error > 3.14159) {
      heading_error -= 2 * 3.14159;
    } else if (heading_error < -3.14159) {
        heading_error += 2 * 3.14159;
    }

    return heading_error;
}

float GuidanceManager::getDistanceError() {
    float ex = waypoints[active_waypoint].x - vehicle_position->x;
    float ey = waypoints[active_waypoint].y - vehicle_position->y;

    return sqrt(ex*ex + ey*ey);
}