#include "guidancemanager.h"
#include "Arduino.h"
#include "../visionsystem/Enes100.h"

GuidanceManager::GuidanceManager():
    total_waypoints(0),
    integral(0),
    active_waypoint(0),
    last_time(millis()),
    prev_err(0),
    pid_config(new PIDConfig{0.0, 0.0, 0.0}),
    vehicle_position(new VehiclePosition{-1.0, -1.0, 0.0, false})
{
}

void GuidanceManager::init() {
    Enes100.begin("MATTerials", MATERIAL, 247, 2,1);
    Serial.println("Vision system online");
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
        Enes100.println("GuidanceManager: ERROR! Attempted to set active waypoint to " + String(index + 1) + ", but there are only " + String(total_waypoints));
        return;
    }

    Enes100.println("GuidanceManager: Setting active waypoint to " + String(index + 1));

    active_waypoint = index;
}

void GuidanceManager::updateLocation() {
    Enes100.updateLocation();
    vehicle_position->x = Enes100.getX();
    vehicle_position->y = Enes100.getY();
    vehicle_position->theta = Enes100.getTheta();

    if(!Enes100.isVisible()) vehicle_position->valid = false;
    else vehicle_position->valid = true;

    while(vehicle_position->theta < 0) {
        vehicle_position->theta += 2*3.14159;
    }
    while(vehicle_position->theta > 2*3.14159) {
        vehicle_position->theta -= 2*3.14159;
    }
}

float GuidanceManager::getUpdatedSteerBias() {
    bool v = Enes100.isVisible();

    float heading_error = getHeadingError();
    float dt = (millis() - last_time) / 1000.0;
    last_time = millis();

    if(!v) heading_error = prev_err;

    integral += heading_error * dt;
    float derivative = (heading_error - prev_err) / dt;
    prev_err = heading_error;
    
    float p_term = pid_config->kp * heading_error;
    float i_term = pid_config->ki * integral;
    float d_term = pid_config->kd * derivative;

    float setpoint = constrain(p_term + i_term + d_term, -1.0, 1.0);
    
    Enes100.println("Heading err: " + String(heading_error) + "Setpoint: " + String(setpoint));

    return setpoint;
}

void GuidanceManager::nextWaypoint() {
    if(active_waypoint < total_waypoints - 1) {
        Enes100.println("GuidanceManager: Switching to next waypoint");
        active_waypoint++;
    } else {
        Enes100.println("GuidanceManager: Reached final waypoint, resetting to first waypoint");
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

    //Enes100.println("Heading Error: " + String(heading_error) + " Desired Heading: " + String(desired_heading) + " Vehicle Heading: " + String(vehicle_position->theta));

    return heading_error;
}

float GuidanceManager::getDistanceError() {
    float ex = waypoints[active_waypoint].x - vehicle_position->x;
    float ey = waypoints[active_waypoint].y - vehicle_position->y;

    //Enes100.println("X Dist: " + String(ex) + " Y Dist: " + String(ey));

    return sqrt(ex*ex + ey*ey);
}

VehiclePosition* GuidanceManager::getPosition() {
    return vehicle_position;
}

Waypoint* GuidanceManager::getWaypoint(int index) {
    if(index >= total_waypoints) {
        return nullptr;
    }

    return &waypoints[index];
}

float GuidanceManager::getDistanceToWaypoint(int index) {
    int origActiveWaypoint = active_waypoint;
    
    if(index >= total_waypoints) {
        return -1;
    }

    //Sneakily change the active waypoint, get the distance to it, and change it back
    setActiveWaypoint(index);
    float dist = getDistanceError();
    setActiveWaypoint(origActiveWaypoint);

    return dist;
}