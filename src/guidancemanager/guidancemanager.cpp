#include "guidancemanager.h"
#include "Arduino.h"
#include "../visionsystem/Enes100.h"

GuidanceManager::GuidanceManager() : total_waypoints(0),
                                     integral(0),
                                     active_waypoint(0),
                                     last_time(millis()),
                                     prev_err(0),
                                     pid_config(new PIDConfig{0.0, 0.0, 0.0}),
                                     vehicle_position(new VehiclePosition{0.0, 0.0, 0.0})
{
    range_data = new RangeData{0.0, 0.0, 0.0, 0.0};
}

void GuidanceManager::init()
{
    Enes100.begin("MATTerials", MATERIAL, 247, 3, 2);
    nav_mode = DIRECT_TO_WAYPOINT;
    Serial.println("Vision system online");
}

void GuidanceManager::tick()
{
    updateLocation();

    // Check for obstacles if not already avoiding one
    if (nav_mode != AVOID_OBSTACLE && nav_mode != BETWEEN_OBSTACLES)
    {
        // Obstacle in front
        if (range_data->front_l < AVOIDANCE_THRESHOLD_M || range_data->front_r < AVOIDANCE_THRESHOLD_M)
        {
            nav_mode = AVOID_OBSTACLE;
            obstacle_avoidance_mode = BACKING_AWAY;
            obstacle_side = OS_FRONT;
        }

        // Obstacle to left
        if (range_data->left < AVOIDANCE_THRESHOLD_M)
        {
            nav_mode = AVOID_OBSTACLE;
            obstacle_avoidance_mode = TURNING;
            desired_course = vehicle_position->theta + SIDE_OBSTACLE_AVOIDANCE_DIVERT_ANGLE;
            obstacle_side = OS_LEFT;
        }

        // Obstacle to right
        if (range_data->right < AVOIDANCE_THRESHOLD_M)
        {
            nav_mode = AVOID_OBSTACLE;
            obstacle_avoidance_mode = TURNING;
            desired_course = vehicle_position->theta - SIDE_OBSTACLE_AVOIDANCE_DIVERT_ANGLE;
            obstacle_side = OS_RIGHT;
        }
    }
    else if (nav_mode == AVOID_OBSTACLE)
    { // Handle obstacle avoidance resolution
        if (obstacle_avoidance_mode == BACKING_AWAY)
        { //Only possible if the obstacle is in front
            if (range_data->front_l > AVOIDANCE_THRESHOLD_M && range_data->front_r > AVOIDANCE_THRESHOLD_M)
            {
                obstacle_avoidance_mode = TURNING;
                desired_course = vehicle_position->theta + SIDE_OBSTACLE_AVOIDANCE_DIVERT_ANGLE;
            }
        }

        if (obstacle_avoidance_mode == TURNING)
        { //If the vehicle has finished turning to avoid the obstacle
            float epsilon = 0.1;
            if (vehicle_position->theta > desired_course - epsilon && vehicle_position->theta < desired_course + epsilon)
            {
                nav_mode = DIRECT_TO_WAYPOINT;
            }
        }
    }
    else if (nav_mode == BETWEEN_OBSTACLES)
    {
        // TODO: not implemented yet
    }

    String navmodestring = "";
    String oamodestring = "";

    switch (nav_mode)
    {
    case DIRECT_TO_WAYPOINT:
        navmodestring = "DIRECT_TO_WAYPOINT";
        break;
    case NAVIGATE_TO_COURSE:
        navmodestring = "NAVIGATE_TO_COURSE";
        break;
    case BETWEEN_OBSTACLES:
        navmodestring = "BETWEEN_OBSTACLES";
        break;
    case AVOID_OBSTACLE:
        navmodestring = "AVOID_OBSTACLE";
        break;
    }

    switch (obstacle_avoidance_mode)
    {
    case BACKING_AWAY:
        oamodestring = "BACKING_AWAY";
        break;
    case TURNING:
        oamodestring = "TURNING";
        break;
    }

    Enes100.println("Nav Mode: " + navmodestring + " | OA Mode: " + oamodestring + " | Obstacle Side: " + String(obstacle_side));
}

void GuidanceManager::setPidConfig(float kp, float ki, float kd)
{
    pid_config->kp = kp;
    pid_config->ki = ki;
    pid_config->kd = kd;
}

void GuidanceManager::addWaypoint(float x, float y)
{
    waypoints[total_waypoints].x = x;
    waypoints[total_waypoints].y = y;
    total_waypoints++;
}

void GuidanceManager::setActiveWaypoint(int index)
{
    if (index < 0 || index >= total_waypoints)
    {
        return;
    }

    active_waypoint = index;
}

void GuidanceManager::updateLocation()
{
    Enes100.updateLocation();
    vehicle_position->x = Enes100.getX();
    vehicle_position->y = Enes100.getY();
    vehicle_position->theta = Enes100.getTheta();

    while (vehicle_position->theta < 0)
    {
        vehicle_position->theta += 2 * 3.14159;
    }
    while (vehicle_position->theta > 2 * 3.14159)
    {
        vehicle_position->theta -= 2 * 3.14159;
    }
}

float GuidanceManager::getUpdatedSteerBias()
{
    bool v = Enes100.isVisible();

    float heading_error = getHeadingError();
    float dt = (millis() - last_time) / 1000.0;
    last_time = millis();

    if (!v)
        heading_error = prev_err;

    integral += heading_error * dt;
    float derivative = (heading_error - prev_err) / dt;
    prev_err = heading_error;

    float p_term = pid_config->kp * heading_error;
    float i_term = pid_config->ki * integral;
    float d_term = pid_config->kd * derivative;

    float setpoint = p_term + i_term + d_term;

    Enes100.println("Heading err: " + String(heading_error) + "Setpoint: " + String(setpoint));

    return setpoint;
}

void GuidanceManager::nextWaypoint()
{
    if (active_waypoint < total_waypoints - 1)
    {
        active_waypoint++;
    }
    else
    {
        active_waypoint = 0;
    }
}

float GuidanceManager::getHeadingError()
{
    float ex = waypoints[active_waypoint].x - vehicle_position->x;
    float ey = waypoints[active_waypoint].y - vehicle_position->y;

    // Obstacle avoidance guard clause
    if (nav_mode == AVOID_OBSTACLE)
    {
        // Turn to "desired_course" if turning to avoid obstacle
        if (obstacle_avoidance_mode == TURNING)
        {
            return desired_course - vehicle_position->theta;
        }

        // Maintain current heading if backing away
        if (obstacle_avoidance_mode == BACKING_AWAY)
        {
            return 0.0;
        }
    }

    float desired_heading = atan2(ey, ex);
    while (desired_heading < 0)
    {
        desired_heading += 2 * 3.14159;
    }
    while (desired_heading > 2 * 3.14159)
    {
        desired_heading -= 2 * 3.14159;
    }

    float heading_error = vehicle_position->theta - desired_heading;
    if (heading_error > 3.14159)
    {
        heading_error -= 2 * 3.14159;
    }
    else if (heading_error < -3.14159)
    {
        heading_error += 2 * 3.14159;
    }

    // Enes100.println("Heading Error: " + String(heading_error) + " Desired Heading: " + String(desired_heading) + " Vehicle Heading: " + String(vehicle_position->theta));

    return heading_error;
}

float GuidanceManager::getDistanceError()
{
    float ex = waypoints[active_waypoint].x - vehicle_position->x;
    float ey = waypoints[active_waypoint].y - vehicle_position->y;

    // Enes100.println("X Dist: " + String(ex) + " Y Dist: " + String(ey));

    return sqrt(ex * ex + ey * ey);
}

void GuidanceManager::setRangeData(RangeData *rd)
{
    range_data = rd;
}