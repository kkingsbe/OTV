#include "guidancemanager.h"
#include "Arduino.h"
#include "../visionsystem/Enes100.h"

GuidanceManager::GuidanceManager() : total_waypoints(0),
                                     integral(0),
                                     active_waypoint(0),
                                     last_time(millis()),
                                     prev_err(0),
                                     pid_config(new PIDConfig{0.0, 0.0, 0.0}),
                                     vehicle_position(new VehiclePosition{-1.0, -1.0, 0.0, false}),
                                     pauseStartTime(millis()),
                                     isPaused(false),
                                     isHeadedUp(true),
                                     maxWaypointIndex(0)
{
}

void GuidanceManager::init()
{
    pinMode(WIFI_RX, INPUT);
    pinMode(WIFI_TX, OUTPUT);

    Enes100.begin("MATTerials", MATERIAL, 247, WIFI_TX, WIFI_RX);
    Serial.println("Vision system online");
}

GuidanceInfo GuidanceManager::tick(RangeData *rd)
{
    GuidanceInfo info;
    info.driveSpeed = 0.5;

    bool moveToNext = false;

    //Enes100.println("Distance to obstacle: " + String(rd->front_r));

    // If at waypoint
    if (getDistanceError() < WAYPOINT_DISTANCE_THRESHOLD)
    {
        Enes100.println("Reached waypoint");
        // If the waypoint has an associated target heading
        if (getWaypoint(active_waypoint)->isGrid)
        {
            float targetHeading = isHeadedUp ? PI / 2.0 : 3 * PI / 2.0;
            float diff = targetHeading - vehicle_position->theta;
            float normalized_diff = fmod(diff + PI, 2 * PI) - PI;
            float error = abs(normalized_diff);
            if (normalized_diff > 0)
            {
                error *= -1;
            }

            Enes100.println("Waypoint has target heading. Turning.");
            Enes100.println("Heading: " + String(targetHeading) + " Vehicle Heading: " + String(vehicle_position->theta) + " Diff: " + String(error));
            if (abs(error) > WAYPOINT_HEADING_THRESHOLD)
            {
                info.steerBias = error < 0 ? -1.0 : 1.0; // Might need to be flipped lol
                // info.driveSpeed = 0.5;
                info.driveSpeed = 0.35;
            }
            else //Pointing in correct direction
            {
                if(!isPaused) {
                    isPaused = true;
                    pauseStartTime = millis();
                }

                if (isPaused && millis() - pauseStartTime > PAUSE_TIME)
                {
                    isPaused = false;
                    // Determine if obstacle exists
                    if (isActiveWaypointGrid())
                    {
                        if (isHeadedUp && rd->right < OBSTACLE_DISTANCE_THRESHOLD || !isHeadedUp && rd->left < OBSTACLE_DISTANCE_THRESHOLD)
                        {
                            Enes100.println("Obstacle detected. Moving to next row");
                            bool res = nextRow();
                            if(!res) {
                                isHeadedUp = !isHeadedUp;
                                nextRow();
                            }
                        }
                        else
                        {
                            Enes100.println("Distance to obstacle: " + String(isHeadedUp ? rd->right : rd->left));
                            Enes100.println("No obstacle detected. Moving to next column");
                            bool res = nextCol(); // No obstacle, move to next column
                            if (!res)
                            {
                                Enes100.println("No more columns. All obstacles passed. Searching for next waypoint");
                                clearColumn();
                            }
                        }
                        Enes100.println("Active waypoint: " + String(active_waypoint));
                    }
                    else
                    {
                        moveToNext = true;
                    }
                }
                else
                {
                    Enes100.println("Pausing for " + String(PAUSE_TIME) + "ms");
                }
            }
        }
        else
        { // No target heading for waypoint
            moveToNext = true;
        }
    }
    else
    {
        info.steerBias = getUpdatedSteerBias();
    }

    if (moveToNext)
    {
        bool nextWaypointRes = nextWaypoint();

        Enes100.println("Proceeding to next waypoint (" + String(active_waypoint) + "/" + String(maxWaypointIndex) + ")");

        if (!nextWaypointRes)
        {
            // Stop driving
            info.driveSpeed = 0.0;
        }
    }

    updateLocation();

    // Enes100.println("Steer Bias: " + String(info.steerBias));

    if(isPaused) {
        info.driveSpeed = 0.0;
        info.steerBias = 0.0;
    }

    return info;
}

void GuidanceManager::setPidConfig(float kp, float ki, float kd)
{
    pid_config->kp = kp;
    pid_config->ki = ki;
    pid_config->kd = kd;
}

void GuidanceManager::addWaypoint(float x, float y, int index, bool useHeading = false, float heading = 0.0, bool isGrid = false, int row = 0, int col = 0)
{
    waypoints[total_waypoints].x = x;
    waypoints[total_waypoints].y = y;
    waypoints[total_waypoints].isGrid = isGrid;
    waypoints[total_waypoints].grid.row = row;
    waypoints[total_waypoints].grid.col = col;
    waypoints[total_waypoints].index = index;
    waypoints[total_waypoints].heading = heading;
    waypoints[total_waypoints].hasHeading = useHeading;

    total_waypoints++;
    maxWaypointIndex = max(maxWaypointIndex, index);
}

void GuidanceManager::setActiveWaypoint(int index)
{
    if (index < 0 || index >= maxWaypointIndex)
    {
        Enes100.println("GuidanceManager: ERROR! Attempted to set active waypoint to " + String(index + 1) + ", but there are only " + String(total_waypoints));
        return;
    }

    Enes100.println("GuidanceManager: Setting active waypoint to " + String(index + 1));

    active_waypoint = index;
}

void GuidanceManager::updateLocation()
{
    float aruco_offset_y = 0.20;
    float aruco_offset_x = 0.06;

    Enes100.updateLocation();
    vehicle_position->x = Enes100.getX();
    vehicle_position->y = Enes100.getY();
    vehicle_position->theta = Enes100.getTheta();

    // Account for vehicle offset
    vehicle_position->x += aruco_offset_y * cos(vehicle_position->theta);
    vehicle_position->y += aruco_offset_y * sin(vehicle_position->theta);

    //vehicle_position->x += aruco_offset_x * cos(vehicle_position->theta + PI / 2.0);
    //vehicle_position->y += aruco_offset_x * sin(vehicle_position->theta + PI / 2.0);

    // if(!Enes100.isVisible()) vehicle_position->valid = false;
    // else vehicle_position->valid = true;
    vehicle_position->valid = true;

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

    // Zero point turn if more than 45 degrees off
    float setpoint = abs(heading_error) > (3.14159 / 4) ? (abs(heading_error) / heading_error) : constrain(p_term + i_term + d_term, -1.0, 1.0);

    // Enes100.println("Heading err: " + String(heading_error) + "Setpoint: " + String(setpoint));

    return setpoint;
}

bool GuidanceManager::nextWaypoint()
{
    if (active_waypoint < maxWaypointIndex)
    {
        Enes100.println("GuidanceManager: Switching to next waypoint (" + String(active_waypoint + 2) + "/" + String(total_waypoints) + ")");
        active_waypoint++;
    }
    else
    {
        Enes100.println("GuidanceManager: Reached final waypoint");
    }
}

float GuidanceManager::getHeadingError()
{
    float ex = getWaypoint(active_waypoint)->x - vehicle_position->x;
    float ey = getWaypoint(active_waypoint)->y - vehicle_position->y;

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
    float ex = getWaypoint(active_waypoint)->x - vehicle_position->x;
    float ey = getWaypoint(active_waypoint)->y - vehicle_position->y;

    // Enes100.println("X Dist: " + String(ex) + " Y Dist: " + String(ey));

    return sqrt(ex * ex + ey * ey);
}

VehiclePosition *GuidanceManager::getPosition()
{
    return vehicle_position;
}

Waypoint *GuidanceManager::getWaypoint(int index)
{
    if (index >= maxWaypointIndex)
    {
        return nullptr;
    }
    else
    {
        for (int i = 0; i < maxWaypointIndex; i++)
        {
            if (waypoints[i].index == index)
            {
                return &waypoints[i];
            }
        }
    }

    return getWaypoint(index);
}

float GuidanceManager::getDistanceToWaypoint(int index)
{
    int origActiveWaypoint = active_waypoint;

    if (index >= total_waypoints)
    {
        return -1;
    }

    // Sneakily change the active waypoint, get the distance to it, and change it back
    setActiveWaypoint(index);
    float dist = getDistanceError();
    setActiveWaypoint(origActiveWaypoint);

    return dist;
}

bool GuidanceManager::nextRow()
{
    int currentRow = getWaypoint(active_waypoint)->grid.row;
    int currentCol = getWaypoint(active_waypoint)->grid.col;

    for (int i = 0; i < total_waypoints; i++)
    {
        if (waypoints[i].grid.row == currentRow + (isHeadedUp ? 1 : -1) && waypoints[i].grid.col == currentCol)
        {
            active_waypoint = waypoints[i].index;
            return true;
        }
    }

    Enes100.println("ERROR: No waypoint for the next row was found");
    return false;
}

bool GuidanceManager::nextCol()
{
    int currentRow = getWaypoint(active_waypoint)->grid.row;
    int currentCol = getWaypoint(active_waypoint)->grid.col;

    for (int i = 0; i < total_waypoints; i++)
    {
        if (waypoints[i].grid.row == currentRow && waypoints[i].grid.col == currentCol + 1)
        {
            active_waypoint = waypoints[i].index;
            return true;
        }
    }

    return false;
}

bool GuidanceManager::isActiveWaypointGrid()
{
    return getWaypoint(active_waypoint)->isGrid;
}

float GuidanceManager::normalizeAngle(float angle)
{
    while (angle > PI)
        angle -= 2 * PI;
    while (angle < -PI)
        angle += 2 * PI;
    return angle;
}

void GuidanceManager::clearColumn() {
    int lastGridIndex = 0;
    for (int i = active_waypoint; i <= maxWaypointIndex; i++) {
        Enes100.println("Checking waypoint " + String(i) + " for grid");
        Enes100.println("Is grid? " + String(getWaypoint(i)->isGrid));
        if (!getWaypoint(i)->isGrid)
        {
            Enes100.println("Valid");
            lastGridIndex = i - 1;
            break;
        } else {
            Enes100.println("Not valid");
        }
    }

    int firstRowClearIndex = lastGridIndex + 1;
    int secondRowClearIndex = lastGridIndex + 2;
    int thirdRowClearIndex = lastGridIndex + 3;

    switch(getWaypoint(active_waypoint)->grid.row) {
        case 0:
            setActiveWaypoint(firstRowClearIndex);
            break;
        case 1:
            setActiveWaypoint(secondRowClearIndex);
            break;
        case 2:
            setActiveWaypoint(thirdRowClearIndex);
            break;
    }
}