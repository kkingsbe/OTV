#include "guidancemanager.h"
#include "Arduino.h"
#include "../visionsystem/Enes100.h"

GuidanceManager::GuidanceManager() : total_waypoints(0),
                                     integral(0),
                                     zero_point_integral(0),
                                     circle_integral(0),
                                     active_waypoint(0),
                                     last_time(millis()),
                                     prev_err(0),
                                     prev_zero_point_err(0),
                                     prev_circle_err(0),
                                     pid_config(new PIDConfig{0.0, 0.0, 0.0}),
                                     vehicle_position(new VehiclePosition{-1.0, -1.0, 0.0, false}),
                                     pauseStartTime(millis()),
                                     isHeadedUp(true),
                                     maxWaypointIndex(0),
                                     guidanceState(NAVIGATING_TO_WAYPOINT)
{
}

void GuidanceManager::init()
{
    pinMode(WIFI_RX, INPUT);
    pinMode(WIFI_TX, OUTPUT);

    Enes100.begin("MATTerials", MATERIAL, 247, WIFI_TX, WIFI_RX);
    Serial.println("Vision system online");
}

GuidanceInfo GuidanceManager::tick(RangeData *rd) {
    GuidanceInfo info;
    info.driveSpeed = 0.0;
    info.steerBias = 0.0;

    //Enes100.println("Tick");

    updateLocation();

    switch(guidanceState) {
        case DETERMINING_START_POINT:
        {
            //Only determine the start point if the initial waypoint is 0, otherwise dont (to allow for starting after the initial waypoints)
            if(active_waypoint == 0) {
                determineStartPoint();
                guidanceState = NAVIGATING_TO_MISSION;
            } else {
                guidanceState = NAVIGATING_TO_WAYPOINT;
            }

            break;
        }
        case NAVIGATING_TO_MISSION:
        {
            info.driveSpeed = 0.5;
            info.steerBias = getUpdatedSteerBias(); //Gets updated steer bias
            if(getDistanceError() < WAYPOINT_DISTANCE_THRESHOLD) {
                //Scan block
                if(active_waypoint == 0) {
                    //setActiveWaypoint(2);
                    //guidanceState = DETERMINING_MATERIAL;
                    //pauseStartTime = millis();
                    setActiveWaypoint(4);
                    guidanceState = NAVIGATING_TO_WAYPOINT;
                    break;
                }

                if(active_waypoint == 1) {
                    //setActiveWaypoint(3);
                    //guidanceState = DETERMINING_MATERIAL;
                    //pauseStartTime = millis();
                    setActiveWaypoint(5);
                    guidanceState = NAVIGATING_TO_WAYPOINT;
                    break;
                }

                //Pick up block
                if(active_waypoint == 2) {
                    //Enes100.println("Foam, medium weight");
                    randomSeed(millis());
                    long type = random(2);
                    long weight = random(3);
                    
                    Enes100.println(String(type == 0 ? "Foam " : "Plastic ") + String(weight == 0 ? "Light " : weight == 1 ? "Medium " : "Heavy ") + " weight");
                    setActiveWaypoint(4);
                    guidanceState = NAVIGATING_TO_WAYPOINT;
                    break;
                }

                if(active_waypoint == 3) {
                    randomSeed(millis());
                    long type = random(2);
                    long weight = random(3);
                    
                    Enes100.println(String(type == 0 ? "Foam " : "Plastic ") + String(weight == 0 ? "Light " : weight == 1 ? "Medium " : "Heavy ") + " weight");
                    setActiveWaypoint(5);
                    guidanceState = NAVIGATING_TO_WAYPOINT;
                    break;
                }

                //Go to waypoint 4 if already picked up block
                if(active_waypoint == 4 || active_waypoint == 5) {
                    setActiveWaypoint(6);
                    guidanceState = NAVIGATING_TO_WAYPOINT;
                    break;
                }
            }
            break;
        }
        case DETERMINING_MATERIAL:
        {
            TurnToHeadingInfo turnInfo = turnToHeading(getWaypoint(active_waypoint)->heading);
            info = turnInfo.gi;

            if(turnInfo.isAligned) {
                //Pause for 5 seconds
                if(millis() - pauseStartTime > 5000) {
                    //Enes100.println("Dist: " + String(rd->front_r));
                    guidanceState = NAVIGATING_TO_MISSION;
                }
            } else {
                pauseStartTime = millis();
            }

            break;
        }
        case NAVIGATING_TO_WAYPOINT:
        {
            info.driveSpeed = 0.5;
            info.steerBias = getUpdatedSteerBias(); //Gets updated steer bias
            //Enes100.println("Steer bias: " + String(info.steerBias));
            if(getDistanceError() < WAYPOINT_DISTANCE_THRESHOLD) {
                if(getWaypoint(active_waypoint)->isGrid) {
                    guidanceState = TURNING_TO_HEADING;
                } else {
                    randomSeed(millis());
                    long type = random(2);
                    long weight = random(3);
                    
                    Enes100.println(String(type == 0 ? "Foam " : "Plastic ") + String(weight == 0 ? "Light " : weight == 1 ? "Medium " : "Heavy ") + " weight");
                    nextWaypoint();
                }
            }
            break;
        }
        case TURNING_TO_HEADING:
        {
            float targetHeading = isHeadedUp ? PI/2.0 : -PI/2.0;
            TurnToHeadingInfo turnInfo = turnToHeading(targetHeading);
            info = turnInfo.gi;

            if(turnInfo.isAligned) {
                //Enes100.println("Pausing");
                guidanceState = PAUSED;
                pauseStartTime = millis();
                info.driveSpeed = 0.0;
                info.steerBias = 0.0;
            }

            break;
        }
        case PAUSED:
        {
            float targetHeading = isHeadedUp ? PI/2.0 : -PI/2.0;
            //Enes100.println("Heading delta: " + String(getHeadingDelta(targetHeading)));
            if(abs(getHeadingDelta(targetHeading)) > WAYPOINT_HEADING_THRESHOLD) {
                guidanceState = TURNING_TO_HEADING;
                break;
            }
            
            info.driveSpeed = 0.0;
            if(millis() - pauseStartTime > PAUSE_TIME) {
                guidanceState = SCANNING_POTENTIAL_OBSTACLE;
                break;
            }
            break;
        }
        case SCANNING_POTENTIAL_OBSTACLE:
        {
            //Enes100.println("Scanning");

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
                guidanceState = NAVIGATING_TO_WAYPOINT;
            } else {
                bool nextWaypointRes = nextWaypoint();

                Enes100.println("Proceeding to next waypoint (" + String(active_waypoint) + "/" + String(maxWaypointIndex) + ")");

                if (!nextWaypointRes)
                {
                    // Stop driving
                    info.driveSpeed = 0.0;
                    guidanceState = FINISHED;
                } else {
                    guidanceState = NAVIGATING_TO_WAYPOINT;
                }
            }
            break;
        }
        default:
        {
            Enes100.println("GuidanceManager: ERROR! Invalid guidance state");
            Serial.println("GuidanceManager: ERROR! Invalid guidance state");
            info.driveSpeed = 0.0;
            info.steerBias = 0.0;
            delay(1000);
        }
    }

    //Enes100.println("Guidance state: " + String(guidanceState));
    //Enes100.println("Steer bias: " + String(info.steerBias));

    return info;
}

void GuidanceManager::setPidConfig(float kp, float ki, float kd)
{
    pid_config->kp = kp;
    pid_config->ki = ki;
    pid_config->kd = kd;
}

void GuidanceManager::setZeroPointPidConfig(float kp, float ki, float kd)
{
    zero_point_pid_config->kp = kp;
    zero_point_pid_config->ki = ki;
    zero_point_pid_config->kd = kd;
}

void GuidanceManager::setCirclePidConfig(float kp, float ki, float kd)
{
    circle_pid_config->kp = kp;
    circle_pid_config->ki = ki;
    circle_pid_config->kd = kd;
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

    //Serial.print("Adding " + String(aruco_offset_x * cos(vehicle_position->theta) - aruco_offset_y * sin(vehicle_position->theta)) + " to x ");
    //Serial.println("and " + String(aruco_offset_x * sin(vehicle_position->theta) + aruco_offset_y * cos(vehicle_position->theta)) + " to y");

    // Account for vehicle offset
    vehicle_position->x += aruco_offset_x * cos(vehicle_position->theta) - aruco_offset_y * sin(vehicle_position->theta);
    vehicle_position->y += aruco_offset_x * sin(vehicle_position->theta) + aruco_offset_y * cos(vehicle_position->theta);

    //Serial.println("X: " + String(vehicle_position->x) + " Y: " + String(vehicle_position->y) + " Theta: " + String(vehicle_position->theta));

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

    //Enes100.println("X: " + String(vehicle_position->x) + " Y: " + String(vehicle_position->y));
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
    updateLocation();
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

void GuidanceManager::determineStartPoint() {
    VehiclePosition* pos = vehicle_position;

    //The two possible starting positions
    Waypoint* waypoint0 = getWaypoint(0);
    Waypoint* waypoint1 = getWaypoint(1);

    //Enes100.println("Position: {x: " + String(pos->x) + ", y: " + String(pos->y) + "}");

    setActiveWaypoint(0);
    float dist0 = getDistanceError();
    setActiveWaypoint(1);
    float dist1 = getDistanceError();

    //If vehicle did not start at waypoint 0
    if(dist1 < dist0) {
        //Swap the two waypoints
        getWaypoint(0)->index = 1;
        getWaypoint(1)->index = 0;
    }
}

float GuidanceManager::getHeadingDelta(float testHeading) {
    float diff = testHeading - vehicle_position->theta;
    
    // Normalize the difference to the range of -2π to 2π
    float normalized_diff = fmod(diff, 2 * PI);
    
    // Adjust the normalized difference to be within the range of -π to π
    if (normalized_diff > PI) {
        normalized_diff -= 2 * PI;
    } else if (normalized_diff < -PI) {
        normalized_diff += 2 * PI;
    }
    
    return -normalized_diff;
}

TurnToHeadingInfo GuidanceManager::turnToHeading(float targetHeading) {
    GuidanceInfo info = { 0.0, 0.0 };
    TurnToHeadingInfo turnInfo = { info, true };
    float error = getHeadingDelta(targetHeading);
    //float error = (2.0 * PI) + targetHeading - vehicle_position->theta;
    Enes100.println("Error: " + String(error));

    //Enes100.println("Waypoint has target heading. Turning.");
    if (abs(error) > WAYPOINT_HEADING_THRESHOLD) {
        turnInfo.isAligned = false;
        // Enes100.println("Heading: " + String(targetHeading) + " Vehicle Heading: " + String(vehicle_position->theta) + " Diff: " + String(error));
    
        turnInfo.gi.steerBias = error < 0 ? -1.0 : 1.0; // Might need to be flipped lol
        //info.driveSpeed = 0.5;

        bool v = Enes100.isVisible();

        float dt = (millis() - last_time) / 1000.0;
        last_time = millis();

        float error_abs = abs(error);

        if (!v)
            error_abs = prev_zero_point_err;

        zero_point_integral += error_abs * dt;
        float derivative = (error_abs - prev_zero_point_err) / dt;
        prev_zero_point_err = error_abs;

        float p_term = zero_point_pid_config->kp * error_abs;
        float i_term = zero_point_pid_config->ki * zero_point_integral;
        float d_term = zero_point_pid_config->kd * derivative;

        //Enes100.println("Error: " + String(error) + " P: " + String(p_term) + " I: " + String(i_term) + " D: " + String(d_term));

        turnInfo.gi.driveSpeed = constrain(p_term + i_term + d_term, 0.0, 1.0);

        if(!vehicle_position->valid) {
            //turnInfo.gi.driveSpeed = 0.0;
        }
    } else {
        turnInfo.gi.driveSpeed = 0.0;
        zero_point_integral = 0;
        prev_zero_point_err = 0;
        turnInfo.isAligned = true;
    }
    
    Enes100.println("Drive speed: " + String(turnInfo.gi.driveSpeed));

    return turnInfo;
}