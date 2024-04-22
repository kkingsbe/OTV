#include "../sensormanager/sensormanager.h"

#ifndef GUIDANCE_MANAGER_H
#define GUIDANCE_MANAGER_H

#define MAX_WAYPOINTS 20
#define AVOIDANCE_THRESHOLD_M 0.05 //The distance at which the vehicle will start to avoid the obstacle
#define SIDE_OBSTACLE_AVOIDANCE_DIVERT_ANGLE 3.14159/4 //The angle at which the vehicle will divert to avoid an obstacle to the side

struct Waypoint {
    float x;
    float y;
};

struct PIDConfig {
    float kp;
    float ki;
    float kd;
};

struct VehiclePosition {
    float x;
    float y;
    float theta;
};

enum NavigationMode {
    DIRECT_TO_WAYPOINT,  //Will navigate directly to the active waypoint
    NAVIGATE_TO_COURSE,  //Will navigate along the shortest path to the course between the previous and active waypoint
    BETWEEN_OBSTACLES,   //Will maintain the same distance on the left & right side from the obstacles
    AVOID_OBSTACLE,       //Will avoid the obstacle in front of the vehicle and return to the previously active mode once clears
};

enum ObstacleAvoidanceMode {
    BACKING_AWAY, //If backing away from the obstacle before turning
    TURNING,      //If turning to avoid the obstacle
};

enum ObstacleSide {
    OS_LEFT,
    OS_RIGHT,
    OS_FRONT
};

class GuidanceManager {
public:
    GuidanceManager(); // Constructor
    void addWaypoint(float x, float y);
    void setActiveWaypoint(int index);
    void nextWaypoint();
    float getUpdatedSteerBias();
    void setPidConfig(float kp, float ki, float kd);
    float getHeadingError();
    float getDistanceError();
    void tick();
    void init();
    void setRangeData(RangeData* rd); //Sets the range data used by the guidance manager

private:
    Waypoint waypoints[MAX_WAYPOINTS];
    int total_waypoints;
    int active_waypoint;
    float integral;
    long last_time;
    float prev_err;
    void updateLocation();
    PIDConfig* pid_config;
    VehiclePosition* vehicle_position;
    RangeData* range_data;
    NavigationMode nav_mode;
    NavigationMode previous_nav_mode; //Stores the previous nav mode (used for obsticale avoidance)
    ObstacleAvoidanceMode obstacle_avoidance_mode; //Stores the current obstacle avoidance mode
    ObstacleSide obstacle_side; //Stores the side of the vehicle which the obstacle was on
    int desired_course; //Stores the desired course angle. Used when in the obstacle avoidance nav mode
};

#endif // SENSOR_MANAGER_H