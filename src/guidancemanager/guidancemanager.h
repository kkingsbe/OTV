#ifndef GUIDANCE_MANAGER_H
#define GUIDANCE_MANAGER_H

#include "../sensormanager/sensormanager.h"

#define MAX_WAYPOINTS 20
#define WIFI_RX 3
#define WIFI_TX 2
#define WAYPOINT_DISTANCE_THRESHOLD 0.25 //m
#define WAYPOINT_HEADING_THRESHOLD 0.5 //rad

struct WaypointGrid {
    int row;
    int col;
};

struct Waypoint {
    float x;
    float y;
    bool isGrid;
    WaypointGrid grid;
    int index;
    float heading;
    bool hasHeading;
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
    bool valid;
};

struct GuidanceInfo {
    float distanceError;
    float headingError;
    float steerBias;
    float driveSpeed;
};

class GuidanceManager {
public:
    GuidanceManager(); // Constructor
    void addWaypoint(float x, float y, int index, bool useHeading = false, float heading = 0.0, bool isGrid = false, int row = 0, int col = 0);
    void setActiveWaypoint(int index);
    bool nextWaypoint(); //True if there is a next waypoint, false if there is not
    void setPidConfig(float kp, float ki, float kd);
    float getHeadingError();
    float getDistanceError();
    GuidanceInfo tick(RangeData* rd);
    void init();
    float getDistanceToWaypoint(int index);
    VehiclePosition* getPosition();
    Waypoint* getWaypoint(int index);
    bool isActiveWaypointGrid();
    void nextRow();
    void nextCol();
private:
    Waypoint waypoints[MAX_WAYPOINTS];
    int total_waypoints;
    int active_waypoint;
    float integral;
    long last_time;
    float prev_err;
    void updateLocation();
    float getUpdatedSteerBias();
    float normalizeAngle(float angle);
    PIDConfig* pid_config;
    VehiclePosition* vehicle_position;
};

#endif // SENSOR_MANAGER_H