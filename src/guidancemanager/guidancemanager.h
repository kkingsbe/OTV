#ifndef GUIDANCE_MANAGER_H
#define GUIDANCE_MANAGER_H

#define MAX_WAYPOINTS 20

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
    bool valid;
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
    VehiclePosition* getPosition();
    Waypoint* getWaypoint(int index);

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
};

#endif // SENSOR_MANAGER_H