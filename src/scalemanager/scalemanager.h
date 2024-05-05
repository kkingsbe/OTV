#ifndef SCALE_MANAGER_H
#define SCALE_MANAGER_H

#include <Arduino.h>
#include "HX711/HX711.h"

#define SCK 9
#define DOUT 13//A3

enum ScaleState {
    CALIBRATING,
    READY
};

struct CalibrationInfo {
    float offset;
    float scale;
    float scaleAmount;
    float tol;
};

class ScaleManager {
public:
    ScaleManager(); // Constructor
    void tick();
private:
    ScaleState state;
    HX711 cell;
    CalibrationInfo calibrationInfo;
    long count;
    float val;

    void calibrate();
    float getWeight();
};

#endif // SCALE_MANAGER_H