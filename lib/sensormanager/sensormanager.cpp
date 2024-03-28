#include "sensormanager.h"

SensorManager::SensorManager():
    right_ultra(RIGHT_TRIG, RIGHT_ECHO),
    left_ultra(LEFT_TRIG, LEFT_ECHO),
    front_ultra(FRONT_TRIG, FRONT_ECHO)
{

}

RangeData SensorManager::getRange() {
    RangeData range;
    range.front = front_ultra.getCM();
    range.left = left_ultra.getCM();
    range.right = right_ultra.getCM();
    return range;
}