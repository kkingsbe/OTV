#include "sensormanager.h"

SensorManager::SensorManager():
    right_ultra(RIGHT_TRIG, RIGHT_ECHO),
    left_ultra(LEFT_TRIG, LEFT_ECHO),
    front_ultra(FRONT_TRIG, FRONT_ECHO)
{

}

RangeData SensorManager::getRange() {
    RangeData range;
    Serial.println("Front...");
    range.front = front_ultra.getCM();
    Serial.println("Left...");
    range.left = left_ultra.getCM();
    Serial.println("Right...");
    range.right = right_ultra.getCM();
    return range;
}
