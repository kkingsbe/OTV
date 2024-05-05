#include "sensormanager.h"

SensorManager::SensorManager() {
    //scale = ScaleManager();
}

void SensorManager::init() {
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);
    pinMode(FRONT_R_TRIG, OUTPUT);
    pinMode(FRONT_R_ECHO, INPUT);
    pinMode(FRONT_L_TRIG, OUTPUT);
    pinMode(FRONT_L_ECHO, INPUT);
}

long SensorManager::takeReading(UltrasonicSensor sensor) {   
    int trigPin;
    int echoPin;
    switch(sensor) {
        case FRONT_R:
            trigPin = FRONT_R_TRIG;
            echoPin = FRONT_R_ECHO;
            break;
        case FRONT_L:
            trigPin = FRONT_L_TRIG;
            echoPin = FRONT_L_ECHO;
            break;
        case LEFT:
            trigPin = LEFT_TRIG;
            echoPin = LEFT_ECHO;
            break;
        case RIGHT:
            trigPin = RIGHT_TRIG;
            echoPin = RIGHT_ECHO;
            break;
    }

    long duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 30000);
    return duration;
}

float SensorManager::durationToCm(long duration) {
    return duration / 29.0 / 2.0;
}

RangeData SensorManager::getRange() {
    int triggerDelay = 1;

    long dur_front_l = takeReading(FRONT_L);
    delay(triggerDelay);

    long dur_front_r = takeReading(FRONT_R);
    delay(triggerDelay);

    long dur_left = takeReading(LEFT);
    delay(triggerDelay);

    long dur_right = takeReading(RIGHT);
    delay(triggerDelay);

    RangeData range;
    range.front_l = durationToCm(dur_front_l);
    range.front_r = durationToCm(dur_front_r);
    range.left = durationToCm(dur_left);
    range.right = durationToCm(dur_right);

    return range;
}

void SensorManager::tick() {
    //scale.tick();
}