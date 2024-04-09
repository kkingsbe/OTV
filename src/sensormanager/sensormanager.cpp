#include "sensormanager.h"

void SensorManager::init() {
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
}

long SensorManager::takeReading(UltrasonicSensor sensor) {   
    int trigPin;
    int echoPin;
    switch(sensor) {
        case FRONT:
            trigPin = FRONT_TRIG;
            echoPin = FRONT_ECHO;
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
    duration = pulseIn(echoPin, HIGH);
    return duration;
}

float SensorManager::durationToCm(long duration) {
    return duration / 29.0 / 2.0;
}

RangeData SensorManager::getRange() {
    long dur_front = takeReading(FRONT);
    long dur_left = takeReading(LEFT);
    long dur_right = takeReading(RIGHT);

    RangeData range;
    range.front = durationToCm(dur_front);
    range.left = durationToCm(dur_left);
    range.right = durationToCm(dur_right);
    return range;
}