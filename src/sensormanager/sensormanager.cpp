#include "sensormanager.h"

SensorManager::SensorManager() {

}

void SensorManager::init() {
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);
    pinMode(FRONT_R_TRIG, OUTPUT);
    pinMode(FRONT_R_ECHO, INPUT);
}

long SensorManager::takeReading(UltrasonicSensor sensor) {   
    int trigPin;
    int echoPin;
    switch(sensor) {
        case FRONT:
            trigPin = FRONT_R_TRIG;
            echoPin = FRONT_R_ECHO;
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
    duration = pulseIn(echoPin, HIGH/*, 30000*/);
    Serial.println("Duration: " + String(duration) + " microseconds");
    return duration;
}

float SensorManager::durationToCm(long duration) {
    return duration / 29.0 / 2.0;
}

RangeData SensorManager::getRange() {
    Serial.println("Taking readings:");
    Serial.println("Front...");
    long dur_front = takeReading(FRONT);
    Serial.println("Left...");
    long dur_left = takeReading(LEFT);
    Serial.println("Right...");
    long dur_right = takeReading(RIGHT);
    Serial.println("Done taking readings. Converting to cm");

    RangeData range;
    range.front = durationToCm(dur_front);
    range.left = durationToCm(dur_left);
    range.right = durationToCm(dur_right);

    Serial.println("Converted to cm. Returning...");

    return range;
}