#include "scalemanager.h"

ScaleManager::ScaleManager():
    cell(DOUT, SCK),
    state(CALIBRATING),
    count(0),
    val(0)
{
}

void ScaleManager::tick() {
    switch(state) {
        case CALIBRATING:
            calibrate();
            state = READY;
            break;
        case READY:
            getWeight();
            break;
    }
}

void ScaleManager::calibrate() {
    Serial.println("Calibrating scale");
    int iterations = 100;
    int dt = 10; //ms
    float total_offset = 0;

    for(int i = 0; i < iterations; i++) {
        total_offset += cell.read();

        delay(dt);
    }

    calibrationInfo.offset = total_offset / iterations;

    Serial.println("Calibration offset: " + String(calibrationInfo.offset));

    calibrationInfo.scaleAmount = 15000;
    calibrationInfo.tol = 4.0;
}

float ScaleManager::getWeight() {
    count ++;

    // Use only one of these, remove after calibration
    //( val- (what scale reads with nothing on) )/ what scale reads with known weight * known weight)
    //val = ((count-1)/count) * val    +  (1/count) * cell.read(); // take long term average
    //val = 0.5 * val    +   0.5 * cell.read(); // take recent average
    long raw_val = cell.read();
    Serial.println( raw_val );
    val = -(raw_val- calibrationInfo.offset)/calibrationInfo.scaleAmount*690.5; // most recent reading
    //scl2 = mass/val;
    Serial.println( val );
    if (val <= abs(calibrationInfo.tol)) {
        //Serial.println("Raw: " + String(raw_val) + " | W Offset: " + String(raw_val - offset) + " | Processed: " + String(0,4));
        Serial.println("MASS (g): " + String(0,4));
    } 
    else {
        //Serial.println("Raw: " + String(raw_val) + " | W Offset: " + String(raw_val - offset) + " | Processed: " + String(val,4));
        Serial.println("MASS (g): " + String(val,4));
        if( val>=250){
            Serial.println( "Heavy");
        }
        else if( val>= 170 && val<= 210){
            Serial.println("Medium");
        }
        else{
            Serial.println("Light");
        }
    }

    return val;
}