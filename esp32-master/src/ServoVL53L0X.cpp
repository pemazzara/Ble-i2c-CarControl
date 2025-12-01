// ServoVL53L0X.cpp
#include "ServoVL53L0X.h"
#include <Arduino.h>
#include "SensorControl.h"

ServoVL53L0X::ServoVL53L0X(uint8_t servoPin /*, TwoWire* bus*/) 
: servoPin(servoPin){} //, i2cBus(bus) {}

void ServoVL53L0X::begin() {
    i2cBus = &Wire1;
    i2cBus->begin(I2C_TOF_SDA, I2C_TOF_SCL, 400000);
    sensor.setBus(i2cBus);
    servo.attach(servoPin);
    
    if (!sensor.init()) {
        Serial.println("❌ Error iniciando VL53L0X en servo");
        return;
    }
    sensor.setTimeout(500);
    sensor.startContinuous();
    
    setAngle(ANGLE_CENTER);
    Serial.println("✅ Servo + VL53L0X inicializado");
}

void ServoVL53L0X::setAngle(int angle) {
    servo.write(angle);
    delay(200);
}

uint16_t ServoVL53L0X::scanAngle(int angle, bool quick) {
    setAngle(angle);
    delay(quick ? 30 : 80);
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    
    if (sensor.timeoutOccurred() || distance > 2000) {
        return 2000;
    }
    return distance;
}

void ServoVL53L0X::navigationScan(uint16_t& left, uint16_t& front, uint16_t& right) {
    left = scanAngle(ANGLE_LEFT, true);
    front = scanAngle(ANGLE_CENTER, true);
    right = scanAngle(ANGLE_RIGHT, true);
    
    setAngle(ANGLE_CENTER);
    Serial.printf("🎯 NAV Scan - L:%dmm F:%dmm R:%dmm\n", left, front, right);
}

void ServoVL53L0X::detailedScan(uint16_t& left, uint16_t& frontLeft, uint16_t& front, 
                               uint16_t& frontRight, uint16_t& right) {
    left = scanAngle(ANGLE_LEFT);
    frontLeft = scanAngle(ANGLE_FRONT_LEFT);
    front = scanAngle(ANGLE_CENTER);
    frontRight = scanAngle(ANGLE_FRONT_RIGHT);
    right = scanAngle(ANGLE_RIGHT);
    
    setAngle(ANGLE_CENTER);
    Serial.printf("🗺️  DETAIL Scan - L:%d FL:%d F:%d FR:%d R:%d\n", 
                 left, frontLeft, front, frontRight, right);
}

uint16_t ServoVL53L0X::emergencyScan() {
    return scanAngle(ANGLE_CENTER, true);
}
