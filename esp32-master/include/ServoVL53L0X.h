// ESP32 - ServoVL53L0X.h
#include <VL53L0X.h>  // Usar la biblioteca VL53L0X de Pololu
#include <ESP32Servo.h> // Biblioteca Servo para ESP32

#ifndef SERVO_VL53L0X_H
#define SERVO_VL53L0X_H

#include <Wire.h>
#include <VL53L0X.h>
//#include <Servo.h>
#include <ESP32Servo.h> // Biblioteca Servo para ESP32

class ServoVL53L0X {
private:
    VL53L0X sensor;
    Servo servo;
    uint8_t servoPin;
    TwoWire* i2cBus;
    // Optimized angles
    const int ANGLE_LEFT = 180;    // ✅ 180° = Izquierda
    const int ANGLE_CENTER = 90;   // ✅ 90° = Frente
    const int ANGLE_RIGHT = 0;     // ✅ 0° = Derecha
    const int ANGLE_FRONT_LEFT = 135;  // ✅ Diagonal frontal-izquierda
    const int ANGLE_FRONT_RIGHT = 45;  // ✅ Diagonal frontal-derecha
    
public:
    ServoVL53L0X(uint8_t servoPin /*, TwoWire* bus = &Wire1*/);  
    void begin();
    void setAngle(int angle);
    uint16_t scanAngle(int angle, bool quick = false);
    void navigationScan(uint16_t& left, uint16_t& front, uint16_t& right);
    void detailedScan(uint16_t& left, uint16_t& frontLeft, uint16_t& front, 
                     uint16_t& frontRight, uint16_t& right);
    uint16_t emergencyScan();
};

#endif
/*
class ServoVL53L0X {
private:
    VL53L0X sensor;
    Servo servo;
    uint8_t servoPin;
    
    // Ángulos optimizados
    const int ANGLE_LEFT = 160;    // No extremo para mejor respuesta
    const int ANGLE_CENTER = 90;
    const int ANGLE_RIGHT = 20;
    const int ANGLE_FRONT_LEFT = 120;
    const int ANGLE_FRONT_RIGHT = 60;
    
public:
    ServoVL53L0X(uint8_t servoPin) : servoPin(servoPin) {}
    
    void begin() {
        Wire.begin();
        void setAngle(int angle);
        uint16_t scanAngle(int angle, bool quick = false);
        void navigationScan(uint16_t& left, uint16_t& front, uint16_t& right);
        void detailedScan(uint16_t& left, uint16_t& frontLeft, uint16_t& front, 
                     uint16_t& frontRight, uint16_t& right);
        uint16_t emergencyScan();
        //servo.attach(servoPin); // Se pasò a ServoVL530X.cpp begin()
        
        if (!sensor.init()) {
            Serial.println("❌ Error iniciando VL53L0X en servo");
            return;
        }
        sensor.setTimeout(500);
        sensor.startContinuous();
        
        setAngle(ANGLE_CENTER);
        Serial.println("✅ Servo + VL53L0X inicializado");
    }
    
    void setAngle(int angle) {
        servo.write(angle);
        delay(200); // Optimizado para velocidad/estabilidad
    }
    
    uint16_t scanAngle(int angle, bool quick = false) {
        setAngle(angle);
        delay(quick ? 30 : 80); // Modo rápido para emergencias
        uint16_t distance = sensor.readRangeContinuousMillimeters();
        
        // Filtrado básico
        if (sensor.timeoutOccurred() || distance > 2000) {
            return 2000;
        }
        return distance;
    }
    
    // Escaneo rápido para navegación (3 puntos)
    void navigationScan(uint16_t& left, uint16_t& front, uint16_t& right) {
        left = scanAngle(ANGLE_LEFT, true);
        front = scanAngle(ANGLE_CENTER, true);
        right = scanAngle(ANGLE_RIGHT, true);
        
        setAngle(ANGLE_CENTER); // Volver a posición útil
        
        Serial.printf("🎯 NAV Scan - L:%dmm F:%dmm R:%dmm\n", left, front, right);
    }
    
    // Escaneo detallado para mapeo (5 puntos)
    void detailedScan(uint16_t& left, uint16_t& frontLeft, uint16_t& front, 
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
    
    // Escaneo ultra-rápido de emergencia (solo frontal)
    uint16_t emergencyScan() {
        return scanAngle(ANGLE_CENTER, true);
    }
};*/