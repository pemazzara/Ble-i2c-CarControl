#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include <Arduino.h>
#include <VL53L0X.h>  // Usar la biblioteca VL53L0X de Pololu

// Pines I2C y sensores VL53L0X
#define I2C_SENSORES_SDA_PIN 45
#define I2C_SENSORES_SCL_PIN 48

#define FRONT_XSHUT_PIN  5
#define LEFT_XSHUT_PIN   6
#define RIGHT_XSHUT_PIN  7

// Direcciones I2C
#define FRONT_ADDRESS    0x30
#define LEFT_ADDRESS     0x31
#define RIGHT_ADDRESS    0x32

class SensorControl {
public:
  void begin();
  void readAll();
  
  // Variables públicas para acceder a las distancias
  int frontDistance;
  int leftDistance;
  int rightDistance;

private:
  VL53L0X sensorFront;
  VL53L0X sensorLeft;
  VL53L0X sensorRight;
  
  void initSensors();
};

#endif