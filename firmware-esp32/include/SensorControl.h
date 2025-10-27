#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include <Arduino.h>
#include <VL53L0X.h>  // Usar la biblioteca VL53L0X de Pololu

// Pines XSHUT
#define FRONT_XSHUT_PIN  5
#define LEFT_XSHUT_PIN   6
#define RIGHT_XSHUT_PIN  7

// Pines I2C y sensores VL53L0X
#define I2C_TOF_SDA 8
#define I2C_TOF_SCL 9

// Dirección por defecto (todos los sensores usan la misma)
#define DEFAULT_ADDRESS  0x29
#define I2C_PORT I2C_NUM_0


class SensorControl {
public:
  void begin();
  void readAll();
  void readSensor(uint8_t sensorIndex); // Nueva: leer sensor individual
  void printDistances();
  void diagnoseSensors();
  
  uint16_t frontDistance;
  uint16_t leftDistance; 
  uint16_t rightDistance;

private:
  void initSensors();
  void enableSensor(uint8_t sensorIndex);
  void disableAllSensors();
  
  VL53L0X sensor; // ¡Solo un objeto sensor!
  
  TwoWire* i2cBus;
};
#endif