// SensorVL53L0X.h
#ifndef SENSOR_VL53L0X_H
#define SENSOR_VL53L0X_H

#include <Wire.h>
#include <VL53L0X.h> // Pololu

// Pines I2C y sensores VL53L0X
#define I2C_TOF_SDA 8
#define I2C_TOF_SCL 9

// Dirección por defecto (todos los sensores usan la misma)
#define DEFAULT_ADDRESS  0x29
#define I2C_PORT I2C_NUM_0

class SensorVL53L0X {
private:
    VL53L0X sensor;
    //TwoWire* i2cBus;
public: // <--- ¡Asegúrate de agregar esto!
    void begin(); // Ya existe
    
    // **NUEVO MÉTODO PÚBLICO**
    uint16_t leerDistancia(); 
    bool timeout;
    
    // ... otros métodos públicos si los hay
};

#endif