# Ble-i2c-CarControl
# 🚗 CarRobot - Sistema de Control Autónomo

Sistema robótico con control BLE, navegación autónoma y evasión de obstáculos.

## ✨ Características

- 🤖 **Navegación autónoma** con 3 sensores VL53L0X
- 📱 **Control remoto** vía BLE desde celular
- 🛡️ **Sistema de prioridades** (Emergencia > Manual > Auto > Seguridad)
- 🎯 **Arquitectura distribuida**
- 1) ESP32 (cerebro) + Arduino (ejecutor)
  2) ESP32 Master SPI (cerebro)
  3) + ESP32 Slave SPI (ejecutor)
- 🔊 **Comunicación I2C o SPI** entre microcontroladores

## 🎮 Comandos BLE
1) Caracteres simples:`F` - Adelante, `B` - Atrás etc.
2) Comandos con velocidad y ángulo.
   
## 🏗️ Arquitectura

