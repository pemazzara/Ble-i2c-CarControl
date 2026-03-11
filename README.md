# CarRobot
# 🚗 CarRobot - Sistema de Control Autónomo

Sistema robótico con control BLE, navegación autónoma y evasión de obstáculos.

## ✨ Características

- 🤖 **Navegación autónoma** con 4 sensores: 1 Ultrasonido en Slave y 3 VL53L0X en el Master
- 📱 **Control remoto** vía BLE desde celular
- 🛡️ **Sistema de prioridades** (Emergenci a > Manual > Auto > Seguridad)
- 🎯 **Arquitectura distribuida**
- 1) Primera etapa: ESP32 (cerebro) + Arduino (ejecutor)
  2) Segunda etapa: ESP32 Master SPI (cerebro) + ESP32 Slave SPI (ejecutor)
- 🔊 **Comunicación entre microcontroladores 1.- I2C, 2.- SPI** 

## 🎮 Comandos BLE

Caso 1.-Comandos simples:  - `F` - Adelante
Caso 2. Comandos estructurados: tipo, velocidad, ángulo. 


## 🏗️ Arquitectura

