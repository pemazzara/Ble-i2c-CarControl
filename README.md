# Ble-i2c-CarControl
# 🚗 CarRobot - Sistema de Control Autónomo

Sistema robótico con control BLE, navegación autónoma y evasión de obstáculos.

## ✨ Características

- 🤖 **Navegación autónoma** con 3 sensores VL53L0X
- 📱 **Control remoto** vía BLE desde celular
- 🛡️ **Sistema de prioridades** (Emergencia > Manual > Auto > Seguridad)
- 🎯 **Arquitectura distribuida** ESP32 (cerebro) + Arduino (ejecutor)
- 🔊 **Comunicación I2C** entre microcontroladores

## 🎮 Comandos BLE

- `F` - Adelante
- `B` - Atrás  
- `L` - Izquierda
- `R` - Derecha
- `S` - Stop
- `A` - Modo autónomo
- `1/2/3` - Velocidades
- `F,L,R` - Calibración de sensores

## 🏗️ Arquitectura

