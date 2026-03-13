# 🚗 CarRobot - Sistema de Control Autónomo
Se desrrolla en dos etapas: 1.-Elemental para hobistas en los directorios: arduino-slave y esp32-master. 2.-(En desarrollo) Riguroso para estudiosos de los sitemas de control en los direrctorios: esp32-spi-master y esp32-spi-slave

La idea es llegar a un sistema robótico con control BLE, navegación autónoma y evasión de obstáculos:

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
 
 Para mas detalles ver Wiki del proyecto
