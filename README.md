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

Recapitulando: El ususario expresa su deseo de velocidad entre 0 y 1000, que para nuestro sistema se convertirá en el intervalo [0, 1]. El sistema debería tener un método de medir la velocidad, en nuestro caso eso sólo ocurre cuando hay un obstáculo visible. Supongamos que ese sea el caso, el sistema mide la tasa de aproximación al objeto (que no sería exactamente la velocidad de aproximación). El control PID se encargaría de ajustar el PWM el motor para minimizar la diferencia entre la tasa medida y la velocidad deseada.

## 🏗️ Arquitectura

