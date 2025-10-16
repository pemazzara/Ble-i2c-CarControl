# Ble-i2c-CarControl
The motors of a car are controlled using an ESP32 micro, which sends commands through an i2c interface to another controller, which could be an Arduino Uno, which directly controls the motors. In turn, the ESP32 communicates via Bluetooth LE with a cell phone application. Simple commands: "F", "B", "L", "R", "1", "2", "3", "A"
