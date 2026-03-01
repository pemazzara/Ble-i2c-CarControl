/*
void handleBLECommand(String command) {
    if (command == "E") {
        // Emergencia - máxima prioridad
        motorControl.sendI2CCommand(CMD_EMERGENCY_STOP);
        sensorFusion.setManualMode(false);
    }
    else if (command == "M") {
        // Modo manual
        sensorFusion.setManualMode(true);
    }
    else if (command == "A") {
        // Modo autónomo
        sensorFusion.setManualMode(false);
    }
    else if (command == "F" || command == "B" || command == "L" || command == "R") {
        // Comando manual - solo si estamos en modo manual
        if (sensorFusion.isManualMode()) {
            byte cmd = convertBLEToI2C(command);
            motorControl.sendI2CCommand(cmd);
        }
    }
}*/