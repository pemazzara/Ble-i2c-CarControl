#include "HybridObstacleAvoider.h"
#include <Arduino.h>

// ✅ CONSTRUCTOR CORREGIDO - inicializa el miembro tofScanner
HybridObstacleAvoider::HybridObstacleAvoider(ServoVL53L0X* scanner) 
    : tofScanner(scanner) {
    Serial.println("🤖 HybridObstacleAvoider inicializado");
}

// ✅ DECIDEACTION CORREGIDO - usa el miembro tofScanner para escanear internamente
byte HybridObstacleAvoider::decideAction(uint16_t sonarDist) { 
    // 1. Obtener datos TOF internamente usando el miembro
    uint16_t tofLeft, tofFront, tofRight;
    tofScanner->navigationScan(tofLeft, tofFront, tofRight);

    // 2. Verificar alerta del sonar
    SonarAlert sonarStatus = checkSonarAlert(sonarDist);
   
    if (sonarStatus == ALERT_CRITICAL) {
        Serial.println("🚨 ALERTA CRÍTICA SONAR - Parada inmediata");
        return CMD_EMERGENCY_STOP;  // ✅ PUNTO Y COMA CORREGIDO
    }
    
    // 3. Calcular decisión fusionada
    return calculateFusedDecision(sonarStatus, tofLeft, tofFront, tofRight);
}

// 🔄 MÉTODOS PRIVADOS (sin cambios)
HybridObstacleAvoider::SonarAlert HybridObstacleAvoider::checkSonarAlert(uint16_t sonarDist) {
    if (sonarDist < 300) {
        return ALERT_CRITICAL;
    } else if (sonarDist < 600) {
        return ALERT_WARNING;
    }
    return ALERT_NONE;
}

byte HybridObstacleAvoider::calculateFusedDecision(SonarAlert sonarAlert, uint16_t left, uint16_t front, uint16_t right) {
    Serial.printf("🧠 FUSIÓN | Sonar:%s | TOF L:%d F:%d R:%d\n", 
                 sonarAlertToString(sonarAlert), left, front, right);
    
    if (sonarAlert == ALERT_WARNING) {
        return defensiveNavigation(left, front, right);
    }
    
    if (front > 400) {
        if (left < 150 && right > 250) {
            return CMD_AUTO_RIGHT;
        } else if (right < 150 && left > 250) {
            return CMD_AUTO_LEFT;
        }
        return CMD_AUTO_FORWARD;
    }
    
    return preciseObstacleAvoidance(left, front, right);
}

byte HybridObstacleAvoider::defensiveNavigation(uint16_t left, uint16_t front, uint16_t right) {
    Serial.println("🛡️  Modo defensivo (alerta sonar)");
    
    if (front > 600) {
        return CMD_AUTO_FORWARD;
    }
    
    if (left > 500 && right > 500) {
        return (left > right) ? CMD_AUTO_LEFT : CMD_AUTO_RIGHT;
    } else if (left > 400) {
        return CMD_AUTO_LEFT;
    } else if (right > 400) {
        return CMD_AUTO_RIGHT;
    }
    
    return CMD_AUTO_BACKWARD;
}

byte HybridObstacleAvoider::preciseObstacleAvoidance(uint16_t left, uint16_t front, uint16_t right) {
    int leftScore = left + (left > 300 ? 100 : 0);
    int rightScore = right + (right > 300 ? 100 : 0);
    
    if (leftScore > rightScore && leftScore > 200) {
        Serial.println("↩️  Giro preciso izquierda");
        return CMD_AUTO_LEFT;
    } else if (rightScore > 200) {
        Serial.println("↪️  Giro preciso derecha");
        return CMD_AUTO_RIGHT;
    }
    
    Serial.println("🔙 Retroceso controlado");
    return CMD_AUTO_BACKWARD;
}

const char* HybridObstacleAvoider::sonarAlertToString(SonarAlert alert) {
    switch(alert) {
        case ALERT_NONE: return "NONE";
        case ALERT_WARNING: return "WARNING";
        case ALERT_CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

