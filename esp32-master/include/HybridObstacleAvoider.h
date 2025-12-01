#ifndef HYBRID_OBSTACLE_AVOIDER_H
#define HYBRID_OBSTACLE_AVOIDER_H

#include "ServoVL53L0X.h"
#include "Command.h"

class HybridObstacleAvoider {
private:
    ServoVL53L0X* tofScanner;  // ✅ MIEMBRO AGREGADO
    
    enum SonarAlert {
        ALERT_NONE,
        ALERT_WARNING,
        ALERT_CRITICAL
    };
    
public:
    // ✅ CONSTRUCTOR CORREGIDO - recibe el scanner
    HybridObstacleAvoider(ServoVL53L0X* scanner);
    
    // ✅ MANTENER FIRMA SIMPLE - escanea internamente
    byte decideAction(uint16_t sonarDist);
    
private:
    SonarAlert checkSonarAlert(uint16_t sonarDist);
    byte calculateFusedDecision(SonarAlert sonarAlert, uint16_t left, uint16_t front, uint16_t right);
    byte defensiveNavigation(uint16_t left, uint16_t front, uint16_t right);
    byte preciseObstacleAvoidance(uint16_t left, uint16_t front, uint16_t right);
    const char* sonarAlertToString(SonarAlert alert);
};

#endif
/* ESP32 - HybridObstacleAvoider.h
#ifndef HYBRID_OBSTACLE_AVOIDER_H
#define HYBRID_OBSTACLE_AVOIDER_H
#include "ServoVL53L0X.h"
#include "SonarReceiver.h"
#include "Command.h" // Incluir comandos

class HybridObstacleAvoider {
private:
    ServoVL53L0X* tofScanner;
    SonarReceiver* sonarReceiver;
    
    // Estados de alerta del sonar
    enum SonarAlert {
        ALERT_NONE,
        ALERT_WARNING,    // Obstáculo detectado a media distancia
        ALERT_CRITICAL    // Obstáculo cercano
    };
    
public:
    HybridObstacleAvoider(ServoVL53L0X* scanner, SonarReceiver* sonar) 
        : tofScanner(scanner), sonarReceiver(sonar) {}
    // Función principal de toma de decisiones
    byte decideAction();
    //byte decideAction;
    private:
    SonarAlert checkSonarAlert();
    byte calculateFusedDecision(SonarAlert sonarAlert, uint16_t left, uint16_t front, uint16_t right);
    byte defensiveNavigation(uint16_t left, uint16_t front, uint16_t right);
    byte preciseObstacleAvoidance(uint16_t left, uint16_t front, uint16_t right);
    const char* sonarAlertToString(SonarAlert alert);
};

#endif

        // 1. VERIFICAR ALERTAS DEL SONAR (máxima prioridad)
        SonarAlert sonarStatus = checkSonarAlert();
        
        if (sonarStatus == ALERT_CRITICAL) {
            Serial.println("🚨 ALERTA CRÍTICA SONAR - Parada inmediata");
            return CMD_EMERGENCY_STOP;
        }
        
        // 2. ESCANEO RÁPIDO CON VL53L0X
        uint16_t tofLeft, tofFront, tofRight;
        tofScanner->navigationScan(tofLeft, tofFront, tofRight);
        
        // 3. FUSIÓN SENSORIAL INTELIGENTE
        return calculateFusedDecision(sonarStatus, tofLeft, tofFront, tofRight);
    }
    
private:
    SonarAlert checkSonarAlert() {
        uint16_t sonarDist = sonarReceiver->getDistance();
        
        if (sonarDist < 300) { // 30cm - crítico
            return ALERT_CRITICAL;
        } else if (sonarDist < 600) { // 60cm - advertencia
            return ALERT_WARNING;
        }
        return ALERT_NONE;
    }
    
    byte calculateFusedDecision(SonarAlert sonarAlert, uint16_t left, uint16_t front, uint16_t right) {
        Serial.printf("🧠 FUSIÓN | Sonar:%s | TOF L:%d F:%d R:%d\n", 
                     sonarAlertToString(sonarAlert), left, front, right);
        
        // A. SI SONAR DETECTA PELIGRO: Comportamiento defensivo
        if (sonarAlert == ALERT_WARNING) {
            return defensiveNavigation(left, front, right);
        }
        
        // B. NAVEGACIÓN NORMAL CON TOF DE PRECISIÓN
        if (front > 400) {
            // Camino despejado - avanzar
            if (left < 150 && right > 250) {
                return CMD_AUTO_RIGHT; // Ajuste preventivo
            } else if (right < 150 && left > 250) {
                return CMD_AUTO_LEFT; // Ajuste preventivo
            }
            return CMD_AUTO_FORWARD;
        }
        
        // C. EVASIÓN DE OBSTÁCULOS PRECISA
        return preciseObstacleAvoidance(left, front, right);
    }
    
    byte defensiveNavigation(uint16_t left, uint16_t front, uint16_t right) {
        Serial.println("🛡️  Modo defensivo (alerta sonar)");
        
        // Reducir velocidad y ser más cauteloso
        if (front > 600) { // Mayor margen de seguridad
            return CMD_AUTO_FORWARD_SLOW; // Comando de velocidad reducida
        }
        
        // Buscar escape seguro con mayores distancias
        if (left > 500 && right > 500) {
            return (left > right) ? CMD_AUTO_LEFT : CMD_AUTO_RIGHT;
        } else if (left > 400) {
            return CMD_AUTO_LEFT;
        } else if (right > 400) {
            return CMD_AUTO_RIGHT;
        }
        
        return CMD_AUTO_BACKWARD;
    }
    
    byte preciseObstacleAvoidance(uint16_t left, uint16_t front, uint16_t right) {
        // Lógica de evasión mejorada con TOF de precisión
        int leftScore = left + (left > 300 ? 100 : 0); // Bonus por camino ancho
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
    
    const char* sonarAlertToString(SonarAlert alert) {
        switch(alert) {
            case ALERT_NONE: return "NONE";
            case ALERT_WARNING: return "WARNING";
            case ALERT_CRITICAL: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }
};*/