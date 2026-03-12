    class GradientAligner {
    private:
        int16_t last_distance = 0;
        int16_t last_angle = 90;  // Ángulo actual
        int16_t search_direction = 1;  // +1: derecha, -1: izquierda
        const int16_t ANGLE_STEP = 5;  // Paso de exploración (grados)
        const int16_t MIN_DISTANCE = 200;  // Distancia mínima deseada
        uint32_t last_measure_time = 0;
        
    public:
        int16_t computeAngle(uint16_t current_distance) {
            uint32_t now = millis();
            
            // Solo actualizar cada cierto tiempo (ej. 200ms)
            if (now - last_measure_time < 200) {
                return last_angle;  // Mantener ángulo actual
            }
            last_measure_time = now;
            
            // 1. Si es la primera medida, no tenemos referencia
            if (last_distance == 0) {
                last_distance = current_distance;
                return last_angle;
            }
            
            // 2. Calcular cambio en distancia
            int16_t delta_distance = current_distance - last_distance;

            // 3. Lógica de decisión Proporcional
            if (abs(delta_distance) < 5) {
                // Estamos muy cerca del óptimo, no mover o mover mínimo
            } else {
                // El paso de ángulo depende de qué tan rápido está cambiando la distancia
                // Esto es: x3(k+1) = x3(k) + alpha * gradiente
                int16_t adaptive_step = constrain(abs(delta_distance) / 10, 1, ANGLE_STEP);
    
                if (delta_distance > 0) {
                    last_angle += search_direction * adaptive_step;
                } else {
                    search_direction = -search_direction;
                    last_angle += search_direction * adaptive_step;
                }
        }
            /* 3. Lógica de decisión
            if (abs(delta_distance) < 10) {
                // Cambio pequeño: mantener dirección actual
                // (podríamos aumentar paso para explorar más)
            }
            else if (delta_distance > 0) {
                // ¡Nos alejamos! Seguir en esta dirección
                // (el movimiento fue beneficioso)
                last_angle += search_direction * ANGLE_STEP;
            }
            else {
                // Nos acercamos: cambiar dirección
                search_direction = -search_direction;
                last_angle += search_direction * ANGLE_STEP;
            }
            */
            // 4. Mantener ángulo en rango [0, 359]
            last_angle = (last_angle + 360) % 360;
            
            // 5. Guardar distancia para próxima iteración
            last_distance = current_distance;
            
            return last_angle;
        }
        
        // Método para cuando queremos orientarnos a distancia mínima (perpendicular)
        void setTargetPerpendicular() {
            search_direction = 1;  // Reiniciar dirección
            last_distance = 0;      // Forzar reinicio en próxima lectura
        }
    };