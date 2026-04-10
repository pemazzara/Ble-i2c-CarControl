// SPISlave.cpp - CORREGIR VERSION CRÍTICA
#include "SPISlave.h"
#include <driver/spi_slave.h>
#include <algorithm> // Añade esto al inicio de tu archivo si no está
#include "freertos/semphr.h" // Header necesario


// Variables estáticas
SPIResponseFrame_t* SPISlave::spi_tx_buffer = nullptr       ;
SPIFrame_t* SPISlave::spi_rx_buffer = nullptr;

extern SemaphoreHandle_t cmd_ready_sem;
extern SemaphoreHandle_t buffer_mutex;
extern SemaphoreHandle_t response_mutex;
ControlCommand_t SPISlave::last_command = {0};
SPIResponseFrame_t SPISlave::last_response = {0};

extern uint16_t calcularChecksum(const void* data, size_t len);
extern UltraSonicMeasure sonar;
// Cambiar la declaración de speedCtrl a un puntero o referencia, 
// y verificar que esté inicializado con un método isReady().
extern SpeedController* pSpeedCtrl;
SonarSensorData_t sensor_data;

SPISlave::SPISlave(MotorControl &motor, UltraSonicMeasure &sensor) {
    // Guardamos la dirección de los objetos
    this->motor_controller = &motor;
    this->sensor_manager = &sensor;
    this->initialized = false;
    this->spi_host = SPI2_HOST;
    
    Serial.println("🔧 SPISlave construido con punteros vinculados");
}
// 1. En el Header (.h), añade esto como miembro privado:
// spi_slave_transaction_t persistent_transaction;

bool SPISlave::init() {
    if (initialized) return true;
    Serial.println("🚀 Inicializando SPI Slave...");
    // 1. Crear semáforos
    cmd_ready_sem = xSemaphoreCreateBinary();
    buffer_mutex = xSemaphoreCreateMutex();
    response_mutex = xSemaphoreCreateMutex();

    size_t size = sizeof(SPIFrame_t); 
    // Aseguramos que el tamaño sea múltiplo de 4 para el DMA
    if (size % 4 != 0) size = ((size / 4) + 1) * 4;
    
    spi_rx_buffer = (SPIFrame_t*) heap_caps_aligned_alloc(4, size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    spi_tx_buffer = (SPIResponseFrame_t*) heap_caps_aligned_alloc(4, size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);    

    if (!spi_rx_buffer || !spi_tx_buffer) {
        Serial.println("❌ No hay memoria DMA para SPI");
        return false;
    }
    memset(spi_rx_buffer, 0, sizeof(SPIFrame_t));
    memset(spi_tx_buffer, 0, sizeof(SPIResponseFrame_t));

    
    if (!cmd_ready_sem || !buffer_mutex || !response_mutex) return false;
    
    // 2. Configuración BUS
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_SLAVE_MOSI,
        .miso_io_num = SPI_SLAVE_MISO,
        .sclk_io_num = SPI_SLAVE_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(SPIFrame_t)
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = SPI_SLAVE_SS,
        .flags = 0,
        .queue_size = 3,
        .mode = 0
    };
    
    // IMPORTANTE: Asegúrate que spi_host sea SPI2_HOST (1)
    esp_err_t ret = spi_slave_initialize(this->spi_host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return false;
      
    // 3. Preparar y Encolar
    prepareResponse(last_response);
    queueNextTransaction(); 
    
    initialized = true;
    Serial.println("🚀 SPI Slave inicializado");
    return true;
}

void SPISlave::queueNextTransaction() {
    // 1. Limpiar el buffer de recepción (DMA RAM) antes de recibir
    memset(spi_rx_buffer, 0, sizeof(SPIFrame_t)); 
    
    // 2. Limpiar la estructura de transacción
    memset(&transaction, 0, sizeof(spi_slave_transaction_t));
    
    transaction.length = sizeof(SPIFrame_t) * 8; 
    transaction.rx_buffer = spi_rx_buffer;
    transaction.tx_buffer = spi_tx_buffer; // Aquí ya debe estar la respuesta preparada
    
    esp_err_t ret = spi_slave_queue_trans(spi_host, &transaction, portMAX_DELAY);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error queue: 0x%X\n", ret);
    }
}



void SPISlave::processSPICommunication() {
    if (!initialized || spi_rx_buffer == nullptr) return;
    
    spi_slave_transaction_t *result = NULL;
    // Timeout 0 para no bloquear la tarea
    esp_err_t ret = spi_slave_get_trans_result(spi_host, &result, pdMS_TO_TICKS(10));

    if (ret == ESP_OK && result != NULL) {
        // 1. DECLARACIÓN: Creamos la variable local que faltaba
        SPIFrame_t rx_local;
        // 2. COPIA SEGURA: Pasamos los datos del buffer DMA a nuestra variable local
        // Nota: spi_rx_buffer es puntero, rx_frame es objeto (usamos &)
        memcpy(&rx_local, spi_rx_buffer, sizeof(SPIFrame_t));
        // VALIDAR MAGIC WORD ANTES DE COPIAR
        if (rx_local.magic_word == 0xA5) processReceivedCommand(rx_local);
        // 3. PREPARAR LA RESPUESTA PARA LA SIGUIENTE VEZ
        // Usamos una variable local para construir la respuesta tranquilamente
        SPIResponseFrame_t nextResponse;
        prepareResponse(nextResponse); 
        // Dentro de prepareResponse ya se hace el memcpy al spi_tx_buffer
        // 4. Encolar la siguiente (el hardware ya tiene la respuesta lista en spi_tx_buffer)
        queueNextTransaction();
    }
}
void SPISlave::processReceivedCommand(const SPIFrame_t& rxFrame) {
    // 1. Verificar Checksum PRIMERO
    uint16_t cs = calcularChecksum((uint8_t*)&rxFrame, sizeof(SPIFrame_t) - 2);
    static uint32_t last_print = 0;

<<<<<<< HEAD
    if (cs != rxFrame.checksum) return; // SALIR INMEDIATAMENTE
    motor_controller->resetSafetyTimer();    
    xSemaphoreGive(cmd_ready_sem);   // Inicializar a 1
    // 2. Guaradr y Señalar que hay nuevo comando        
    if (cmd_ready_sem != NULL && xSemaphoreTake(cmd_ready_sem, pdMS_TO_TICKS(5)) == pdTRUE) {
        last_command = rxFrame.payload;
        xSemaphoreGive(cmd_ready_sem);
    } else {
        Serial.println("❌ Error: No se pudo tomar el semáforo para cmd_ready_sem");
    }
    
}    

bool SPISlave::isCommandReady() {
    if (!cmd_ready_sem) return false;    
    // Intentamos tomarlo con 0 espera. 
    // Si retorna pdTRUE, había una señal y ya la "limpiamos".
    return (xSemaphoreTake(cmd_ready_sem, 0) == pdTRUE);
=======
    if (cs != rxFrame.checksum) return; // SALIR INMEDIATAMENTE   
    motor_controller->resetSafetyTimer();
    if(rxFrame.payload.type == CMD_DRIVE){
       uint16_t pwm = rxFrame.payload.speed;
       uint16_t angle = rxFrame.payload.angle;
       motor_controller->setPWM(pwm, angle, true);
    } 
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Guardar comando para que MotorTask lo procese
        memcpy(&last_command, &rxFrame.payload, sizeof(ControlCommand_t));
        xSemaphoreGive(buffer_mutex);           
        // Señalar que hay nuevo comando
        if (millis() - last_print > 200) {// Debug
            Serial.printf("📥 SPI Cmd: type=0x%02X, speed=%d, angle=%d\n",
                         last_command.type, last_command.speed, last_command.angle);
                last_print = millis();
            }
        }    
>>>>>>> 94d233aa331fd8a74ac1deaf9fecfe638a8a9cb4
}

void SPISlave::prepareResponse(SPIResponseFrame_t &txFrame) {
    memset(&txFrame, 0, sizeof(SPIResponseFrame_t)); // Limpiar todo, incluyendo padding
    
    // 🛡️ GUARDIA 1: Buffers de memoria
    if (spi_tx_buffer == nullptr) return; 
    // 🛡️ GUARDIA 2: Verificar que los periféricos están vinculados
    if (motor_controller == nullptr || sensor_manager == nullptr) {
        return; 
    }

    // 🛡️ GUARDIA 3: Semáforo
    if (xSemaphoreTake(response_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    // Rellenar la estructura de respuesta
    txFrame.msg_id = getNextMsgId(); // Opcional: podrías implementar un contador de mensajes
    // 1. Magic word
    txFrame.magic_word = SPI_MAGIC_SLAVE;
        // 2. Tipo de respuesta (puedes expandir esto según tus necesidades)
    txFrame.type = TYPE_SENSORS; // Por defecto, respondemos con datos de sensores
    txFrame.payload.motors.motor_flags = 0;

    // 2. Motores (Verificar que el puntero no sea NULL antes de usar ->)
    if (motor_controller != nullptr && motor_controller->isInitialized()) {
        txFrame.payload.motors.rpm_left = motor_controller->getCurrentLeft();
        txFrame.payload.motors.rpm_right = motor_controller->getCurrentRight();
        txFrame.payload.motors.a_vel = 0; // Valor por defecto, se actualizará si speedCtrl está listo  
        txFrame.payload.motors.distance = sensor_manager->getLatestSonarData(sensor_data) ? sensor_data.distance : 0xFFFF; // Si no hay datos, poner un valor inválido
        txFrame.payload.motors.motor_flags = motor_controller->getStatusFlags(); // Implementa este método para obtener bits de error

        // Si el controlador está disponible, añadir datos
        if (pSpeedCtrl != nullptr) {
            float avel = pSpeedCtrl->getCurrentAvel();
            float error = pSpeedCtrl->getLastError();

            // Escalar y limitar
            int16_t avel_scaled = (int16_t)constrain(avel * 1000, -32768, 32767);
            int16_t error_scaled = (int16_t)constrain(error * 1000, -32768, 32767);
            txFrame.payload.motors.a_vel = (uint16_t)(avel_scaled < 0 ? -avel_scaled : avel_scaled); // valor absoluto de la velocidad
        }
        
        // Bit 0: Motores parados
        if (motor_controller->getTargetLeft() == 0 && motor_controller->getTargetRight() == 0) {
            txFrame.payload.motors.motor_flags |= 0x01; // Bit 0: motores parados
        }
        } else {
            txFrame.payload.motors.motor_flags = 0x80; // Error: No vinculado o no inicializado
        }

    // 3. Sensores
    if (sensor_manager != nullptr && sensor_manager->initialized) {
        SonarSensorData_t s_data;
        if (sensor_manager->getLatestSonarData(s_data)) {
            txFrame.payload.motors.distance = s_data.distance;
                        // Bit 2: Obstáculo Crítico (Basado en Sonar del Slave)
            if (s_data.distance > 0 && s_data.distance < 50) {
                txFrame.payload.motors.motor_flags |= 0x04; 
            }

            txFrame.payload.motors.a_vel = (uint16_t)(s_data.a_vel < 0 ? -s_data.a_vel : s_data.a_vel); // valor absoluto de la velocidad
            // Opcional: indicar dirección en el campo status
            // Dirección: bit 5 = 1 si acercándose (avel negativo)
            if (s_data.a_vel < 0) { txFrame.payload.motors.motor_flags |= 0x20; // Bit 5 = acercándose
            } else { txFrame.payload.motors.motor_flags &= ~0x20; // limpiar bit (opcional)
            }
            if (s_data.distance > 0 && s_data.distance < 50) txFrame.payload.motors.motor_flags |= 0x04; // Bit 4: Obstáculo Ultra-cercano
            if (!s_data.sensor_ok) txFrame.payload.motors.motor_flags |= 0x08; // Bit 3: Error de hardware Sonar
        }
    } else {
        txFrame.payload.motors.distance = 0xFFFF;
        txFrame.payload.motors.motor_flags |= 0x08;
    }

    // 4. Checksum y Copia a memoria DMA
    txFrame.checksum = calcularChecksum((void*)&txFrame, sizeof(SPIResponseFrame_t) - 2);
    if (spi_tx_buffer != nullptr) {
        memcpy(spi_tx_buffer, &txFrame, sizeof(SPIResponseFrame_t));
    }

    xSemaphoreGive(response_mutex);
}
uint8_t SPISlave::getNextMsgId() {
    static uint8_t msgCounter = 0;
    if (++msgCounter == 0) msgCounter = 1; // saltar 0
    return msgCounter;
}

ControlCommand_t SPISlave::getLastCommand() {

    ControlCommand_t cmd = {0};
    
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&cmd, &last_command, sizeof(ControlCommand_t));
        xSemaphoreGive(buffer_mutex);
    }
    
    return cmd;
}

SPIFrame_t SPISlave::getReceivedFrame() {
    SPIFrame_t frame;
    memset(&frame, 0, sizeof(frame));
    
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&frame, spi_rx_buffer, sizeof(SPIFrame_t));
        xSemaphoreGive(buffer_mutex);
    }
    
    return frame;
}

void SPISlave::signalDataProcessed() {
    // Limpiar semáforo
    while (xSemaphoreTake(data_ready_sem, 0) == pdTRUE) {
        // Consumir todas las señales pendientes
    }
}



