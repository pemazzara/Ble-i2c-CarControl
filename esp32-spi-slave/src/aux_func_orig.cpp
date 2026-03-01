#include <cstddef>
#include <cstdint>

// Función de Checksum genérica
uint16_t calcularChecksum(void* data, size_t len) {
    uint8_t* byteData = (uint8_t*)data;
    uint32_t checksum = 0;

    for (size_t i = 0; i < len; i++) {
        checksum += byteData[i];
    }
    
    return (uint16_t)(checksum & 0xFFFF);
}
