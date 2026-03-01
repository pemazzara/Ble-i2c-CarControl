// aux_func.cpp Fletcher-16
#include <cstddef>
#include <cstdint>
#include "SPIDefinitions.h"

/**
 * Calcula un checksum simple por suma.
 * @param data Puntero a los datos (usualmente el inicio de la estructura)
 * @param len Longitud a calcular (sizeof(Estructura) - 2 para ignorar el campo checksum)
 */
uint16_t calcularChecksum(const void* data, size_t len) {
    if (data == nullptr || len == 0) return 0;

    const uint8_t* byteData = (const uint8_t*)data;
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for (size_t i = 0; i < len; i++) {
        // Primera suma: Acumula el valor del byte (módulo 255)
        sum1 = (sum1 + byteData[i]) % 255;
        // Segunda suma: Acumula la progresión de la primera suma (módulo 255)
        sum2 = (sum2 + sum1) % 255;
    }

    // Combinamos ambos resultados en un solo valor de 16 bits
    return (uint16_t)((sum2 << 8) | sum1);
}