

#include "messages.h"
#include <stdint.h>

// Function to compute the CRC-8 checksum
uint8_t crc8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0x00; // Initial value

    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR the data byte with the CRC

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL; // Shift left and XOR with the polynomial
            } else {
                crc <<= 1; // Just shift left
            }
        }
    }

    return crc;
}


float array_to_float(const uint8_t *buff) {
    uint32_t as_int = ((uint32_t)buff[0]) |
                      ((uint32_t)buff[1] << 8) |
                      ((uint32_t)buff[2] << 16) |
                      ((uint32_t)buff[3] << 24);

    return *((float*)&as_int);
}
