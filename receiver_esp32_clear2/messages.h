

#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h> // Add this line to include the stdint.h header

#define M_HEADER_0 0xba
#define M_HEADER_1 0xcc


// Error codes
#define M_ERROR_OK 0x00
#define M_ERROR_GENERIC 0x01

/* responses */
#define M_ID_SETTINGS_CHEKSUM 200
// #define M_ID_SETTINGS_POS_VEL    201  // sending countinuosly
// #define M_ID_SETTINGS_ARMED      202  // sending countinuosly
// #define M_ID_SETTINGS_SETPOINT   203  // sending countinuosly
#define M_ID_SETTINGS_PID        204
#define M_ID_SETTINGS_TRIM       205

/* incoming data packets */
#define M_ID_POS_VEL    0x01
#define M_ID_ARMED      0x02
#define M_ID_SETPOINT   0x03
#define M_ID_PID        0x04
#define M_ID_TRIM       0x05


/*  output data packets */
#define M_ID_CONTROLS_STATE   0x40


// Define the polynomial for CRC-8
#define POLYNOMIAL 0x07


#if defined(__cplusplus)
extern "C"
{
#endif

uint8_t crc8(const uint8_t *data, uint8_t length);
float array_to_float(const uint8_t *buff);

#if defined(__cplusplus)
}
#endif

#endif // MESSAGES_H
