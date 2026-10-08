#include "sensor_stub.h"

#include <stddef.h>

/* Replace this function with the real sensor driver. This deterministic stub
 * supplies two little-endian 16-bit values so the example runs on a bench
 * without selecting a particular sensor. */
bool sensor_read_payload(uint8_t *payload, uint8_t capacity,
    uint8_t *payload_len, void *user)
{
    (void) user;
    static uint16_t sample_number;

    if ((payload == NULL) || (payload_len == NULL) || (capacity < 4u)) {
        return false;
    }

    uint16_t sensor_counts = (uint16_t) (1000u + sample_number);
    int16_t temperature_centi_degrees =
        (int16_t) (2150 + (sample_number % 25u));
    ++sample_number;

    payload[0] = (uint8_t) sensor_counts;
    payload[1] = (uint8_t) (sensor_counts >> 8);
    payload[2] = (uint8_t) temperature_centi_degrees;
    payload[3] = (uint8_t) ((uint16_t) temperature_centi_degrees >> 8);
    *payload_len = 4u;
    return true;
}
