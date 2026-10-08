#ifndef SENSOR_STUB_H
#define SENSOR_STUB_H

#include <stdbool.h>
#include <stdint.h>

bool sensor_read_payload(uint8_t *payload, uint8_t capacity,
    uint8_t *payload_len, void *user);

#endif
