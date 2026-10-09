#ifndef IOS_PACKET_SENDER_H
#define IOS_PACKET_SENDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "mmi2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PAYLOAD_LEN                 (4u)
#define PACKET_HEADER_LEN               (4u)
#define PACKET_SERIALIZED_MAX_LEN       (PACKET_HEADER_LEN + MAX_PAYLOAD_LEN)

#define APOLLO3_IOS_DATA_REGISTER       (0x50u)
#define APOLLO3_IOS_TRIGGER_REGISTER    (0x01u)
#define APOLLO3_IOS_TRIGGER_VALUE       (0x80u)
#define APOLLO3_IOS_WRITE_COMMAND_BIT   (0x80u)

#if (PACKET_SERIALIZED_MAX_LEN > 8u)
#error "The application packet is limited to 64 data bits"
#endif

typedef struct {
    uint8_t module_id;
    uint8_t message_type;
    uint8_t sequence_number;
    uint8_t payload_len;
    uint8_t payload[MAX_PAYLOAD_LEN];
} packet_t;

typedef bool (*ios_sensor_read_fn)(uint8_t *payload, uint8_t capacity,
    uint8_t *payload_len, void *user);

typedef enum {
    IOS_PACKET_ERROR_NONE = 0,
    IOS_PACKET_ERROR_SENSOR_READ,
    IOS_PACKET_ERROR_PAYLOAD_LENGTH,
    IOS_PACKET_ERROR_SERIALIZATION,
    IOS_PACKET_ERROR_I2C,
    IOS_PACKET_ERROR_BRIDGE_RELEASE
} ios_packet_sender_error_t;

typedef struct {
    mmi2c_t *i2c;
    ios_sensor_read_fn sensor_read;
    void *user;
    uint8_t ios_address;
    uint8_t module_id;
    uint8_t message_type;
    uint8_t next_sequence_number;

    QueueHandle_t request_queue;
    StaticQueue_t request_queue_storage;
    uint8_t request_queue_buffer[1u];

    mmi2c_result_t last_i2c_result;
    ios_packet_sender_error_t last_error;
    uint32_t packets_completed;
    uint32_t packets_failed;
    uint32_t sensor_read_failures;
} ios_packet_sender_t;

size_t packet_serialize(
    const packet_t *packet, uint8_t *output, size_t output_capacity);

bool ios_packet_sender_init(ios_packet_sender_t *sender, mmi2c_t *i2c,
    uint8_t ios_address, uint8_t module_id, uint8_t message_type,
    ios_sensor_read_fn sensor_read, void *user);

/* Blocks forever on the one-entry request queue and sends one packet/request. */
void ios_packet_sender_task(void *argument);

/* Queue one request. Repeated pending requests are intentionally coalesced. */
void ios_packet_sender_request(ios_packet_sender_t *sender);
void ios_packet_sender_request_from_isr(ios_packet_sender_t *sender);

/* Exposed for tests or a caller that already owns a task-level trigger. */
bool ios_packet_sender_send(ios_packet_sender_t *sender);

#ifdef __cplusplus
}
#endif

#endif
