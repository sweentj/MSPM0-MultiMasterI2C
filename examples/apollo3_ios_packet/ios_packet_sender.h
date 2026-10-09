#ifndef IOS_PACKET_SENDER_H
#define IOS_PACKET_SENDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

#if (MMI2C_MAX_MESSAGE_BYTES < 2u)
#error "The IOS sender needs room for a register byte and at least one data byte"
#endif

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

/* Called only from the main loop, never from an ISR. The implementation must
 * set *payload_len to 0..capacity and return true for a valid sample. */
typedef bool (*ios_sensor_read_fn)(uint8_t *payload, uint8_t capacity,
    uint8_t *payload_len, void *user);

typedef enum {
    IOS_PACKET_SENDER_IDLE = 0,
    IOS_PACKET_SENDER_SUBMIT_DATA,
    IOS_PACKET_SENDER_WAIT_DATA,
    IOS_PACKET_SENDER_SUBMIT_TRIGGER,
    IOS_PACKET_SENDER_WAIT_TRIGGER,
    IOS_PACKET_SENDER_RETRY_DELAY,
    IOS_PACKET_SENDER_RELEASE_BRIDGE
} ios_packet_sender_state_t;

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
    mmi2c_now_us_fn now_us;
    void *user;

    uint8_t ios_address;
    uint8_t module_id;
    uint8_t message_type;
    uint8_t next_sequence_number;
    uint8_t max_application_retries;
    uint32_t retry_delay_us;

    volatile uint8_t pending_requests;
    volatile uint32_t dropped_requests;

    packet_t packet;
    uint8_t tx_data[PACKET_SERIALIZED_MAX_LEN];
    uint8_t i2c_frame[MMI2C_MAX_MESSAGE_BYTES];
    uint8_t tx_data_len;
    uint8_t bytes_sent;
    uint8_t current_chunk_len;
    uint8_t application_retries;

    ios_packet_sender_state_t state;
    ios_packet_sender_state_t retry_state;
    uint32_t retry_at_us;
    bool release_is_success;
    mmi2c_result_t last_i2c_result;
    ios_packet_sender_error_t last_error;

    uint32_t packets_completed;
    uint32_t packets_failed;
    uint32_t sensor_read_failures;
} ios_packet_sender_t;

/* Returns zero if the packet is invalid or output_capacity is too small. */
size_t packet_serialize(
    const packet_t *packet, uint8_t *output, size_t output_capacity);

bool ios_packet_sender_init(ios_packet_sender_t *sender, mmi2c_t *i2c,
    uint8_t ios_address, uint8_t module_id, uint8_t message_type,
    ios_sensor_read_fn sensor_read, mmi2c_now_us_fn now_us, void *user,
    uint8_t max_application_retries, uint32_t retry_delay_us);

/* ISR-safe for the intended single-core MSPM0 use. Requests saturate at 255. */
void ios_packet_sender_request_from_isr(ios_packet_sender_t *sender);

/* Call repeatedly from the main loop. It never busy-waits. */
void ios_packet_sender_process(ios_packet_sender_t *sender);

bool ios_packet_sender_is_busy(const ios_packet_sender_t *sender);

#ifdef __cplusplus
}
#endif

#endif
