#include "ios_packet_sender.h"

#include <string.h>

#define IOS_TRANSFER_MAX_BYTES      (8u)
#define IOS_DATA_BYTES_PER_TRANSFER (IOS_TRANSFER_MAX_BYTES - 1u)

#if (MMI2C_MAX_MESSAGE_BYTES < IOS_TRANSFER_MAX_BYTES)
#error "The packet sender requires eight-byte I2C transfers"
#endif

size_t packet_serialize(
    const packet_t *packet, uint8_t *output, size_t output_capacity)
{
    if ((packet == NULL) || (output == NULL) ||
        (packet->payload_len > MAX_PAYLOAD_LEN)) {
        return 0u;
    }

    size_t length = PACKET_HEADER_LEN + packet->payload_len;
    if (output_capacity < length) {
        return 0u;
    }

    output[0] = packet->module_id;
    output[1] = packet->message_type;
    output[2] = packet->sequence_number;
    output[3] = packet->payload_len;
    (void) memcpy(&output[PACKET_HEADER_LEN], packet->payload,
        packet->payload_len);
    return length;
}

bool ios_packet_sender_init(ios_packet_sender_t *sender, mmi2c_t *i2c,
    uint8_t ios_address, uint8_t module_id, uint8_t message_type,
    ios_sensor_read_fn sensor_read, void *user)
{
    if ((sender == NULL) || (i2c == NULL) || (ios_address > 0x7Fu) ||
        (sensor_read == NULL)) {
        return false;
    }

    (void) memset(sender, 0, sizeof(*sender));
    sender->i2c = i2c;
    sender->sensor_read = sensor_read;
    sender->user = user;
    sender->ios_address = ios_address;
    sender->module_id = module_id;
    sender->message_type = message_type;
    sender->last_i2c_result = MMI2C_RESULT_OK;
    sender->request_queue = xQueueCreateStatic(1u, sizeof(uint8_t),
        sender->request_queue_buffer, &sender->request_queue_storage);
    return sender->request_queue != NULL;
}

static mmi2c_result_t write_ios(ios_packet_sender_t *sender,
    uint8_t ios_register, const uint8_t *data, uint8_t length)
{
    uint8_t frame[IOS_TRANSFER_MAX_BYTES];
    frame[0] = APOLLO3_IOS_WRITE_COMMAND_BIT | ios_register;
    (void) memcpy(&frame[1], data, length);

    mmi2c_transfer_t transfer = {
        .address = sender->ios_address,
        .tx_data = frame,
        .rx_data = NULL,
        .tx_length = (uint16_t) length + 1u,
        .rx_length = 0u,
    };
    return mmi2c_transfer(sender->i2c, &transfer);
}

static bool fail(ios_packet_sender_t *sender,
    ios_packet_sender_error_t error)
{
    sender->last_error = error;
    ++sender->packets_failed;
    return false;
}

static bool fail_i2c_and_release(ios_packet_sender_t *sender)
{
    mmi2c_result_t release_result = mmi2c_release_bridge(sender->i2c);
    if (release_result != MMI2C_RESULT_OK) {
        sender->last_i2c_result = release_result;
        return fail(sender, IOS_PACKET_ERROR_BRIDGE_RELEASE);
    }
    return fail(sender, IOS_PACKET_ERROR_I2C);
}

bool ios_packet_sender_send(ios_packet_sender_t *sender)
{
    if ((sender == NULL) || (sender->i2c == NULL) ||
        (sender->sensor_read == NULL)) {
        return false;
    }

    packet_t packet = {
        .module_id = sender->module_id,
        .message_type = sender->message_type,
        .sequence_number = sender->next_sequence_number,
        .payload_len = 0u,
    };
    sender->last_i2c_result = MMI2C_RESULT_OK;

    if (!sender->sensor_read(packet.payload, MAX_PAYLOAD_LEN,
            &packet.payload_len, sender->user)) {
        ++sender->sensor_read_failures;
        return fail(sender, IOS_PACKET_ERROR_SENSOR_READ);
    }
    if (packet.payload_len > MAX_PAYLOAD_LEN) {
        return fail(sender, IOS_PACKET_ERROR_PAYLOAD_LENGTH);
    }

    uint8_t tx_data[PACKET_SERIALIZED_MAX_LEN];
    size_t tx_length = packet_serialize(&packet, tx_data, sizeof(tx_data));
    if (tx_length == 0u) {
        return fail(sender, IOS_PACKET_ERROR_SERIALIZATION);
    }

    sender->last_error = IOS_PACKET_ERROR_NONE;
    size_t bytes_sent = 0u;

    while (bytes_sent < tx_length) {
        size_t chunk_length = tx_length - bytes_sent;
        if (chunk_length > IOS_DATA_BYTES_PER_TRANSFER) {
            chunk_length = IOS_DATA_BYTES_PER_TRANSFER;
        }

        sender->last_i2c_result = write_ios(sender,
            (uint8_t) (APOLLO3_IOS_DATA_REGISTER + bytes_sent),
            &tx_data[bytes_sent], (uint8_t) chunk_length);
        if (sender->last_i2c_result != MMI2C_RESULT_OK) {
            return fail_i2c_and_release(sender);
        }
        bytes_sent += chunk_length;
    }

    const uint8_t trigger = APOLLO3_IOS_TRIGGER_VALUE;
    sender->last_i2c_result = write_ios(sender,
        APOLLO3_IOS_TRIGGER_REGISTER, &trigger, 1u);
    if (sender->last_i2c_result != MMI2C_RESULT_OK) {
        return fail_i2c_and_release(sender);
    }

    sender->last_i2c_result = mmi2c_release_bridge(sender->i2c);
    if (sender->last_i2c_result != MMI2C_RESULT_OK) {
        return fail(sender, IOS_PACKET_ERROR_BRIDGE_RELEASE);
    }

    sender->last_error = IOS_PACKET_ERROR_NONE;
    ++sender->next_sequence_number;
    ++sender->packets_completed;
    return true;
}

void ios_packet_sender_task(void *argument)
{
    ios_packet_sender_t *sender = argument;
    uint8_t request;

    for (;;) {
        if (xQueueReceive(sender->request_queue, &request,
                portMAX_DELAY) == pdTRUE) {
            (void) ios_packet_sender_send(sender);
        }
    }
}

void ios_packet_sender_request(ios_packet_sender_t *sender)
{
    if ((sender == NULL) || (sender->request_queue == NULL)) {
        return;
    }

    const uint8_t request = 1u;
    (void) xQueueOverwrite(sender->request_queue, &request);
}

void ios_packet_sender_request_from_isr(ios_packet_sender_t *sender)
{
    if ((sender == NULL) || (sender->request_queue == NULL)) {
        return;
    }

    const uint8_t request = 1u;
    BaseType_t higher_priority_task_woken = pdFALSE;
    (void) xQueueOverwriteFromISR(sender->request_queue, &request,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
