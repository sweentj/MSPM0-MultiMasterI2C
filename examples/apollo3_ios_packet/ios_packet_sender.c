#include "ios_packet_sender.h"

#include <string.h>

#define IOS_DATA_BYTES_PER_TRANSFER (MMI2C_MAX_MESSAGE_BYTES - 1u)

static bool time_reached(uint32_t now, uint32_t target)
{
    return ((int32_t) (now - target) >= 0);
}

static bool take_request(ios_packet_sender_t *sender)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool have_request = (sender->pending_requests != 0u);
    if (have_request) {
        --sender->pending_requests;
    }
    /* Restore the caller's exact interrupt state before sensor acquisition. */
    __set_PRIMASK(primask);
    return have_request;
}

size_t packet_serialize(
    const packet_t *packet, uint8_t *output, size_t output_capacity)
{
    if ((packet == NULL) || (output == NULL) ||
        (packet->payload_len > MAX_PAYLOAD_LEN)) {
        return 0u;
    }

    size_t serialized_len = PACKET_HEADER_LEN + packet->payload_len;
    if (output_capacity < serialized_len) {
        return 0u;
    }

    output[0] = packet->module_id;
    output[1] = packet->message_type;
    output[2] = packet->sequence_number;
    output[3] = packet->payload_len;
    if (packet->payload_len != 0u) {
        (void) memcpy(&output[PACKET_HEADER_LEN], packet->payload,
            packet->payload_len);
    }
    return serialized_len;
}

bool ios_packet_sender_init(ios_packet_sender_t *sender, mmi2c_t *i2c,
    uint8_t ios_address, uint8_t module_id, uint8_t message_type,
    ios_sensor_read_fn sensor_read, mmi2c_now_us_fn now_us, void *user,
    uint8_t max_application_retries, uint32_t retry_delay_us)
{
    if ((sender == NULL) || (i2c == NULL) || (ios_address > 0x7Fu) ||
        (sensor_read == NULL) || (now_us == NULL) || (retry_delay_us == 0u)) {
        return false;
    }

    (void) memset(sender, 0, sizeof(*sender));
    sender->i2c                    = i2c;
    sender->ios_address            = ios_address;
    sender->module_id              = module_id;
    sender->message_type           = message_type;
    sender->sensor_read            = sensor_read;
    sender->now_us                 = now_us;
    sender->user                   = user;
    sender->max_application_retries = max_application_retries;
    sender->retry_delay_us         = retry_delay_us;
    sender->last_i2c_result        = MMI2C_RESULT_NONE;
    sender->last_error             = IOS_PACKET_ERROR_NONE;
    sender->state                  = IOS_PACKET_SENDER_IDLE;
    return true;
}

void ios_packet_sender_request_from_isr(ios_packet_sender_t *sender)
{
    if (sender == NULL) {
        return;
    }

    if (sender->pending_requests != UINT8_MAX) {
        ++sender->pending_requests;
    } else {
        ++sender->dropped_requests;
    }
}

static void begin_bridge_release(ios_packet_sender_t *sender, bool success,
    mmi2c_result_t result)
{
    sender->release_is_success = success;
    sender->last_i2c_result    = result;
    sender->state              = IOS_PACKET_SENDER_RELEASE_BRIDGE;
}

static void handle_failed_transfer(ios_packet_sender_t *sender,
    mmi2c_result_t result, ios_packet_sender_state_t retry_state)
{
    sender->last_i2c_result = result;
    sender->last_error      = IOS_PACKET_ERROR_I2C;
    if (sender->application_retries < sender->max_application_retries) {
        ++sender->application_retries;
        sender->retry_state = retry_state;
        sender->retry_at_us = sender->now_us(sender->user) +
                              sender->retry_delay_us;
        sender->state = IOS_PACKET_SENDER_RETRY_DELAY;
    } else {
        begin_bridge_release(sender, false, result);
    }
}

static void submit_data_chunk(ios_packet_sender_t *sender)
{
    uint8_t remaining = sender->tx_data_len - sender->bytes_sent;
    uint8_t chunk_len = remaining;
    if (chunk_len > IOS_DATA_BYTES_PER_TRANSFER) {
        chunk_len = IOS_DATA_BYTES_PER_TRANSFER;
    }

    sender->i2c_frame[0] = APOLLO3_IOS_WRITE_COMMAND_BIT |
                           (uint8_t) (APOLLO3_IOS_DATA_REGISTER +
                                     sender->bytes_sent);
    (void) memcpy(&sender->i2c_frame[1],
        &sender->tx_data[sender->bytes_sent], chunk_len);
    sender->current_chunk_len = chunk_len;

    mmi2c_transfer_t transfer = {
        .address   = sender->ios_address,
        .tx_data   = sender->i2c_frame,
        .rx_data   = NULL,
        .tx_length = (uint16_t) chunk_len + 1u,
        .rx_length = 0u,
    };
    mmi2c_result_t result = mmi2c_submit(sender->i2c, &transfer);
    if ((result == MMI2C_RESULT_IN_PROGRESS) ||
        (result == MMI2C_RESULT_BRIDGE_NOT_READY)) {
        sender->state = IOS_PACKET_SENDER_WAIT_DATA;
    } else if (result != MMI2C_RESULT_BUSY) {
        handle_failed_transfer(
            sender, result, IOS_PACKET_SENDER_SUBMIT_DATA);
    }
}

static void submit_trigger(ios_packet_sender_t *sender)
{
    sender->i2c_frame[0] = APOLLO3_IOS_WRITE_COMMAND_BIT |
                           APOLLO3_IOS_TRIGGER_REGISTER;
    sender->i2c_frame[1] = APOLLO3_IOS_TRIGGER_VALUE;

    mmi2c_transfer_t transfer = {
        .address   = sender->ios_address,
        .tx_data   = sender->i2c_frame,
        .rx_data   = NULL,
        .tx_length = 2u,
        .rx_length = 0u,
    };
    mmi2c_result_t result = mmi2c_submit(sender->i2c, &transfer);
    if ((result == MMI2C_RESULT_IN_PROGRESS) ||
        (result == MMI2C_RESULT_BRIDGE_NOT_READY)) {
        sender->state = IOS_PACKET_SENDER_WAIT_TRIGGER;
    } else if (result != MMI2C_RESULT_BUSY) {
        handle_failed_transfer(
            sender, result, IOS_PACKET_SENDER_SUBMIT_TRIGGER);
    }
}

static void wait_for_data_result(ios_packet_sender_t *sender)
{
    mmi2c_result_t result = mmi2c_get_result(sender->i2c);
    if (result == MMI2C_RESULT_IN_PROGRESS) {
        return;
    }

    result = mmi2c_take_result(sender->i2c);
    if (result == MMI2C_RESULT_OK) {
        sender->bytes_sent += sender->current_chunk_len;
        sender->application_retries = 0u;
        sender->state = (sender->bytes_sent == sender->tx_data_len)
                            ? IOS_PACKET_SENDER_SUBMIT_TRIGGER
                            : IOS_PACKET_SENDER_SUBMIT_DATA;
    } else if (result != MMI2C_RESULT_IN_PROGRESS) {
        handle_failed_transfer(
            sender, result, IOS_PACKET_SENDER_SUBMIT_DATA);
    }
}

static void wait_for_trigger_result(ios_packet_sender_t *sender)
{
    mmi2c_result_t result = mmi2c_get_result(sender->i2c);
    if (result == MMI2C_RESULT_IN_PROGRESS) {
        return;
    }

    result = mmi2c_take_result(sender->i2c);
    if (result == MMI2C_RESULT_OK) {
        begin_bridge_release(sender, true, result);
    } else if (result != MMI2C_RESULT_IN_PROGRESS) {
        handle_failed_transfer(
            sender, result, IOS_PACKET_SENDER_SUBMIT_TRIGGER);
    }
}

static void finish_preparation_failure(ios_packet_sender_t *sender,
    ios_packet_sender_error_t error)
{
    /* No I2C transaction has started, so no bridge release is necessary.
     * Return explicitly to IDLE; the GPIO/timer IRQs are already restored by
     * take_request(), and any later or already-queued request can run. */
    sender->tx_data_len         = 0u;
    sender->bytes_sent          = 0u;
    sender->current_chunk_len   = 0u;
    sender->application_retries = 0u;
    sender->last_i2c_result     = MMI2C_RESULT_NONE;
    sender->last_error          = error;
    ++sender->packets_failed;
    sender->state = IOS_PACKET_SENDER_IDLE;
}

static void start_packet(ios_packet_sender_t *sender)
{
    sender->packet.module_id       = sender->module_id;
    sender->packet.message_type    = sender->message_type;
    sender->packet.sequence_number = sender->next_sequence_number;
    sender->packet.payload_len     = 0u;

    if (!sender->sensor_read(sender->packet.payload, MAX_PAYLOAD_LEN,
            &sender->packet.payload_len, sender->user)) {
        ++sender->sensor_read_failures;
        finish_preparation_failure(sender, IOS_PACKET_ERROR_SENSOR_READ);
        return;
    }
    if (sender->packet.payload_len > MAX_PAYLOAD_LEN) {
        finish_preparation_failure(sender, IOS_PACKET_ERROR_PAYLOAD_LENGTH);
        return;
    }

    size_t serialized_len = packet_serialize(
        &sender->packet, sender->tx_data, sizeof(sender->tx_data));
    if (serialized_len == 0u) {
        finish_preparation_failure(sender, IOS_PACKET_ERROR_SERIALIZATION);
        return;
    }

    sender->tx_data_len        = (uint8_t) serialized_len;
    sender->bytes_sent         = 0u;
    sender->current_chunk_len  = 0u;
    sender->application_retries = 0u;
    sender->last_i2c_result    = MMI2C_RESULT_IN_PROGRESS;
    sender->last_error         = IOS_PACKET_ERROR_NONE;
    sender->state              = IOS_PACKET_SENDER_SUBMIT_DATA;
}

void ios_packet_sender_process(ios_packet_sender_t *sender)
{
    if ((sender == NULL) || (sender->i2c == NULL)) {
        return;
    }

    mmi2c_process(sender->i2c);

    /* Drain states that require no external event. This ensures, for example,
     * that IDLE -> prepare -> SUBMIT_DATA happens in one call and does not
     * depend on a periodic interrupt waking the main loop. */
    for (;;) {
        switch (sender->state) {
            case IOS_PACKET_SENDER_IDLE:
                if (!take_request(sender)) {
                    return;
                }
                start_packet(sender);
                if (sender->state == IOS_PACKET_SENDER_IDLE) {
                    /* Packet preparation failed. Bound this invocation rather
                     * than draining every queued request after a sensor fault. */
                    return;
                }
                break;

            case IOS_PACKET_SENDER_SUBMIT_DATA:
                submit_data_chunk(sender);
                if (sender->state == IOS_PACKET_SENDER_SUBMIT_DATA) {
                    /* The bus context reported BUSY. Wait for its next event. */
                    return;
                }
                break;

            case IOS_PACKET_SENDER_WAIT_DATA:
                wait_for_data_result(sender);
                if (sender->state == IOS_PACKET_SENDER_WAIT_DATA) {
                    return;
                }
                break;

            case IOS_PACKET_SENDER_SUBMIT_TRIGGER:
                submit_trigger(sender);
                if (sender->state == IOS_PACKET_SENDER_SUBMIT_TRIGGER) {
                    return;
                }
                break;

            case IOS_PACKET_SENDER_WAIT_TRIGGER:
                wait_for_trigger_result(sender);
                if (sender->state == IOS_PACKET_SENDER_WAIT_TRIGGER) {
                    return;
                }
                break;

            case IOS_PACKET_SENDER_RETRY_DELAY:
                if (!time_reached(sender->now_us(sender->user),
                        sender->retry_at_us)) {
                    return;
                }
                sender->state = sender->retry_state;
                break;

            case IOS_PACKET_SENDER_RELEASE_BRIDGE: {
                mmi2c_result_t release_result =
                    mmi2c_release_bridge(sender->i2c);
                if (release_result == MMI2C_RESULT_BUSY) {
                    return;
                }
                if (sender->release_is_success &&
                    (release_result == MMI2C_RESULT_OK)) {
                    ++sender->packets_completed;
                    ++sender->next_sequence_number;
                    sender->last_error = IOS_PACKET_ERROR_NONE;
                } else {
                    ++sender->packets_failed;
                    if (release_result != MMI2C_RESULT_OK) {
                        sender->last_i2c_result = release_result;
                        sender->last_error = IOS_PACKET_ERROR_BRIDGE_RELEASE;
                    }
                }
                sender->state = IOS_PACKET_SENDER_IDLE;
                break;
            }

            default:
                begin_bridge_release(
                    sender, false, MMI2C_RESULT_INVALID_ARGUMENT);
                break;
        }
    }
}

bool ios_packet_sender_is_busy(const ios_packet_sender_t *sender)
{
    return (sender != NULL) && (sender->state != IOS_PACKET_SENDER_IDLE);
}
