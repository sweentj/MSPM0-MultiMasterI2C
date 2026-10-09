#include "ios_packet_sender.h"

#include <string.h>

#define IOS_DATA_BYTES_PER_TRANSFER (MMI2C_MAX_MESSAGE_BYTES - 1u)

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
    ios_sensor_read_fn sensor_read, void *user, uint8_t max_write_retries)
{
    if ((sender == NULL) || (i2c == NULL) || (ios_address > 0x7Fu) ||
        (sensor_read == NULL)) {
        return false;
    }

    (void) memset(sender, 0, sizeof(*sender));
    sender->i2c               = i2c;
    sender->sensor_read       = sensor_read;
    sender->user              = user;
    sender->ios_address       = ios_address;
    sender->module_id         = module_id;
    sender->message_type      = message_type;
    sender->max_write_retries = max_write_retries;
    sender->last_i2c_result   = MMI2C_RESULT_NONE;
    sender->last_error        = IOS_PACKET_ERROR_NONE;
    return true;
}

/* Wait for the interrupt-driven I2C transfer to finish. Timeouts and
 * arbitration backoff are checked by mmi2c_process() in this polling loop. */
static mmi2c_result_t wait_for_i2c(mmi2c_t *i2c)
{
    while (mmi2c_get_result(i2c) == MMI2C_RESULT_IN_PROGRESS) {
        mmi2c_process(i2c);
    }
    return mmi2c_take_result(i2c);
}

static mmi2c_result_t write_i2c(ios_packet_sender_t *sender,
    const uint8_t *data, uint16_t length)
{
    mmi2c_transfer_t transfer = {
        .address   = sender->ios_address,
        .tx_data   = data,
        .rx_data   = NULL,
        .tx_length = length,
        .rx_length = 0u,
    };

    /* One initial attempt, followed by max_write_retries retries. */
    uint16_t attempts = (uint16_t) sender->max_write_retries + 1u;
    for (uint16_t attempt = 0u; attempt < attempts; ++attempt) {
        mmi2c_result_t result = mmi2c_submit(sender->i2c, &transfer);

        if (result == MMI2C_RESULT_IN_PROGRESS) {
            result = wait_for_i2c(sender->i2c);
        } else if (result == MMI2C_RESULT_BRIDGE_NOT_READY) {
            /* mmi2c_submit() can finish immediately, for example when the
             * bridge cannot be enabled. Take that result to return to IDLE. */
            result = mmi2c_take_result(sender->i2c);
        }

        if (result == MMI2C_RESULT_OK) {
            return result;
        }

        sender->last_i2c_result = result;
        if ((attempt + 1u) == attempts) {
            return result;
        }
    }

    return MMI2C_RESULT_INVALID_ARGUMENT;
}

static bool release_bridge(ios_packet_sender_t *sender)
{
    uint32_t started_at =
        sender->i2c->config.now_us(sender->i2c->config.user);
    mmi2c_result_t result;
    do {
        mmi2c_process(sender->i2c);
        result = mmi2c_release_bridge(sender->i2c);
        if ((result == MMI2C_RESULT_BUSY) &&
            ((uint32_t) (sender->i2c->config.now_us(
                sender->i2c->config.user) - started_at) >=
                sender->i2c->config.bus_idle_timeout_us)) {
            result = MMI2C_RESULT_BUS_BUSY_TIMEOUT;
        }
    } while (result == MMI2C_RESULT_BUSY);

    if (result != MMI2C_RESULT_OK) {
        sender->last_i2c_result = result;
        sender->last_error = IOS_PACKET_ERROR_BRIDGE_RELEASE;
        return false;
    }
    return true;
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
    if (!release_bridge(sender)) {
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
        .module_id       = sender->module_id,
        .message_type    = sender->message_type,
        .sequence_number = sender->next_sequence_number,
        .payload_len     = 0u,
    };

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
    sender->last_i2c_result = MMI2C_RESULT_IN_PROGRESS;

    /* Write the packet into IOS LRAM, starting at register 0x50. */
    size_t bytes_sent = 0u;
    while (bytes_sent < tx_length) {
        size_t chunk_length = tx_length - bytes_sent;
        if (chunk_length > IOS_DATA_BYTES_PER_TRANSFER) {
            chunk_length = IOS_DATA_BYTES_PER_TRANSFER;
        }

        uint8_t frame[MMI2C_MAX_MESSAGE_BYTES];
        frame[0] = APOLLO3_IOS_WRITE_COMMAND_BIT |
                   (uint8_t) (APOLLO3_IOS_DATA_REGISTER + bytes_sent);
        (void) memcpy(&frame[1], &tx_data[bytes_sent], chunk_length);

        mmi2c_result_t result =
            write_i2c(sender, frame, (uint16_t) chunk_length + 1u);
        if (result != MMI2C_RESULT_OK) {
            return fail_i2c_and_release(sender);
        }
        bytes_sent += chunk_length;
    }

    /* All packet bytes succeeded. Tell the Apollo3 that the packet is ready. */
    const uint8_t trigger[] = {
        APOLLO3_IOS_WRITE_COMMAND_BIT | APOLLO3_IOS_TRIGGER_REGISTER,
        APOLLO3_IOS_TRIGGER_VALUE,
    };
    mmi2c_result_t result = write_i2c(sender, trigger, sizeof(trigger));
    if (result != MMI2C_RESULT_OK) {
        return fail_i2c_and_release(sender);
    }

    if (!release_bridge(sender)) {
        return fail(sender, IOS_PACKET_ERROR_BRIDGE_RELEASE);
    }

    sender->last_i2c_result = MMI2C_RESULT_OK;
    sender->last_error = IOS_PACKET_ERROR_NONE;
    ++sender->next_sequence_number;
    ++sender->packets_completed;
    return true;
}
