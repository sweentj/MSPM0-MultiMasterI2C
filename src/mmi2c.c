#include "mmi2c.h"

#include <string.h>

/*
 * State flow
 * ----------
 *
 * submit -> WAITING_FOR_BUS -> TRANSMITTING -> [RECEIVING] -> COMPLETE
 *                         \-> RECEIVING --------------------/
 *
 * Arbitration loss or a configured NACK retry moves the transfer to
 * WAITING_TO_RETRY. The main loop waits for the backoff time and an idle bus,
 * then restarts the whole transfer.
 *
 * Bus ownership is deliberately simple:
 *   - an external START or arbitration loss locks the bus;
 *   - STOP unlocks the bus; and
 *   - BUSBSY is always checked as a hardware safety net.
 *
 * The ISR handles exactly one pending event per invocation. If another event
 * remains pending, the peripheral asserts its IRQ again after the ISR returns.
 */

#define MMI2C_NORMAL_INTERRUPTS                                             \
    (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                              \
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |                       \
        DL_I2C_INTERRUPT_CONTROLLER_NACK |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_START |                                \
        DL_I2C_INTERRUPT_CONTROLLER_STOP |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

#define MMI2C_TX_FIFO_INTERRUPTS                                            \
    (DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |                          \
        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY)

static bool time_reached(uint32_t now, uint32_t target)
{
    return ((int32_t) (now - target) >= 0);
}

static uint32_t controller_status(const mmi2c_t *context)
{
    return DL_I2C_getControllerStatus(context->regs);
}

static bool hardware_bus_is_idle(const mmi2c_t *context)
{
    uint32_t status = controller_status(context);
    return ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0u) &&
           ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0u) &&
           ((status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) == 0u);
}

static bool bus_is_available(const mmi2c_t *context)
{
    return !context->bus_locked && hardware_bus_is_idle(context);
}

static bool controller_has_error(const mmi2c_t *context)
{
    uint32_t status = controller_status(context);
    return (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                         DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0u;
}

static bool transfer_is_active(const mmi2c_t *context)
{
    return (context->state == MMI2C_STATE_TRANSMITTING) ||
           (context->state == MMI2C_STATE_RECEIVING) ||
           (context->state == MMI2C_STATE_RECEIVE_SETTLE);
}

static uint32_t next_random(mmi2c_t *context)
{
    uint32_t value = context->prng;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    context->prng = (value == 0u) ? 0xA341316Cu : value;
    return context->prng;
}

/* Read at most one byte. This is the only RX helper called by the ISR. */
static void receive_one_byte(mmi2c_t *context)
{
    if (DL_I2C_isControllerRXFIFOEmpty(context->regs)) {
        return;
    }

    uint8_t data = DL_I2C_receiveControllerData(context->regs);
    if (context->rx_count < context->transfer.rx_length) {
        context->transfer.rx_data[context->rx_count] = data;
        ++context->rx_count;
    }
}

/* This loop runs only from mmi2c_process(), never from the ISR. */
static void drain_rx_fifo_from_main(mmi2c_t *context)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(context->regs)) {
        receive_one_byte(context);
    }
}

static void release_bridge_automatically(mmi2c_t *context)
{
    if (!context->bridge_acquired ||
        !context->config.release_bridge_when_done ||
        (context->config.bridge_release == NULL) ||
        !bus_is_available(context)) {
        return;
    }

    context->config.bridge_release(context->config.user);
    context->bridge_acquired = false;
}

static void complete_transfer(mmi2c_t *context, mmi2c_result_t result)
{
    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS);
    context->result = result;
    context->state  = MMI2C_STATE_COMPLETE;
    release_bridge_automatically(context);
}

static void start_receive(mmi2c_t *context)
{
    DL_I2C_flushControllerRXFIFO(context->regs);
    context->rx_count = 0u;
    context->state    = MMI2C_STATE_RECEIVING;
    context->deadline_us = context->config.now_us(context->config.user) +
                           context->config.transfer_timeout_us;

    DL_I2C_startControllerTransferAdvanced(context->regs,
        context->transfer.address, DL_I2C_CONTROLLER_DIRECTION_RX,
        context->transfer.rx_length, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void start_transfer(mmi2c_t *context)
{
    context->tx_count = 0u;
    context->rx_count = 0u;
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);
    DL_I2C_clearInterruptStatus(context->regs, MMI2C_TX_FIFO_INTERRUPTS);

    if (context->transfer.tx_length == 0u) {
        start_receive(context);
        return;
    }

    context->tx_count = DL_I2C_fillControllerTXFIFO(context->regs,
        context->transfer.tx_data, context->transfer.tx_length);

    if (context->tx_count < context->transfer.tx_length) {
        DL_I2C_enableInterrupt(context->regs,
            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    } else {
        DL_I2C_disableInterrupt(context->regs,
            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    }

    context->state = MMI2C_STATE_TRANSMITTING;
    context->deadline_us = context->config.now_us(context->config.user) +
                           context->config.transfer_timeout_us;

    DL_I2C_startControllerTransferAdvanced(context->regs,
        context->transfer.address, DL_I2C_CONTROLLER_DIRECTION_TX,
        context->transfer.tx_length, DL_I2C_CONTROLLER_START_ENABLE,
        (context->transfer.rx_length == 0u) ? DL_I2C_CONTROLLER_STOP_ENABLE
                                            : DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void retry_or_complete(mmi2c_t *context, mmi2c_result_t reason)
{
    bool arbitration_lost = (reason == MMI2C_RESULT_ARBITRATION_LOST);
    uint8_t retry_count = arbitration_lost ? context->arbitration_retries
                                           : context->nack_retries;
    uint8_t retry_limit = arbitration_lost
                              ? context->config.max_arbitration_retries
                              : context->config.max_nack_retries;

    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS);
    DL_I2C_clearInterruptStatus(context->regs, MMI2C_TX_FIFO_INTERRUPTS);
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);

    if (retry_count >= retry_limit) {
        complete_transfer(context, reason);
        return;
    }

    ++retry_count;
    if (arbitration_lost) {
        context->arbitration_retries = retry_count;
    } else {
        context->nack_retries = retry_count;
    }

    uint8_t exponent = (retry_count > 8u) ? 8u : retry_count;
    uint32_t slot_count = (1u << exponent);
    uint32_t delay_us =
        (next_random(context) % slot_count) * context->config.backoff_slot_us;

    context->retry_at_us = context->config.now_us(context->config.user) +
                           delay_us;
    context->deadline_us = context->retry_at_us +
                           context->config.bus_idle_timeout_us;
    context->state = MMI2C_STATE_WAITING_TO_RETRY;
}

static bool config_is_valid(
    I2C_Regs *regs, const mmi2c_config_t *config)
{
    if ((regs == NULL) || (config == NULL) || (config->now_us == NULL) ||
        (config->bus_idle_timeout_us == 0u) ||
        (config->transfer_timeout_us == 0u) ||
        (config->rx_done_settle_us == 0u)) {
        return false;
    }
    if ((config->bridge_acquire != NULL) &&
        (config->bridge_ready_timeout_us == 0u)) {
        return false;
    }
    if (config->release_bridge_when_done &&
        (config->bridge_release == NULL)) {
        return false;
    }
    if (((config->max_arbitration_retries != 0u) ||
            (config->max_nack_retries != 0u)) &&
        ((config->backoff_slot_us == 0u) || (config->random_seed == 0u))) {
        return false;
    }
    return true;
}

static bool transfer_is_valid(const mmi2c_transfer_t *transfer)
{
    if ((transfer == NULL) || (transfer->address > 0x7Fu)) {
        return false;
    }
    if ((transfer->tx_length == 0u) && (transfer->rx_length == 0u)) {
        return false;
    }
    if (((uint32_t) transfer->tx_length + transfer->rx_length) >
        MMI2C_MAX_MESSAGE_BYTES) {
        return false;
    }
    if ((transfer->tx_length != 0u) && (transfer->tx_data == NULL)) {
        return false;
    }
    if ((transfer->rx_length != 0u) && (transfer->rx_data == NULL)) {
        return false;
    }
    return true;
}

mmi2c_result_t mmi2c_init(
    mmi2c_t *context, I2C_Regs *regs, const mmi2c_config_t *config)
{
    if ((context == NULL) || !config_is_valid(regs, config)) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }

    (void) memset(context, 0, sizeof(*context));
    context->regs   = regs;
    context->config = *config;
    context->prng   = config->random_seed;
    context->state  = MMI2C_STATE_IDLE;
    context->result = MMI2C_RESULT_NONE;

    DL_I2C_enableMultiControllerMode(regs);
    DL_I2C_enableInterrupt(regs, MMI2C_NORMAL_INTERRUPTS);
    DL_I2C_disableInterrupt(regs, MMI2C_TX_FIFO_INTERRUPTS);
    return MMI2C_RESULT_OK;
}

mmi2c_result_t mmi2c_submit(
    mmi2c_t *context, const mmi2c_transfer_t *transfer)
{
    if ((context == NULL) ||
        (context->state == MMI2C_STATE_UNINITIALIZED) ||
        !transfer_is_valid(transfer)) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }
    if (context->state != MMI2C_STATE_IDLE) {
        return MMI2C_RESULT_BUSY;
    }

    context->transfer            = *transfer;
    context->result              = MMI2C_RESULT_IN_PROGRESS;
    context->arbitration_retries = 0u;
    context->nack_retries        = 0u;
    context->recovery_attempted  = false;

    if (!context->bridge_acquired &&
        (context->config.bridge_acquire != NULL)) {
        bool ready = context->config.bridge_acquire(context->config.user,
            context->config.bridge_ready_timeout_us);
        if (!ready) {
            complete_transfer(context, MMI2C_RESULT_BRIDGE_NOT_READY);
            return MMI2C_RESULT_BRIDGE_NOT_READY;
        }
        context->bridge_acquired = true;
    }

    context->deadline_us = context->config.now_us(context->config.user) +
                           context->config.bus_idle_timeout_us;
    context->state = MMI2C_STATE_WAITING_FOR_BUS;

    /* Start immediately if no other controller owns the bus. */
    mmi2c_process(context);
    return context->result;
}

static void handle_transfer_timeout(mmi2c_t *context)
{
    DL_I2C_resetControllerTransfer(context->regs);
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);
    complete_transfer(context, MMI2C_RESULT_TRANSFER_TIMEOUT);
}

static void process_bus_wait(mmi2c_t *context, uint32_t now)
{
    bool backoff_finished =
        (context->state == MMI2C_STATE_WAITING_FOR_BUS) ||
        time_reached(now, context->retry_at_us);

    if (backoff_finished && bus_is_available(context)) {
        start_transfer(context);
        return;
    }

    if (!time_reached(now, context->deadline_us)) {
        return;
    }

    if (!context->recovery_attempted &&
        (context->config.bus_recover != NULL)) {
        context->recovery_attempted = true;
        if (context->config.bus_recover(context->config.user)) {
            context->bus_locked = false;
            context->deadline_us = now + context->config.bus_idle_timeout_us;
            return;
        }
    }

    complete_transfer(context, MMI2C_RESULT_BUS_BUSY_TIMEOUT);
}

void mmi2c_process(mmi2c_t *context)
{
    if ((context == NULL) || (context->config.now_us == NULL)) {
        return;
    }

    uint32_t now = context->config.now_us(context->config.user);

    switch (context->state) {
        case MMI2C_STATE_IDLE:
        case MMI2C_STATE_COMPLETE:
            release_bridge_automatically(context);
            break;

        case MMI2C_STATE_WAITING_FOR_BUS:
        case MMI2C_STATE_WAITING_TO_RETRY:
            process_bus_wait(context, now);
            break;

        case MMI2C_STATE_TRANSMITTING:
        case MMI2C_STATE_RECEIVING:
            if (time_reached(now, context->deadline_us)) {
                handle_transfer_timeout(context);
            }
            break;

        case MMI2C_STATE_RECEIVE_SETTLE:
            if (time_reached(now, context->retry_at_us)) {
                drain_rx_fifo_from_main(context);
                complete_transfer(context,
                    (context->rx_count == context->transfer.rx_length)
                        ? MMI2C_RESULT_OK
                        : MMI2C_RESULT_SHORT_READ);
            } else if (time_reached(now, context->deadline_us)) {
                handle_transfer_timeout(context);
            }
            break;

        case MMI2C_STATE_UNINITIALIZED:
        default:
            break;
    }
}

static void handle_tx_fifo_request(mmi2c_t *context)
{
    if (context->state != MMI2C_STATE_TRANSMITTING) {
        return;
    }

    uint16_t remaining = context->transfer.tx_length - context->tx_count;
    context->tx_count += DL_I2C_fillControllerTXFIFO(context->regs,
        &context->transfer.tx_data[context->tx_count], remaining);

    if (context->tx_count == context->transfer.tx_length) {
        DL_I2C_disableInterrupt(context->regs,
            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    }
}

static void handle_tx_done(mmi2c_t *context)
{
    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS);

    if ((context->state != MMI2C_STATE_TRANSMITTING) ||
        controller_has_error(context)) {
        /* NACK or arbitration loss has its own pending interrupt. */
        return;
    }

    if (context->transfer.rx_length != 0u) {
        start_receive(context);
    } else {
        complete_transfer(context, MMI2C_RESULT_OK);
    }
}

static void handle_rx_done(mmi2c_t *context)
{
    if ((context->state != MMI2C_STATE_RECEIVING) ||
        controller_has_error(context)) {
        return;
    }

    /* I2C_ERR_08: wait at least two functional clocks before the final read. */
    context->retry_at_us = context->config.now_us(context->config.user) +
                           context->config.rx_done_settle_us;
    context->state = MMI2C_STATE_RECEIVE_SETTLE;
}

static void handle_start(mmi2c_t *context)
{
    bool this_controller_started =
        (context->state == MMI2C_STATE_TRANSMITTING) ||
        (context->state == MMI2C_STATE_RECEIVING);

    if (!this_controller_started) {
        context->bus_locked = true;
    }
}

static void handle_stop(mmi2c_t *context)
{
    /* BUSBSY can already be set again if another START immediately followed
     * this STOP. Preserve the lock in that case. */
    context->bus_locked =
        (controller_status(context) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u;
}

static void handle_arbitration_lost(mmi2c_t *context)
{
    if (!transfer_is_active(context)) {
        return;
    }

    uint32_t status = controller_status(context);
    context->bus_locked =
        (status & (DL_I2C_CONTROLLER_STATUS_BUSY_BUS |
                      DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0u;

    /* Never generate STOP here; the winning controller owns the bus. */
    retry_or_complete(context, MMI2C_RESULT_ARBITRATION_LOST);
}

static void handle_nack(mmi2c_t *context)
{
    if (!transfer_is_active(context)) {
        return;
    }

    /* A combined TX/RX transfer disabled STOP for its TX phase. Re-enable it
     * so a NACK always releases the transaction before retry/completion. */
    DL_I2C_enableStopCondition(context->regs);
    retry_or_complete(context, MMI2C_RESULT_NACK);
}

void mmi2c_handle_irq(mmi2c_t *context)
{
    if ((context == NULL) || (context->regs == NULL)) {
        return;
    }

    /* Handle one event only. Any remaining event retriggers the IRQ. */
    DL_I2C_IIDX source = DL_I2C_getPendingInterrupt(context->regs);

    switch (source) {
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            handle_tx_fifo_request(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
            if ((context->state == MMI2C_STATE_RECEIVING) ||
                (context->state == MMI2C_STATE_RECEIVE_SETTLE)) {
                receive_one_byte(context);
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            handle_tx_done(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            handle_rx_done(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_START:
            handle_start(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_STOP:
            handle_stop(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
            handle_arbitration_lost(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_NACK:
            handle_nack(context);
            break;

        case DL_I2C_IIDX_NO_INT:
        default:
            break;
    }
}

mmi2c_result_t mmi2c_get_result(const mmi2c_t *context)
{
    return (context == NULL) ? MMI2C_RESULT_INVALID_ARGUMENT : context->result;
}

mmi2c_result_t mmi2c_take_result(mmi2c_t *context)
{
    if (context == NULL) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }
    if (context->state == MMI2C_STATE_COMPLETE) {
        mmi2c_result_t result = context->result;
        context->result       = MMI2C_RESULT_NONE;
        context->state        = MMI2C_STATE_IDLE;
        return result;
    }
    return (context->state == MMI2C_STATE_IDLE) ? MMI2C_RESULT_NONE
                                                : MMI2C_RESULT_IN_PROGRESS;
}

bool mmi2c_is_busy(const mmi2c_t *context)
{
    return (context != NULL) && (context->state != MMI2C_STATE_IDLE) &&
           (context->state != MMI2C_STATE_COMPLETE);
}

mmi2c_result_t mmi2c_release_bridge(mmi2c_t *context)
{
    if ((context == NULL) || (context->regs == NULL) ||
        (context->state == MMI2C_STATE_UNINITIALIZED)) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }
    if (mmi2c_is_busy(context)) {
        return MMI2C_RESULT_BUSY;
    }
    if (!context->bridge_acquired) {
        return MMI2C_RESULT_OK;
    }
    if (!bus_is_available(context)) {
        return MMI2C_RESULT_BUSY;
    }

    if (context->config.bridge_release != NULL) {
        context->config.bridge_release(context->config.user);
    }
    context->bridge_acquired = false;
    return MMI2C_RESULT_OK;
}
