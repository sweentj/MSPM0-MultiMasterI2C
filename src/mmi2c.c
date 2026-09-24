#include "mmi2c.h"

#include <string.h>

#define MMI2C_BASE_INTERRUPTS                                                \
    (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                                  \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                               \
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |                        \
        DL_I2C_INTERRUPT_CONTROLLER_NACK |                                  \
        DL_I2C_INTERRUPT_CONTROLLER_STOP |                                  \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

#define MMI2C_TX_INTERRUPTS                                                  \
    (DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |                           \
        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY)

static bool time_reached(uint32_t now, uint32_t target)
{
    return ((int32_t) (now - target) >= 0);
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

static void drain_rx_fifo(mmi2c_t *context)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(context->regs)) {
        uint8_t data = DL_I2C_receiveControllerData(context->regs);
        if (context->rx_count < context->transfer.rx_length) {
            context->transfer.rx_data[context->rx_count++] = data;
        }
    }
}

static void release_bridge(mmi2c_t *context)
{
    if (context->bridge_acquired && context->config.release_bridge_when_done &&
        (context->config.bridge_release != NULL)) {
        uint32_t status = DL_I2C_getControllerStatus(context->regs);
        /* Do not disconnect before the STOP has physically released the bus. */
        if (((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u) ||
            ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0u)) {
            return;
        }
        context->config.bridge_release(context->config.user);
        context->bridge_acquired = false;
    }
}

static void finish(mmi2c_t *context, mmi2c_result_t result)
{
    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_INTERRUPTS);
    context->result = result;
    context->state  = MMI2C_STATE_COMPLETE;
    release_bridge(context);
}

static void start_rx(mmi2c_t *context)
{
    DL_I2C_flushControllerRXFIFO(context->regs);
    context->rx_count = 0u;
    context->state    = MMI2C_STATE_RECEIVING;
    context->deadline_us =
        context->config.now_us(context->config.user) +
        context->config.transfer_timeout_us;

    DL_I2C_startControllerTransferAdvanced(context->regs,
        context->transfer.address, DL_I2C_CONTROLLER_DIRECTION_RX,
        context->transfer.rx_length, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void start_attempt(mmi2c_t *context)
{
    context->tx_count = 0u;
    context->rx_count = 0u;
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);
    DL_I2C_clearInterruptStatus(context->regs, MMI2C_TX_INTERRUPTS);

    if (context->transfer.tx_length == 0u) {
        start_rx(context);
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
    context->deadline_us =
        context->config.now_us(context->config.user) +
        context->config.transfer_timeout_us;

    DL_I2C_startControllerTransferAdvanced(context->regs,
        context->transfer.address, DL_I2C_CONTROLLER_DIRECTION_TX,
        context->transfer.tx_length, DL_I2C_CONTROLLER_START_ENABLE,
        (context->transfer.rx_length == 0u) ? DL_I2C_CONTROLLER_STOP_ENABLE
                                            : DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void schedule_retry(mmi2c_t *context, bool arbitration_lost)
{
    uint8_t retry_count;
    uint8_t retry_limit;

    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_INTERRUPTS);
    DL_I2C_clearInterruptStatus(context->regs, MMI2C_TX_INTERRUPTS);
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);

    if (arbitration_lost) {
        retry_count = context->arbitration_retries;
        retry_limit = context->config.max_arbitration_retries;
    } else {
        retry_count = context->nack_retries;
        retry_limit = context->config.max_nack_retries;
    }

    if (retry_count >= retry_limit) {
        finish(context, arbitration_lost ? MMI2C_RESULT_ARBITRATION_LOST
                                         : MMI2C_RESULT_NACK);
        return;
    }

    ++retry_count;
    if (arbitration_lost) {
        context->arbitration_retries = retry_count;
    } else {
        context->nack_retries = retry_count;
    }
    uint8_t exponent = retry_count;
    if (exponent > 8u) {
        exponent = 8u;
    }
    uint32_t slots = (1u << exponent) - 1u;
    uint32_t delay = (next_random(context) % (slots + 1u)) *
                     (uint32_t) context->config.backoff_slot_us;

    context->retry_at_us = context->config.now_us(context->config.user) + delay;
    /* The idle timeout begins after backoff, not before it. Otherwise a later
     * exponential-backoff attempt could time out before it is eligible. */
    context->deadline_us =
        context->retry_at_us + context->config.bus_idle_timeout_us;
    context->state = MMI2C_STATE_WAITING_TO_RETRY;
}

mmi2c_result_t mmi2c_init(
    mmi2c_t *context, I2C_Regs *regs, const mmi2c_config_t *config)
{
    if ((context == NULL) || (regs == NULL) || (config == NULL) ||
        (config->now_us == NULL) || (config->bus_idle_timeout_us == 0u) ||
        (config->transfer_timeout_us == 0u) ||
        (config->rx_done_settle_us == 0u) ||
        ((config->bridge_acquire != NULL) &&
            (config->bridge_ready_timeout_us == 0u)) ||
        (config->release_bridge_when_done &&
            (config->bridge_release == NULL)) ||
        ((config->max_arbitration_retries != 0u) &&
            ((config->backoff_slot_us == 0u) ||
                (config->random_seed == 0u)))) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }

    (void) memset(context, 0, sizeof(*context));
    context->regs   = regs;
    context->config = *config;
    context->prng   = config->random_seed;
    context->state  = MMI2C_STATE_IDLE;
    context->result = MMI2C_RESULT_NONE;

    DL_I2C_enableMultiControllerMode(regs);
    DL_I2C_enableInterrupt(regs, MMI2C_BASE_INTERRUPTS);
    DL_I2C_disableInterrupt(regs, MMI2C_TX_INTERRUPTS);
    return MMI2C_RESULT_OK;
}

mmi2c_result_t mmi2c_submit(
    mmi2c_t *context, const mmi2c_transfer_t *transfer)
{
    if ((context == NULL) || (transfer == NULL) ||
        (context->state == MMI2C_STATE_UNINITIALIZED) ||
        (transfer->address > 0x7Fu) ||
        ((transfer->tx_length == 0u) && (transfer->rx_length == 0u)) ||
        (transfer->tx_length > MMI2C_MAX_MESSAGE_BYTES) ||
        (transfer->rx_length > MMI2C_MAX_MESSAGE_BYTES) ||
        (((uint32_t) transfer->tx_length + transfer->rx_length) >
            MMI2C_MAX_MESSAGE_BYTES) ||
        ((transfer->tx_length != 0u) && (transfer->tx_data == NULL)) ||
        ((transfer->rx_length != 0u) && (transfer->rx_data == NULL))) {
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
        if (!context->config.bridge_acquire(context->config.user,
                context->config.bridge_ready_timeout_us)) {
            finish(context, MMI2C_RESULT_BRIDGE_NOT_READY);
            return MMI2C_RESULT_BRIDGE_NOT_READY;
        }
        context->bridge_acquired = true;
    }

    context->deadline_us = context->config.now_us(context->config.user) +
                           context->config.bus_idle_timeout_us;
    context->state = MMI2C_STATE_WAITING_FOR_BUS;
    mmi2c_process(context);
    return context->result;
}

void mmi2c_process(mmi2c_t *context)
{
    if ((context == NULL) || (context->config.now_us == NULL)) {
        return;
    }

    uint32_t now    = context->config.now_us(context->config.user);
    uint32_t status = DL_I2C_getControllerStatus(context->regs);

    if ((context->state == MMI2C_STATE_IDLE) ||
        (context->state == MMI2C_STATE_COMPLETE)) {
        release_bridge(context);
        return;
    }

    if (context->state == MMI2C_STATE_RECEIVE_SETTLE) {
        if (time_reached(now, context->retry_at_us)) {
            drain_rx_fifo(context);
            finish(context,
                (context->rx_count == context->transfer.rx_length)
                    ? MMI2C_RESULT_OK
                    : MMI2C_RESULT_SHORT_READ);
        } else if (time_reached(now, context->deadline_us)) {
            finish(context, MMI2C_RESULT_TRANSFER_TIMEOUT);
        }
        return;
    }

    if ((context->state == MMI2C_STATE_TRANSMITTING) ||
        (context->state == MMI2C_STATE_RECEIVING)) {
        if (time_reached(now, context->deadline_us)) {
            DL_I2C_resetControllerTransfer(context->regs);
            DL_I2C_flushControllerTXFIFO(context->regs);
            DL_I2C_flushControllerRXFIFO(context->regs);
            finish(context, MMI2C_RESULT_TRANSFER_TIMEOUT);
        }
        return;
    }

    if ((context->state != MMI2C_STATE_WAITING_FOR_BUS) &&
        (context->state != MMI2C_STATE_WAITING_TO_RETRY)) {
        return;
    }

    bool bus_idle = ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0u) &&
                    ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0u) &&
                    ((status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) == 0u);
    bool backoff_done = (context->state == MMI2C_STATE_WAITING_FOR_BUS) ||
                        time_reached(now, context->retry_at_us);

    if (bus_idle && backoff_done) {
        start_attempt(context);
        return;
    }

    if (time_reached(now, context->deadline_us)) {
        if (!context->recovery_attempted &&
            (context->config.bus_recover != NULL)) {
            context->recovery_attempted = true;
            if (context->config.bus_recover(context->config.user)) {
                context->deadline_us = now + context->config.bus_idle_timeout_us;
                return;
            }
        }
        finish(context, MMI2C_RESULT_BUS_BUSY_TIMEOUT);
    }
}

void mmi2c_handle_irq(mmi2c_t *context)
{
    if ((context == NULL) || (context->regs == NULL)) {
        return;
    }

    DL_I2C_IIDX source;
    do {
        source = DL_I2C_getPendingInterrupt(context->regs);
        switch (source) {
            case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
                if (context->state == MMI2C_STATE_TRANSMITTING) {
                    uint16_t remaining =
                        context->transfer.tx_length - context->tx_count;
                    context->tx_count += DL_I2C_fillControllerTXFIFO(
                        context->regs,
                        &context->transfer.tx_data[context->tx_count], remaining);
                    if (context->tx_count == context->transfer.tx_length) {
                        DL_I2C_disableInterrupt(context->regs,
                            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
                    }
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
                if ((context->state == MMI2C_STATE_RECEIVING) ||
                    (context->state == MMI2C_STATE_RECEIVE_SETTLE)) {
                    drain_rx_fifo(context);
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_TX_DONE:
                DL_I2C_disableInterrupt(context->regs, MMI2C_TX_INTERRUPTS);
                if (context->state == MMI2C_STATE_TRANSMITTING) {
                    uint32_t status =
                        DL_I2C_getControllerStatus(context->regs);
                    /* DONE has higher IIDX priority than NACK/ARBLST. Leave
                     * the state active so the pending error is handled next. */
                    if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                                      DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) !=
                        0u) {
                        break;
                    } else if (context->transfer.rx_length != 0u) {
                        start_rx(context);
                    } else {
                        finish(context, MMI2C_RESULT_OK);
                    }
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_RX_DONE:
                if (context->state == MMI2C_STATE_RECEIVING) {
                    uint32_t status =
                        DL_I2C_getControllerStatus(context->regs);
                    if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                                      DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) !=
                        0u) {
                        break;
                    }
                    /* I2C_ERR_08: the last FIFO byte may not be visible at
                     * RXDONE. Defer the final read by >= 2 functional clocks. */
                    context->retry_at_us =
                        context->config.now_us(context->config.user) +
                        context->config.rx_done_settle_us;
                    context->state = MMI2C_STATE_RECEIVE_SETTLE;
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
                if ((context->state == MMI2C_STATE_TRANSMITTING) ||
                    (context->state == MMI2C_STATE_RECEIVING) ||
                    (context->state == MMI2C_STATE_RECEIVE_SETTLE)) {
                    /* Do not generate STOP: the winning controller owns bus. */
                    schedule_retry(context, true);
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_NACK:
                if ((context->state == MMI2C_STATE_TRANSMITTING) ||
                    (context->state == MMI2C_STATE_RECEIVING) ||
                    (context->state == MMI2C_STATE_RECEIVE_SETTLE)) {
                    /* Unlike arbitration loss, this controller still owns the
                     * transaction. Ensure a combined transfer whose TX phase
                     * disabled STOP releases the bus after a NACK. */
                    DL_I2C_enableStopCondition(context->regs);
                    schedule_retry(context, false);
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_STOP:
                /* ARBLST is cleared by hardware when this STOP is observed. */
                break;

            case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
            case DL_I2C_IIDX_CONTROLLER_START:
            case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
            case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
            case DL_I2C_IIDX_CONTROLLER_PEC_RX_ERROR:
            case DL_I2C_IIDX_TIMEOUT_A:
            case DL_I2C_IIDX_TIMEOUT_B:
            case DL_I2C_IIDX_TARGET_RX_DONE:
            case DL_I2C_IIDX_TARGET_TX_DONE:
            case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
            case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
            case DL_I2C_IIDX_TARGET_START:
            case DL_I2C_IIDX_TARGET_STOP:
            case DL_I2C_IIDX_TARGET_GENERAL_CALL:
            case DL_I2C_IIDX_TARGET_EVENT1_DMA_DONE:
            case DL_I2C_IIDX_TARGET_EVENT2_DMA_DONE:
            case DL_I2C_IIDX_TARGET_PEC_RX_ERROR:
            case DL_I2C_IIDX_TARGET_TXFIFO_UNDERFLOW:
            case DL_I2C_IIDX_TARGET_RXFIFO_OVERFLOW:
            case DL_I2C_IIDX_TARGET_ARBITRATION_LOST:
            case DL_I2C_IIDX_INTERRUPT_OVERFLOW:
            case DL_I2C_IIDX_NO_INT:
            default:
                break;
        }
    } while (source != DL_I2C_IIDX_NO_INT);
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
