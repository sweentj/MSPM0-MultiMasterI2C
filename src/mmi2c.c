#include "mmi2c.h"

#include <string.h>

#define MMI2C_NORMAL_INTERRUPTS                                             \
    (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                              \
        DL_I2C_INTERRUPT_CONTROLLER_NACK |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_START |                                \
        DL_I2C_INTERRUPT_CONTROLLER_STOP |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

#define MMI2C_TX_FIFO_INTERRUPTS                                            \
    (DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |                          \
        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY)

static uint32_t controller_status(const mmi2c_t *context)
{
    return DL_I2C_getControllerStatus(context->regs);
}

static bool bus_is_available(const mmi2c_t *context)
{
    uint32_t status = controller_status(context);
    return !context->bus_locked &&
           ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0u) &&
           ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0u) &&
           ((status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) == 0u);
}

static bool controller_has_error(const mmi2c_t *context)
{
    uint32_t status = controller_status(context);
    return (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                         DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0u;
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

static bool config_is_valid(
    I2C_Regs *regs, const mmi2c_config_t *config)
{
    if ((regs == NULL) || (config == NULL) ||
        (config->mutex_timeout_ticks == 0u) ||
        (config->bus_timeout_ticks == 0u) ||
        (config->transfer_timeout_ticks == 0u) ||
        (config->rx_settle_ticks == 0u)) {
        return false;
    }
    if ((config->bridge_acquire == NULL) !=
        (config->bridge_release == NULL)) {
        return false;
    }
    if (((config->max_arbitration_retries != 0u) ||
            (config->max_nack_retries != 0u)) &&
        ((config->backoff_slot_ticks == 0u) ||
            (config->random_seed == 0u))) {
        return false;
    }
    return true;
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

static void clear_task_notification(void)
{
    (void) ulTaskNotifyTake(pdTRUE, 0u);
}

static void set_waiting_task(mmi2c_t *context)
{
    taskENTER_CRITICAL();
    context->waiting_task = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
    clear_task_notification();
}

static void clear_waiting_task(mmi2c_t *context)
{
    taskENTER_CRITICAL();
    context->waiting_task = NULL;
    taskEXIT_CRITICAL();
}

static void notify_waiting_task_from_isr(
    mmi2c_t *context, BaseType_t *higher_priority_task_woken)
{
    TaskHandle_t task = context->waiting_task;
    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, higher_priority_task_woken);
    }
}

static bool wait_until_bus_is_idle(mmi2c_t *context)
{
    TimeOut_t timeout;
    TickType_t remaining = context->config.bus_timeout_ticks;

    clear_task_notification();
    vTaskSetTimeOutState(&timeout);

    for (;;) {
        if (bus_is_available(context)) {
            return true;
        }

        (void) ulTaskNotifyTake(pdTRUE, remaining);
        if (bus_is_available(context)) {
            return true;
        }
        if (xTaskCheckForTimeOut(&timeout, &remaining) != pdFALSE) {
            return false;
        }
    }
}

static mmi2c_result_t wait_for_bus(mmi2c_t *context)
{
    if (wait_until_bus_is_idle(context)) {
        return MMI2C_RESULT_OK;
    }

    if ((context->config.bus_recover != NULL) &&
        context->config.bus_recover(context->config.user)) {
        context->bus_locked = false;
        if (wait_until_bus_is_idle(context)) {
            return MMI2C_RESULT_OK;
        }
    }

    return MMI2C_RESULT_BUS_BUSY_TIMEOUT;
}

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

/* This is called only from task context. */
static void drain_rx_fifo(mmi2c_t *context)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(context->regs)) {
        receive_one_byte(context);
    }
}

static void start_receive(mmi2c_t *context)
{
    DL_I2C_flushControllerRXFIFO(context->regs);
    DL_I2C_enableInterrupt(context->regs,
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    context->rx_count = 0u;
    context->state = MMI2C_STATE_RECEIVING;

    DL_I2C_startControllerTransferAdvanced(context->regs,
        context->transfer.address, DL_I2C_CONTROLLER_DIRECTION_RX,
        context->transfer.rx_length, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void start_transfer(mmi2c_t *context)
{
    context->tx_count = 0u;
    context->rx_count = 0u;
    context->result = MMI2C_RESULT_BUSY;
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
    }

    context->state = MMI2C_STATE_TRANSMITTING;
    DL_I2C_startControllerTransferAdvanced(context->regs,
        context->transfer.address, DL_I2C_CONTROLLER_DIRECTION_TX,
        context->transfer.tx_length, DL_I2C_CONTROLLER_START_ENABLE,
        (context->transfer.rx_length == 0u) ? DL_I2C_CONTROLLER_STOP_ENABLE
                                            : DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void abort_transfer(mmi2c_t *context)
{
    taskENTER_CRITICAL();
    DL_I2C_resetControllerTransfer(context->regs);
    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS |
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);
    context->state = MMI2C_STATE_IDLE;
    context->result = MMI2C_RESULT_TRANSFER_TIMEOUT;
    taskEXIT_CRITICAL();
}

static mmi2c_result_t wait_for_transfer(mmi2c_t *context)
{
    TimeOut_t timeout;
    TickType_t remaining = context->config.transfer_timeout_ticks;

    vTaskSetTimeOutState(&timeout);
    for (;;) {
        if (context->state == MMI2C_STATE_RECEIVE_SETTLE) {
            vTaskDelay(context->config.rx_settle_ticks);
            if (context->result != MMI2C_RESULT_BUSY) {
                return context->result;
            }

            drain_rx_fifo(context);
            context->state = MMI2C_STATE_IDLE;
            context->result =
                (context->rx_count == context->transfer.rx_length)
                    ? MMI2C_RESULT_OK
                    : MMI2C_RESULT_SHORT_READ;
            return context->result;
        }

        if (context->result != MMI2C_RESULT_BUSY) {
            return context->result;
        }

        (void) ulTaskNotifyTake(pdTRUE, remaining);
        if (context->result != MMI2C_RESULT_BUSY) {
            return context->result;
        }
        if (xTaskCheckForTimeOut(&timeout, &remaining) != pdFALSE) {
            abort_transfer(context);
            return MMI2C_RESULT_TRANSFER_TIMEOUT;
        }
    }
}

static void backoff(mmi2c_t *context, uint8_t retry_number)
{
    uint8_t exponent = (retry_number > 8u) ? 8u : retry_number;
    uint32_t window = 1u << exponent;
    TickType_t slots = (TickType_t) (next_random(context) % window);
    TickType_t delay = slots * context->config.backoff_slot_ticks;
    if (delay != 0u) {
        vTaskDelay(delay);
    }
}

static mmi2c_result_t acquire_bridge(mmi2c_t *context)
{
    if (context->bridge_acquired ||
        (context->config.bridge_acquire == NULL)) {
        return MMI2C_RESULT_OK;
    }

    if (!context->config.bridge_acquire(context->config.user)) {
        return MMI2C_RESULT_BRIDGE_NOT_READY;
    }
    context->bridge_acquired = true;
    return MMI2C_RESULT_OK;
}

static mmi2c_result_t release_bridge_locked(mmi2c_t *context)
{
    if (!context->bridge_acquired) {
        return MMI2C_RESULT_OK;
    }

    mmi2c_result_t result = wait_for_bus(context);
    if (result != MMI2C_RESULT_OK) {
        return result;
    }

    context->config.bridge_release(context->config.user);
    context->bridge_acquired = false;
    return MMI2C_RESULT_OK;
}

mmi2c_result_t mmi2c_init(
    mmi2c_t *context, I2C_Regs *regs, const mmi2c_config_t *config)
{
    if ((context == NULL) || !config_is_valid(regs, config)) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }

    (void) memset(context, 0, sizeof(*context));
    context->regs = regs;
    context->config = *config;
    context->prng = config->random_seed;
    context->state = MMI2C_STATE_IDLE;
    context->result = MMI2C_RESULT_OK;
    context->mutex = xSemaphoreCreateMutexStatic(&context->mutex_storage);
    if (context->mutex == NULL) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }

    DL_I2C_enableMultiControllerMode(regs);
    DL_I2C_enableInterrupt(regs, MMI2C_NORMAL_INTERRUPTS);
    DL_I2C_disableInterrupt(regs, MMI2C_TX_FIFO_INTERRUPTS);
    return MMI2C_RESULT_OK;
}

mmi2c_result_t mmi2c_transfer(
    mmi2c_t *context, const mmi2c_transfer_t *transfer)
{
    if ((context == NULL) || (context->mutex == NULL) ||
        !transfer_is_valid(transfer)) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }
    if (xSemaphoreTake(context->mutex,
            context->config.mutex_timeout_ticks) != pdTRUE) {
        return MMI2C_RESULT_BUSY;
    }

    set_waiting_task(context);
    mmi2c_result_t result = acquire_bridge(context);
    uint8_t arbitration_retries = 0u;
    uint8_t nack_retries = 0u;

    while (result == MMI2C_RESULT_OK) {
        result = wait_for_bus(context);
        if (result != MMI2C_RESULT_OK) {
            break;
        }

        clear_task_notification();
        context->transfer = *transfer;
        start_transfer(context);
        result = wait_for_transfer(context);

        if ((result == MMI2C_RESULT_ARBITRATION_LOST) &&
            (arbitration_retries <
                context->config.max_arbitration_retries)) {
            ++arbitration_retries;
            result = wait_for_bus(context);
            if (result == MMI2C_RESULT_OK) {
                backoff(context, arbitration_retries);
            }
            continue;
        }

        if ((result == MMI2C_RESULT_NACK) &&
            (nack_retries < context->config.max_nack_retries)) {
            ++nack_retries;
            result = wait_for_bus(context);
            if (result == MMI2C_RESULT_OK) {
                backoff(context, nack_retries);
            }
            continue;
        }
        break;
    }

    if (context->config.release_bridge_after_transfer &&
        context->bridge_acquired) {
        mmi2c_result_t release_result = release_bridge_locked(context);
        if (result == MMI2C_RESULT_OK) {
            result = release_result;
        }
    }

    clear_waiting_task(context);
    clear_task_notification();
    (void) xSemaphoreGive(context->mutex);
    return result;
}

mmi2c_result_t mmi2c_release_bridge(mmi2c_t *context)
{
    if ((context == NULL) || (context->mutex == NULL)) {
        return MMI2C_RESULT_INVALID_ARGUMENT;
    }
    if (xSemaphoreTake(context->mutex,
            context->config.mutex_timeout_ticks) != pdTRUE) {
        return MMI2C_RESULT_BUSY;
    }

    set_waiting_task(context);
    mmi2c_result_t result = release_bridge_locked(context);
    clear_waiting_task(context);
    clear_task_notification();
    (void) xSemaphoreGive(context->mutex);
    return result;
}

static void finish_from_isr(mmi2c_t *context, mmi2c_result_t result,
    BaseType_t *higher_priority_task_woken)
{
    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS |
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    context->state = MMI2C_STATE_IDLE;
    context->result = result;
    notify_waiting_task_from_isr(context, higher_priority_task_woken);
}

static void handle_tx_fifo_request(mmi2c_t *context)
{
    if (context->state != MMI2C_STATE_TRANSMITTING) {
        return;
    }
    if (controller_has_error(context)) {
        DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS);
        return;
    }

    if (context->tx_count < context->transfer.tx_length) {
        DL_I2C_transmitControllerData(context->regs,
            context->transfer.tx_data[context->tx_count]);
        ++context->tx_count;
    }
    if (context->tx_count == context->transfer.tx_length) {
        DL_I2C_disableInterrupt(context->regs,
            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    }
}

static void handle_tx_done(mmi2c_t *context,
    BaseType_t *higher_priority_task_woken)
{
    DL_I2C_disableInterrupt(context->regs, MMI2C_TX_FIFO_INTERRUPTS);
    if ((context->state != MMI2C_STATE_TRANSMITTING) ||
        controller_has_error(context)) {
        return;
    }

    if (context->transfer.rx_length != 0u) {
        start_receive(context);
    } else {
        finish_from_isr(
            context, MMI2C_RESULT_OK, higher_priority_task_woken);
    }
}

static void handle_rx_done(mmi2c_t *context,
    BaseType_t *higher_priority_task_woken)
{
    if (context->state != MMI2C_STATE_RECEIVING) {
        return;
    }
    if (controller_has_error(context)) {
        DL_I2C_disableInterrupt(context->regs,
            DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
        return;
    }

    DL_I2C_disableInterrupt(context->regs,
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    context->state = MMI2C_STATE_RECEIVE_SETTLE;
    notify_waiting_task_from_isr(context, higher_priority_task_woken);
}

static void handle_start(mmi2c_t *context)
{
    if ((context->state != MMI2C_STATE_TRANSMITTING) &&
        (context->state != MMI2C_STATE_RECEIVING)) {
        context->bus_locked = true;
    }
}

static void handle_stop(mmi2c_t *context,
    BaseType_t *higher_priority_task_woken)
{
    context->bus_locked =
        (controller_status(context) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u;
    notify_waiting_task_from_isr(context, higher_priority_task_woken);
}

static void handle_arbitration_lost(mmi2c_t *context,
    BaseType_t *higher_priority_task_woken)
{
    if ((context->state != MMI2C_STATE_TRANSMITTING) &&
        (context->state != MMI2C_STATE_RECEIVING)) {
        return;
    }

    /* The winning controller owns the bus. Do not generate STOP. If STOP was
     * serviced first, an already-idle status prevents a stale lock. */
    uint32_t status = controller_status(context);
    context->bus_locked =
        (status & (DL_I2C_CONTROLLER_STATUS_BUSY_BUS |
                      DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0u;
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);
    finish_from_isr(context, MMI2C_RESULT_ARBITRATION_LOST,
        higher_priority_task_woken);
}

static void handle_nack(mmi2c_t *context,
    BaseType_t *higher_priority_task_woken)
{
    if ((context->state != MMI2C_STATE_TRANSMITTING) &&
        (context->state != MMI2C_STATE_RECEIVING) &&
        (context->state != MMI2C_STATE_RECEIVE_SETTLE)) {
        return;
    }

    /* NACK has higher IIDX priority than arbitration-lost. If both are
     * pending, leave the bus alone and let the arbitration handler run next. */
    if ((controller_status(context) &
            DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0u) {
        return;
    }

    DL_I2C_enableStopCondition(context->regs);
    DL_I2C_flushControllerTXFIFO(context->regs);
    DL_I2C_flushControllerRXFIFO(context->regs);
    finish_from_isr(context, MMI2C_RESULT_NACK,
        higher_priority_task_woken);
}

void mmi2c_handle_irq(mmi2c_t *context)
{
    if ((context == NULL) || (context->regs == NULL)) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;

    /* Handle one event only. Any remaining event retriggers the IRQ. */
    switch (DL_I2C_getPendingInterrupt(context->regs)) {
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            handle_tx_fifo_request(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
            if ((context->state == MMI2C_STATE_RECEIVING) ||
                (context->state == MMI2C_STATE_RECEIVE_SETTLE)) {
                if (controller_has_error(context)) {
                    DL_I2C_disableInterrupt(context->regs,
                        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
                } else {
                    receive_one_byte(context);
                }
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            handle_tx_done(context, &higher_priority_task_woken);
            break;

        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            handle_rx_done(context, &higher_priority_task_woken);
            break;

        case DL_I2C_IIDX_CONTROLLER_START:
            handle_start(context);
            break;

        case DL_I2C_IIDX_CONTROLLER_STOP:
            handle_stop(context, &higher_priority_task_woken);
            break;

        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
            handle_arbitration_lost(context, &higher_priority_task_woken);
            break;

        case DL_I2C_IIDX_CONTROLLER_NACK:
            handle_nack(context, &higher_priority_task_woken);
            break;

        case DL_I2C_IIDX_NO_INT:
        default:
            break;
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}
