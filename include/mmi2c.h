#ifndef MMI2C_H
#define MMI2C_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <ti/driverlib/dl_i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MMI2C_MAX_MESSAGE_BYTES
#define MMI2C_MAX_MESSAGE_BYTES (8u)
#endif

#if (MMI2C_MAX_MESSAGE_BYTES < 1u) || (MMI2C_MAX_MESSAGE_BYTES > 4095u)
#error "MMI2C_MAX_MESSAGE_BYTES must be in the MSPM0 MBLEN range 1..4095"
#endif

#if (configSUPPORT_STATIC_ALLOCATION != 1)
#error "mmi2c requires configSUPPORT_STATIC_ALLOCATION == 1"
#endif

#if (configUSE_MUTEXES != 1)
#error "mmi2c requires configUSE_MUTEXES == 1"
#endif

#if (configUSE_TASK_NOTIFICATIONS != 1)
#error "mmi2c requires configUSE_TASK_NOTIFICATIONS == 1"
#endif

typedef enum {
    MMI2C_RESULT_OK = 0,
    MMI2C_RESULT_INVALID_ARGUMENT,
    MMI2C_RESULT_BUSY,
    MMI2C_RESULT_ARBITRATION_LOST,
    MMI2C_RESULT_NACK,
    MMI2C_RESULT_BUS_BUSY_TIMEOUT,
    MMI2C_RESULT_TRANSFER_TIMEOUT,
    MMI2C_RESULT_BRIDGE_NOT_READY,
    MMI2C_RESULT_SHORT_READ
} mmi2c_result_t;

typedef bool (*mmi2c_bridge_acquire_fn)(void *user);
typedef void (*mmi2c_bridge_release_fn)(void *user);
typedef bool (*mmi2c_bus_recover_fn)(void *user);

typedef struct {
    mmi2c_bridge_acquire_fn bridge_acquire;
    mmi2c_bridge_release_fn bridge_release;
    mmi2c_bus_recover_fn bus_recover;
    void *user;

    TickType_t mutex_timeout_ticks;
    TickType_t bus_timeout_ticks;
    TickType_t transfer_timeout_ticks;
    TickType_t backoff_slot_ticks;
    TickType_t rx_settle_ticks;

    uint32_t random_seed;
    uint8_t max_arbitration_retries;
    uint8_t max_nack_retries;

    /* Set false when several transfers share one PCA9615 power cycle. */
    bool release_bridge_after_transfer;
} mmi2c_config_t;

typedef struct {
    uint8_t address;          /* Unshifted 7-bit address. */
    const uint8_t *tx_data;
    uint8_t *rx_data;
    uint16_t tx_length;
    uint16_t rx_length;
} mmi2c_transfer_t;

typedef enum {
    MMI2C_STATE_IDLE = 0,
    MMI2C_STATE_TRANSMITTING,
    MMI2C_STATE_RECEIVING,
    MMI2C_STATE_RECEIVE_SETTLE
} mmi2c_state_t;

typedef struct {
    I2C_Regs *regs;
    mmi2c_config_t config;
    SemaphoreHandle_t mutex;
    StaticSemaphore_t mutex_storage;

    mmi2c_transfer_t transfer;
    TaskHandle_t volatile waiting_task;
    volatile mmi2c_state_t state;
    volatile mmi2c_result_t result;
    volatile uint16_t tx_count;
    volatile uint16_t rx_count;
    volatile bool bus_locked;

    uint32_t prng;
    bool bridge_acquired;
} mmi2c_t;

/* Call once after SysConfig initializes the I2C peripheral. */
mmi2c_result_t mmi2c_init(
    mmi2c_t *context, I2C_Regs *regs, const mmi2c_config_t *config);

/*
 * Performs one complete transfer and blocks the calling task until it
 * succeeds, fails, or times out. Never call this function from an ISR.
 *
 * The context mutex serializes callers. This function uses notification index
 * zero of the calling task while the transfer is active.
 */
mmi2c_result_t mmi2c_transfer(
    mmi2c_t *context, const mmi2c_transfer_t *transfer);

/*
 * Waits for an idle bus, then deasserts the bridge through bridge_release().
 * Use this after a batch when release_bridge_after_transfer is false.
 */
mmi2c_result_t mmi2c_release_bridge(mmi2c_t *context);

/* Call from the selected I2C IRQ. It handles one IIDX event and returns. */
void mmi2c_handle_irq(mmi2c_t *context);

#ifdef __cplusplus
}
#endif

#endif
