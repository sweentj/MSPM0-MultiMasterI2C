#ifndef MMI2C_H
#define MMI2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

typedef enum {
    MMI2C_RESULT_NONE = 0,
    MMI2C_RESULT_IN_PROGRESS,
    MMI2C_RESULT_OK,
    MMI2C_RESULT_INVALID_ARGUMENT,
    MMI2C_RESULT_BUSY,
    MMI2C_RESULT_ARBITRATION_LOST,
    MMI2C_RESULT_NACK,
    MMI2C_RESULT_BUS_BUSY_TIMEOUT,
    MMI2C_RESULT_TRANSFER_TIMEOUT,
    MMI2C_RESULT_BRIDGE_NOT_READY,
    MMI2C_RESULT_SHORT_READ
} mmi2c_result_t;

typedef enum {
    MMI2C_STATE_UNINITIALIZED = 0,
    MMI2C_STATE_IDLE,
    MMI2C_STATE_WAITING_FOR_BUS,
    MMI2C_STATE_TRANSMITTING,
    MMI2C_STATE_RECEIVING,
    MMI2C_STATE_RECEIVE_SETTLE,
    MMI2C_STATE_WAITING_TO_RETRY,
    MMI2C_STATE_COMPLETE
} mmi2c_state_t;

typedef uint32_t (*mmi2c_now_us_fn)(void *user);
typedef bool (*mmi2c_bridge_acquire_fn)(void *user, uint32_t timeout_us);
typedef void (*mmi2c_bridge_release_fn)(void *user);
typedef bool (*mmi2c_bus_recover_fn)(void *user);

typedef struct {
    /* Must be monotonic modulo 2^32. Required. */
    mmi2c_now_us_fn now_us;

    /* Optional PCA9615 power/EN hooks. acquire() must return only after the
     * buffer is connected and ready. release() may deassert EN or power. */
    mmi2c_bridge_acquire_fn bridge_acquire;
    mmi2c_bridge_release_fn bridge_release;

    /* Optional board-specific GPIO bus recovery. It is called only after the
     * bus-busy timeout, never merely because arbitration was lost. */
    mmi2c_bus_recover_fn bus_recover;
    void *user;

    uint32_t bus_idle_timeout_us;
    uint32_t transfer_timeout_us;
    uint32_t bridge_ready_timeout_us;
    /* MSPM0 I2C_ERR_08 workaround: delay after RXDONE before the final FIFO
     * read. Must cover at least two I2C functional-clock cycles. */
    uint16_t rx_done_settle_us;

    /* Arbitration retries use binary exponential randomized backoff. The
     * seed must differ between modules (for example, derived from serial ID). */
    uint32_t random_seed;
    uint16_t backoff_slot_us;
    uint8_t max_arbitration_retries;
    uint8_t max_nack_retries;

    /* If true, bridge_release() is called after every terminal result. */
    bool release_bridge_when_done;
} mmi2c_config_t;

typedef struct {
    uint8_t address;          /* Unshifted 7-bit address. */
    const uint8_t *tx_data;   /* May be NULL only when tx_length is zero. */
    uint8_t *rx_data;         /* May be NULL only when rx_length is zero. */
    uint16_t tx_length;
    uint16_t rx_length;
} mmi2c_transfer_t;

typedef struct {
    I2C_Regs *regs;
    mmi2c_config_t config;
    mmi2c_transfer_t transfer;
    volatile mmi2c_state_t state;
    volatile mmi2c_result_t result;
    volatile uint16_t tx_count;
    volatile uint16_t rx_count;
    volatile uint8_t arbitration_retries;
    volatile uint8_t nack_retries;
    uint32_t deadline_us;
    uint32_t retry_at_us;
    uint32_t prng;
    bool bridge_acquired;
    bool recovery_attempted;
} mmi2c_t;

/**
 * Initializes a context around an I2C instance already configured by
 * SysConfig/DriverLib. This function enables MSPM0 multi-controller timing and
 * the required peripheral interrupt sources; the caller still enables the
 * corresponding NVIC IRQ.
 */
mmi2c_result_t mmi2c_init(
    mmi2c_t *context, I2C_Regs *regs, const mmi2c_config_t *config);

/**
 * Queues one write, read, or combined write/repeated-START/read transaction.
 * tx_length + rx_length must not exceed MMI2C_MAX_MESSAGE_BYTES. Buffers must
 * remain valid until the result is taken.
 */
mmi2c_result_t mmi2c_submit(
    mmi2c_t *context, const mmi2c_transfer_t *transfer);

/** Call regularly from the main loop or a timer tick to handle waits/timeouts. */
void mmi2c_process(mmi2c_t *context);

/** Call from the selected I2C peripheral's IRQ handler. */
void mmi2c_handle_irq(mmi2c_t *context);

/** Returns the current result without changing it. */
mmi2c_result_t mmi2c_get_result(const mmi2c_t *context);

/**
 * Returns a terminal result and returns the context to IDLE. Returns
 * IN_PROGRESS if active, or NONE when no completed transaction is available.
 */
mmi2c_result_t mmi2c_take_result(mmi2c_t *context);

bool mmi2c_is_busy(const mmi2c_t *context);

#ifdef __cplusplus
}
#endif

#endif
