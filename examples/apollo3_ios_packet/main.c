#include "app_config.h"
#include "ios_packet_sender.h"
#include "sensor_stub.h"

static mmi2c_t g_i2c;
static ios_packet_sender_t g_sender;
static volatile bool g_send_requested;
static uint16_t g_last_timebase_count;
static uint32_t g_time_us;

/* TIMER_TIMEBASE is a 1 MHz, 16-bit, free-running down counter. This function
 * extends it in software. It is polled continuously whenever a timeout is
 * active, so its 65.536 ms hardware wrap is not missed during a send. */
static uint32_t app_now_us(void *unused)
{
    (void) unused;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t count = (uint16_t) (UINT16_MAX -
        DL_TimerG_getTimerCount(APP_TIMEBASE_INST));
    g_time_us += (uint16_t) (count - g_last_timebase_count);
    g_last_timebase_count = count;

    uint32_t now = g_time_us;
    __set_PRIMASK(primask);
    return now;
}

static void start_timebase(void)
{
    DL_TimerG_startCounter(APP_TIMEBASE_INST);
    g_last_timebase_count = (uint16_t) (UINT16_MAX -
        DL_TimerG_getTimerCount(APP_TIMEBASE_INST));
}

static void delay_us(uint32_t delay)
{
    uint32_t started_at = app_now_us(NULL);
    while ((uint32_t) (app_now_us(NULL) - started_at) < delay) {
        /* This short, deliberate busy wait needs no timer interrupt. */
    }
}

static void app_bridge_release(void *unused)
{
    (void) unused;
    DL_GPIO_clearPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    DL_GPIO_clearPins(APP_BRIDGE_VDDB_PORT, APP_BRIDGE_VDDB_PIN);
}

static bool app_bridge_acquire(void *unused, uint32_t timeout_us)
{
    (void) unused;

    if (timeout_us < APP_BRIDGE_STARTUP_US) {
        return false;
    }

    /* PCA9615 VDD(B) and EN are separate. Keep EN low while VDD(B) rises. */
    DL_GPIO_clearPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    DL_GPIO_setPins(APP_BRIDGE_VDDB_PORT, APP_BRIDGE_VDDB_PIN);
    delay_us(APP_BRIDGE_POWER_SETTLE_US);

    DL_GPIO_setPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    delay_us(APP_BRIDGE_ENABLE_SETTLE_US);
    return true;
}

/* The I2C interrupt handler does only I2C work. */
void APP_I2C_IRQHandler(void)
{
    mmi2c_handle_irq(&g_i2c);
}

/* The button interrupt only records the request and wakes the main loop. */
void APP_BUTTON_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        APP_BUTTON_PORT, APP_BUTTON_PIN);
    if ((status & APP_BUTTON_PIN) != 0u) {
        DL_GPIO_clearInterruptStatus(APP_BUTTON_PORT, APP_BUTTON_PIN);
        g_send_requested = true;
        __SEV();
    }
}

/* A product timer ISR can request a packet in exactly the same way:
 *
 *     g_send_requested = true;
 *     __SEV();
 *
 * Do not call ios_packet_sender_send() from an ISR.
 */

static bool take_send_request(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool requested = g_send_requested;
    g_send_requested = false;
    __set_PRIMASK(primask);
    return requested;
}

int main(void)
{
    SYSCFG_DL_init();
    start_timebase();

    mmi2c_config_t i2c_config = {
        .now_us                   = app_now_us,
        .bridge_acquire           = app_bridge_acquire,
        .bridge_release           = app_bridge_release,
        .bus_recover              = NULL,
        .user                     = NULL,
        .bus_idle_timeout_us      = 10000u,
        .transfer_timeout_us      = 5000u,
        .bridge_ready_timeout_us  = APP_BRIDGE_STARTUP_US,
        .rx_done_settle_us        = 2u,
        .random_seed              = APP_UNIQUE_RANDOM_SEED,
        .backoff_slot_us          = 100u,
        .max_arbitration_retries  = 8u,
        .max_nack_retries         = 0u,
        /* Keep the bridge on for all data chunks plus the trigger. */
        .release_bridge_when_done = false,
    };

    if (mmi2c_init(&g_i2c, APP_I2C_INST, &i2c_config) != MMI2C_RESULT_OK) {
        for (;;) {
            __WFE();
        }
    }

    if (!ios_packet_sender_init(&g_sender, &g_i2c, APP_IOS_ADDRESS,
            APP_MODULE_ID, APP_MESSAGE_TYPE_SENSOR, sensor_read_payload,
            NULL, 3u)) {
        for (;;) {
            __WFE();
        }
    }

    app_bridge_release(NULL);
    NVIC_EnableIRQ(APP_I2C_IRQN);
    NVIC_EnableIRQ(APP_BUTTON_IRQN);

    for (;;) {
        /* __SEV() in the request ISR makes this race-safe: if the interrupt
         * occurs just before __WFE(), the pending event prevents sleeping. */
        while (!take_send_request()) {
            __WFE();
        }

        /* One straightforward, sequential operation. All I2C completion
         * events are still serviced by APP_I2C_IRQHandler(). */
        (void) ios_packet_sender_send(&g_sender);
    }
}
