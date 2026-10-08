#include "app_config.h"
#include "ios_packet_sender.h"
#include "sensor_stub.h"

static mmi2c_t g_i2c;
static ios_packet_sender_t g_sender;
static volatile uint32_t g_now_us;
static uint32_t g_sample_divider_ticks;

static uint32_t app_now_us(void *unused)
{
    (void) unused;
    return g_now_us;
}

static bool delay_us_with_timeout(
    uint32_t delay_us, uint32_t started_at, uint32_t timeout_us)
{
    uint32_t delay_started_at = app_now_us(NULL);
    while ((uint32_t) (app_now_us(NULL) - delay_started_at) < delay_us) {
        if ((uint32_t) (app_now_us(NULL) - started_at) >= timeout_us) {
            return false;
        }
        __WFI();
    }
    return true;
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
    uint32_t started_at = app_now_us(NULL);

    /* PCA9615 VDD(B) and EN are separate. Keep EN low while VDD(B) rises. */
    DL_GPIO_clearPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    DL_GPIO_setPins(APP_BRIDGE_VDDB_PORT, APP_BRIDGE_VDDB_PIN);

    if (!delay_us_with_timeout(12000u, started_at, timeout_us)) {
        app_bridge_release(NULL);
        return false;
    }

    DL_GPIO_setPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    if (!delay_us_with_timeout(1000u, started_at, timeout_us)) {
        app_bridge_release(NULL);
        return false;
    }
    return true;
}

void APP_I2C_IRQHandler(void)
{
    mmi2c_handle_irq(&g_i2c);
}

void APP_BUTTON_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        APP_BUTTON_PORT, APP_BUTTON_PIN);
    if ((status & APP_BUTTON_PIN) != 0u) {
        DL_GPIO_clearInterruptStatus(APP_BUTTON_PORT, APP_BUTTON_PIN);
        ios_packet_sender_request_from_isr(&g_sender);
    }
}

/* SysTick is the example's timer interrupt and monotonic time source. Replace
 * it with a low-power hardware timer in the final product if periodic 1 ms
 * wakeups are not acceptable. */
void SysTick_Handler(void)
{
    g_now_us += (1000000u / APP_SYSTICK_HZ);
    ++g_sample_divider_ticks;
    if (g_sample_divider_ticks >= APP_SAMPLE_PERIOD_TICKS) {
        g_sample_divider_ticks = 0u;
        ios_packet_sender_request_from_isr(&g_sender);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    mmi2c_config_t i2c_config = {
        .now_us                   = app_now_us,
        .bridge_acquire           = app_bridge_acquire,
        .bridge_release           = app_bridge_release,
        .bus_recover              = NULL,
        .user                     = NULL,
        .bus_idle_timeout_us      = 10000u,
        .transfer_timeout_us      = 5000u,
        .bridge_ready_timeout_us  = 15000u,
        .rx_done_settle_us        = 2000u,
        .random_seed              = APP_UNIQUE_RANDOM_SEED,
        .backoff_slot_us          = 1000u,
        .max_arbitration_retries  = 8u,
        .max_nack_retries         = 0u,
        /* Keep the bridge on for all data chunks plus the trigger. */
        .release_bridge_when_done = false,
    };

    if (mmi2c_init(&g_i2c, APP_I2C_INST, &i2c_config) != MMI2C_RESULT_OK) {
        for (;;) {
            __WFI();
        }
    }

    if (!ios_packet_sender_init(&g_sender, &g_i2c, APP_IOS_ADDRESS,
            APP_MODULE_ID, APP_MESSAGE_TYPE_SENSOR, sensor_read_payload,
            app_now_us, NULL, 3u, 2000u)) {
        for (;;) {
            __WFI();
        }
    }

    app_bridge_release(NULL);
    NVIC_EnableIRQ(APP_I2C_IRQN);
    NVIC_EnableIRQ(APP_BUTTON_IRQN);
    if (SysTick_Config(CPUCLK_FREQ / APP_SYSTICK_HZ) != 0u) {
        for (;;) {
            __WFI();
        }
    }

    for (;;) {
        ios_packet_sender_process(&g_sender);
        __WFI();
    }
}
