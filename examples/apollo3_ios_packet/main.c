#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "ios_packet_sender.h"
#include "sensor_stub.h"

static mmi2c_t g_i2c;
static ios_packet_sender_t g_sender;
static StaticTask_t g_sender_task_storage;
static StackType_t g_sender_task_stack[APP_SENDER_TASK_STACK_WORDS];

static void delay_at_least_ms(uint32_t milliseconds)
{
    /* The extra tick prevents a delay shortened by the current tick phase. */
    vTaskDelay(pdMS_TO_TICKS(milliseconds) + 1u);
}

static bool app_bridge_acquire(void *unused)
{
    (void) unused;

    DL_GPIO_clearPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    DL_GPIO_setPins(APP_BRIDGE_VDDB_PORT, APP_BRIDGE_VDDB_PIN);
    delay_at_least_ms(APP_BRIDGE_POWER_SETTLE_MS);

    DL_GPIO_setPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    delay_at_least_ms(APP_BRIDGE_ENABLE_SETTLE_MS);
    return true;
}

static void app_bridge_release(void *unused)
{
    (void) unused;
    DL_GPIO_clearPins(APP_BRIDGE_EN_PORT, APP_BRIDGE_EN_PIN);
    DL_GPIO_clearPins(APP_BRIDGE_VDDB_PORT, APP_BRIDGE_VDDB_PIN);
}

static void app_task(void *argument)
{
    /* FromISR APIs are safe only after the scheduler has started. */
    DL_GPIO_clearInterruptStatus(APP_BUTTON_PORT, APP_BUTTON_PIN);
    NVIC_ClearPendingIRQ(APP_I2C_IRQN);
    NVIC_ClearPendingIRQ(APP_BUTTON_IRQN);
    NVIC_EnableIRQ(APP_I2C_IRQN);
    NVIC_EnableIRQ(APP_BUTTON_IRQN);

    ios_packet_sender_task(argument);
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

/* A product timer ISR requests a packet with the same one-line call:
 *
 *     ios_packet_sender_request_from_isr(&g_sender);
 */

int main(void)
{
    SYSCFG_DL_init();

    mmi2c_config_t i2c_config = {
        .bridge_acquire = app_bridge_acquire,
        .bridge_release = app_bridge_release,
        .bus_recover = NULL,
        .user = NULL,
        .mutex_timeout_ticks = pdMS_TO_TICKS(100u) + 1u,
        .bus_timeout_ticks = pdMS_TO_TICKS(10u) + 1u,
        .transfer_timeout_ticks = pdMS_TO_TICKS(5u) + 1u,
        .backoff_slot_ticks = pdMS_TO_TICKS(1u) + 1u,
        /* Two ticks is intentionally conservative for I2C_ERR_08. */
        .rx_settle_ticks = 2u,
        .random_seed = APP_UNIQUE_RANDOM_SEED,
        .max_arbitration_retries = 8u,
        .max_nack_retries = 3u,
        /* Keep the bridge on across both data chunks and the trigger. */
        .release_bridge_after_transfer = false,
    };

    if (mmi2c_init(&g_i2c, APP_I2C_INST, &i2c_config) !=
        MMI2C_RESULT_OK) {
        for (;;) {
        }
    }

    if (!ios_packet_sender_init(&g_sender, &g_i2c, APP_IOS_ADDRESS,
            APP_MODULE_ID, APP_MESSAGE_TYPE_SENSOR, sensor_read_payload,
            NULL)) {
        for (;;) {
        }
    }

    TaskHandle_t sender_task = xTaskCreateStatic(app_task,
        "packet", APP_SENDER_TASK_STACK_WORDS, &g_sender,
        APP_SENDER_TASK_PRIORITY, g_sender_task_stack,
        &g_sender_task_storage);
    if (sender_task == NULL) {
        for (;;) {
        }
    }

    app_bridge_release(NULL);
    vTaskStartScheduler();

    /* Reached only if the scheduler could not start. */
    for (;;) {
    }
}
