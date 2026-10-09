#include "FreeRTOS.h"
#include "task.h"

#include "mmi2c.h"
#include "ti_msp_dl_config.h"

#define APP_I2C_INST I2C_0_INST
#define APP_I2C_IRQN I2C_0_INST_INT_IRQN

static mmi2c_t g_i2c;
static StaticTask_t g_demo_task_storage;
static StackType_t g_demo_task_stack[192u];

static bool app_bridge_acquire(void *unused)
{
    (void) unused;

    /* Board code: EN low, switch VDD(B) on, delay >= 11 ms, then EN high. */
    vTaskDelay(pdMS_TO_TICKS(12u) + 1u);
    vTaskDelay(pdMS_TO_TICKS(1u) + 1u);
    return true;
}

static void app_bridge_release(void *unused)
{
    (void) unused;
    /* Board code: drive EN low, then switch VDD(B) off. */
}

void I2C_0_INST_IRQHandler(void)
{
    mmi2c_handle_irq(&g_i2c);
}

static void demo_task(void *unused)
{
    (void) unused;

    /* Enable the IRQ only after the scheduler can service FromISR calls. */
    NVIC_ClearPendingIRQ(APP_I2C_IRQN);
    NVIC_EnableIRQ(APP_I2C_IRQN);

    const uint8_t message[] = {
        0xA5u, 0x01u, 0x12u, 0x34u, 0x56u, 0x78u, 0x9Au, 0xBCu};
    const mmi2c_transfer_t transfer = {
        .address = 0x42u,
        .tx_data = message,
        .rx_data = NULL,
        .tx_length = sizeof(message),
        .rx_length = 0u,
    };

    mmi2c_result_t result = mmi2c_transfer(&g_i2c, &transfer);
    (void) result;

    vTaskDelete(NULL);
}

int main(void)
{
    SYSCFG_DL_init();

    const mmi2c_config_t config = {
        .bridge_acquire = app_bridge_acquire,
        .bridge_release = app_bridge_release,
        .bus_recover = NULL,
        .user = NULL,
        .mutex_timeout_ticks = pdMS_TO_TICKS(100u) + 1u,
        .bus_timeout_ticks = pdMS_TO_TICKS(10u) + 1u,
        .transfer_timeout_ticks = pdMS_TO_TICKS(5u) + 1u,
        .backoff_slot_ticks = pdMS_TO_TICKS(1u) + 1u,
        .rx_settle_ticks = 2u,
        .random_seed = 0x12345678u, /* Replace with a unique module value. */
        .max_arbitration_retries = 8u,
        .max_nack_retries = 0u,
        .release_bridge_after_transfer = true,
    };

    if (mmi2c_init(&g_i2c, APP_I2C_INST, &config) != MMI2C_RESULT_OK) {
        for (;;) {
        }
    }

    if (xTaskCreateStatic(demo_task, "i2c", 192u, NULL,
            tskIDLE_PRIORITY + 2u, g_demo_task_stack,
            &g_demo_task_storage) == NULL) {
        for (;;) {
        }
    }

    vTaskStartScheduler();
    for (;;) {
    }
}
