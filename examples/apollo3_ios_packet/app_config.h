#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "ti_msp_dl_config.h"

/* These aliases assume the SysConfig component names described in README.md.
 * Change only this file if your generated identifiers differ. */
#define APP_I2C_INST                 I2C_0_INST
#define APP_I2C_IRQN                 I2C_0_INST_INT_IRQN
#define APP_I2C_IRQHandler           I2C_0_INST_IRQHandler

#define APP_BUTTON_PORT              GPIO_BUTTON_PORT
#define APP_BUTTON_PIN               GPIO_BUTTON_BUTTON_PIN
#define APP_BUTTON_IRQN              GPIO_BUTTON_INT_IRQN
#define APP_BUTTON_IRQHandler        GROUP1_IRQHandler

#define APP_BRIDGE_EN_PORT           GPIO_BRIDGE_PORT
#define APP_BRIDGE_EN_PIN            GPIO_BRIDGE_EN_PIN
#define APP_BRIDGE_VDDB_PORT         GPIO_BRIDGE_PORT
#define APP_BRIDGE_VDDB_PIN          GPIO_BRIDGE_VDDB_PIN

#define APP_IOS_ADDRESS              (0x10u)
#define APP_MODULE_ID                (0x01u)
#define APP_MESSAGE_TYPE_SENSOR      (0x01u)

#define APP_SYSTICK_HZ               (1000u)
#define APP_SAMPLE_PERIOD_MS         (1000u)

#if ((1000000u % APP_SYSTICK_HZ) != 0u)
#error "APP_SYSTICK_HZ must divide 1,000,000 exactly"
#endif

#define APP_SAMPLE_PERIOD_TICKS                                      \
    ((APP_SAMPLE_PERIOD_MS * APP_SYSTICK_HZ) / 1000u)

#if (APP_SAMPLE_PERIOD_TICKS == 0u)
#error "The sample period must be at least one SysTick interval"
#endif

/* Replace with a nonzero value derived from a unique factory identifier. */
#define APP_UNIQUE_RANDOM_SEED       (0x13579BDFu)

#endif
