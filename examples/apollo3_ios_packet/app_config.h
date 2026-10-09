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

/* SysConfig name: TIMER_TIMEBASE. Configure TIMG8 as a 1 MHz, periodic,
 * 16-bit down counter with load value 0xFFFF. Do not enable its IRQ. */
#define APP_TIMEBASE_INST             TIMER_TIMEBASE_INST

#define APP_IOS_ADDRESS              (0x10u)
#define APP_MODULE_ID                (0x01u)
#define APP_MESSAGE_TYPE_SENSOR      (0x01u)

#define APP_BRIDGE_POWER_SETTLE_US  (12000u)
#define APP_BRIDGE_ENABLE_SETTLE_US (1000u)
#define APP_BRIDGE_STARTUP_US                                        \
    (APP_BRIDGE_POWER_SETTLE_US + APP_BRIDGE_ENABLE_SETTLE_US)

/* Replace with a nonzero value derived from a unique factory identifier. */
#define APP_UNIQUE_RANDOM_SEED       (0x13579BDFu)

#endif
