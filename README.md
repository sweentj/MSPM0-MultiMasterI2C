# MSPM0 FreeRTOS multi-controller I2C

A small FreeRTOS/DriverLib I2C controller library for MSPM0G and MSPM0L
devices. The public transfer API is synchronous and thread-safe:

```c
mmi2c_result_t result = mmi2c_transfer(&i2c, &transfer);
```

The calling task sleeps while the hardware is busy. The I2C ISR handles one
IIDX event, records the result, and wakes that task with a direct task
notification. There is no application polling function or I2C state machine to
advance from a timer tick.

## What it supports

- Multiple controllers sharing a 400 kHz bus.
- Write, read, and write/repeated-START/read transactions.
- Eight total data bytes per transaction by default.
- A FreeRTOS mutex around each hardware instance.
- Bus-idle and transfer timeouts expressed in FreeRTOS ticks.
- Randomized arbitration-loss retries.
- Optional NACK retries.
- Optional PCA9615 VDD(B)/EN callbacks.
- Optional stuck-bus recovery callback.
- Static FreeRTOS allocation only.

## Public API

There are four calls:

```c
mmi2c_init(&i2c, I2C_0_INST, &config);
mmi2c_transfer(&i2c, &transfer);
mmi2c_release_bridge(&i2c);
mmi2c_handle_irq(&i2c);
```

`mmi2c_transfer()` blocks only the calling task. Other tasks continue to run.
The internal mutex allows several tasks to share one I2C instance safely.

When one logical message requires several I2C transactions, set
`release_bridge_after_transfer = false`, perform the transfers, then call
`mmi2c_release_bridge()`. The Apollo3 packet example uses this to keep the
PCA9615 powered across two data writes and the trigger write.

## Multi-controller behavior

Bus ownership is deliberately simple:

- A START observed while this MCU is not transferring locks the bus.
- Arbitration loss locks the bus and wakes the calling task with
  `MMI2C_RESULT_ARBITRATION_LOST`.
- The losing controller never generates STOP.
- STOP unlocks the bus unless hardware already reports a new transaction.
- Every new attempt requires both the software lock to be clear and the
  MSPM0 peripheral to report IDLE and bus-not-busy.

After arbitration loss, the task waits for STOP, sleeps for a randomized
binary-exponential backoff, and retries the complete transaction. No retry
delay occurs in the ISR.

The hardware BUSBSY check remains necessary because a newly connected PCA9615
can join a transaction after its START has already occurred.

## Interrupt handler

```c
void I2C_0_INST_IRQHandler(void)
{
    mmi2c_handle_irq(&i2c);
}
```

`mmi2c_handle_irq()` reads IIDX once and returns. It contains no interrupt
draining loop. TX and RX FIFO service each moves one byte. The final RX FIFO
drain for MSPM0 I2C_ERR_08 runs in task context after
`rx_settle_ticks`.

Because the ISR calls `vTaskNotifyGiveFromISR()`, configure its NVIC priority as
a FreeRTOS system-call-safe interrupt priority. The library temporarily uses
notification index zero of the task calling `mmi2c_transfer()`; that task must
not use the same notification concurrently.

## Minimal configuration

```c
static const mmi2c_config_t config = {
    .bridge_acquire = bridge_acquire,
    .bridge_release = bridge_release,
    .mutex_timeout_ticks = pdMS_TO_TICKS(100) + 1,
    .bus_timeout_ticks = pdMS_TO_TICKS(10) + 1,
    .transfer_timeout_ticks = pdMS_TO_TICKS(5) + 1,
    .backoff_slot_ticks = pdMS_TO_TICKS(1) + 1,
    .rx_settle_ticks = 2,
    .random_seed = UNIQUE_MODULE_SEED,
    .max_arbitration_retries = 8,
    .max_nack_retries = 0,
    .release_bridge_after_transfer = true,
};
```

The extra tick guarantees a nonzero delay even when the configured tick rate
cannot represent one millisecond exactly.

Each module needs a different nonzero `random_seed`, preferably derived from a
factory identifier. Identical seeds and synchronized activation can produce
repeated collisions.

## Project setup

Start from TI's `empty_freertos` example for the selected MSPM0 device, then
add:

- `include/mmi2c.h`
- `src/mmi2c.c`
- the application files that use the library

FreeRTOS must enable:

```c
#define configSUPPORT_STATIC_ALLOCATION 1
#define configUSE_MUTEXES               1
#define configUSE_TASK_NOTIFICATIONS    1
```

SysConfig must configure the selected I2C instance as a 400 kHz controller,
with TX/RX FIFO thresholds of one byte and DMA disabled. Enable the peripheral
IRQ from a task after the scheduler has started. This prevents an early IRQ
from calling a FreeRTOS `FromISR` API before the kernel is running.

The library is intended for FreeRTOS 11.2.0 supplied with MSPM0 SDK 2.11.00.07.
It uses standard FreeRTOS queue, mutex, task-delay, timeout, and direct-task-
notification APIs.

The repository Makefile provides compile-only checks when GNU Arm Embedded is
on `PATH`:

```sh
make check-example-g3519 SDK=/path/to/mspm0-sdk
make check-example-l1116 SDK=/path/to/mspm0-sdk
```

## PCA9615 power control

`bridge_acquire()` runs in task context, so it may call `vTaskDelay()`:

1. Hold EN low.
2. Switch VDD(B) on.
3. Delay at least 11 ms; the example uses 12 ms plus tick-phase margin.
4. Assert EN.
5. Allow the bus-idle qualification interval.

`bridge_release()` must drive EN low before switching VDD(B) off. The library
calls it only after the bus is idle. If the idle wait times out, the bridge is
left powered rather than disconnecting an active transaction.

## Assumptions and limits

- Seven-bit addresses only.
- `tx_length + rx_length <= MMI2C_MAX_MESSAGE_BYTES` (eight by default).
- Up to six controllers connected through correctly terminated PCA9615 links.
- Transfer buffers remain valid until `mmi2c_transfer()` returns.
- `mmi2c_transfer()` and `mmi2c_release_bridge()` are task-only APIs.
- The optional recovery callback is called only after the full bus timeout.
- Hardware validation is still required for hot-swap timing and simultaneous
  controller behavior.

If two controllers transmit an identical address and identical data at the
same time, neither is guaranteed to detect arbitration loss. That is inherent
I2C behavior; make concurrently generated packets differ early.

See [the packet example](examples/apollo3_ios_packet/README.md) for the complete
FreeRTOS sensor-to-Apollo3 flow.

## Primary references

- [MSPM0 G-Series technical reference manual](https://www.ti.com/lit/pdf/slau846)
- [MSPM0 L-Series technical reference manual](https://www.ti.com/lit/pdf/slau847)
- [MSPM0 SDK examples guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/sdk_users_guide/doc_guide/doc_guide-srcs/examples_guide.html)
- [FreeRTOS task notifications](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/03-Direct-to-task-notifications/01-Task-notifications)
- NXP PCA9615 data sheet, Rev. 2
