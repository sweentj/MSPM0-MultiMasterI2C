# MSPM0 multi-controller I2C library

This is a small, allocation-free, interrupt-driven controller library for the
MSPM0 I2C peripheral. It supplies the transaction and retry behavior missing
from TI's higher-level I2C driver while using TI DriverLib as the portable
register layer. It compiles for both MSPM0G3519 and MSPM0L1116.

It supports:

- multiple I2C controllers on one bus;
- arbitration-loss detection and the recovery sequence required by the MSPM0
  technical reference manual;
- randomized binary exponential retry backoff;
- write, read, and write/repeated-START/read transactions with an eight-byte
  total message limit;
- NACK, bus-idle, and transfer timeouts;
- optional PCA9615 power/enable hooks;
- an optional board-specific stuck-bus recovery hook; and
- operation without an RTOS or dynamic allocation.

## Important terminology

Recent I2C specifications use **controller/target**. Older material uses
**master/slave**. "Multi-controller" here is the same electrical behavior as
"multi-master".

## Assumptions

1. MSPM0 SDK 2.11.00.07 (or a DriverLib version with the same I2C API) is used.
2. SysConfig initializes the I2C clock, pins, FIFO thresholds, and controller.
   This library does not own the pin mux or system clock.
3. Addresses passed to the API are unshifted 7-bit addresses. Ten-bit
   addressing is intentionally rejected.
4. Every controller uses open-drain-compatible I2C pins and the single-ended
   side of each PCA9615 has suitable pull-ups. No device actively drives either
   bus line high.
5. The bus is configured for 400 kHz Fast-mode operation, with at most six
   MSPM0 controllers connected through PCA9615 buffers.
6. `now_us()` is monotonic modulo 2^32 and is safe to call in an ISR. Configured
   timeouts must be less than 2^31 microseconds.
   `rx_done_settle_us` must span at least two clocks of the configured I2C
   functional clock; 2 us is conservative when that clock is at least 1 MHz.
7. The application calls `mmi2c_process()` often enough to enforce its timeout
   requirements, including while waiting for another controller to finish.
8. The selected I2C IRQ is enabled in the NVIC, and its ISR calls
   `mmi2c_handle_irq()`.
9. A transfer's buffers stay allocated and unchanged until
   `mmi2c_take_result()` returns its terminal result.
10. There is one `mmi2c_t` context per I2C hardware instance. API calls that
    mutate a context are made from one application execution context. The I2C
    ISR may run concurrently as designed; do not submit from the ISR.
11. A complete API transaction contains at most 64 data bits: `tx_length +
    rx_length <= 8`. The address, START, ACK/NACK, and STOP bits are not counted
    in that limit. Override `MMI2C_MAX_MESSAGE_BYTES` at compile time only if
    this product constraint changes.
12. PCA9615 VDD(B) and EN are controlled separately. The library's
    `bridge_acquire()` callback owns their sequencing; the I2C state machine
    never manipulates board GPIOs directly.
13. Bus recovery is not attempted after ordinary arbitration loss. Recovery is
    only considered after the entire bus-idle timeout, because clocking a bus
    that another valid controller owns would corrupt its transaction.

## SysConfig setup

For the chosen I2C instance:

1. Configure it as a controller at 400 kHz.
2. Enable the controller peripheral and configure SDA/SCL pins.
3. Select TX and RX FIFO thresholds of one byte. The library also drains all
   available RX bytes each time it is interrupted.
4. Do not enable DMA for the same I2C instance.
5. Let SysConfig generate `ti_msp_dl_config.c/.h`.
6. Add `include/mmi2c.h` and `src/mmi2c.c` to the project.
7. Call `mmi2c_init()` after `SYSCFG_DL_init()`, then enable the instance's NVIC
   interrupt.

`mmi2c_init()` explicitly sets `MCR.MMST`, which makes the SCL high-period timer
start only after SCL is actually observed high. It also enables RX/TX done,
FIFO, NACK, STOP, and arbitration-loss peripheral interrupts.

The implementation also accounts for current G351x and L111x I2C errata:

- I2C_ERR_07: each transfer START is programmed with one complete MCTR update;
- I2C_ERR_08: the final RX FIFO read is deferred for the configured interval,
  which must be at least two I2C functional-clock cycles;
- I2C_ERR_13: completion uses peripheral interrupts rather than immediately
  polling BUSY after BURSTRUN; and
- I2C_ERR_05: the library never toggles controller ACTIVE during a transfer.

## Minimal integration

See `examples/lp_mspm0g3519_example.c`. The essential ISR is:

```c
void I2C_0_INST_IRQHandler(void)
{
    mmi2c_handle_irq(&g_i2c);
}
```

The application submits a transfer and continues to call `mmi2c_process()`:

```c
mmi2c_transfer_t transfer = {
    .address = TARGET_ADDRESS,
    .tx_data = packet,
    .tx_length = packet_length,
};

if (mmi2c_submit(&g_i2c, &transfer) == MMI2C_RESULT_IN_PROGRESS) {
    /* Continue the application; IRQs move the bytes. */
}
```

When the result stops being `MMI2C_RESULT_IN_PROGRESS`, consume it with
`mmi2c_take_result()`. A completed context deliberately remains unavailable
until the result is taken so an error cannot be silently overwritten.

## Arbitration behavior

On arbitration loss, hardware releases the bus. The library then follows TI's
documented sequence:

1. mask the TX FIFO trigger and TX-empty sources;
2. clear those pending interrupt flags;
3. flush the TX FIFO;
4. do **not** generate STOP, because the winning controller owns the bus;
5. wait for the winner's STOP, controller IDLE, and `ARBLST` to clear;
6. wait a randomized backoff; and
7. refill the FIFO and retry the complete application transaction.

Use a different nonzero `random_seed` on every module. A stable hash of a
factory serial number is suitable. Identical seeds and identical activation
timing can cause repeated collisions. Initialization rejects a zero seed or a
zero backoff slot when arbitration retries are enabled.

NACK retries default to zero in the example. NACK is usually a protocol or
target-readiness problem, not contention, so retrying it automatically can hide
a fault. Arbitration loss is the only condition that normally merits automatic
retry.

## Identical-transfer limitation

If two modules transmit the same target address and identical eight-byte data
at the same instant, neither is guaranteed to observe arbitration loss because
both are driving the same bit values. This is inherent I2C behavior rather than
a library defect. Ensure concurrently transmitted data differs early, or handle
the ambiguity above this library.

## PCA9615 VDD(B) and EN sequencing

The PCA9615 hot-swap behavior must be treated separately from MSPM0 arbitration:

- Begin with EN low, switch VDD(B) on, and wait at least 11 ms before asserting
  EN. Using 12 ms provides modest timer and supply-ramp margin.
- After EN is asserted, allow the PCA9615 bus-idle qualification interval. The
  data sheet specifies 100 us for idle detection; 200 us is a conservative
  software allowance when no STOP is observed.
- EN high does not by itself guarantee immediate connection. Connection also
  depends on a valid, terminated differential bus and an idle/STOP condition.
- On shutdown, deassert EN before switching VDD(B) off.

`bridge_acquire()` is intentionally synchronous and board-owned: it asserts
power/EN and returns only when a transaction may safely start. If the device is
kept powered and connected, omit both bridge callbacks.

For this power-sensitive design, set `release_bridge_when_done = true`. Release
is deferred until hardware
reports both controller IDLE and bus-not-busy, so the PCA9615 cannot be
disconnected before the transaction's STOP condition reaches the bus. Continue
calling `mmi2c_process()` after observing a result; the normal main-loop pattern
in the example already does this.

## Stuck-bus recovery

The optional `bus_recover()` callback exists because generic recovery cannot
safely change the SysConfig-owned I2C pins. A board implementation should:

1. first establish that all other controllers have exceeded the agreed maximum
   transaction duration;
2. disconnect or disable the local PCA9615 if appropriate;
3. temporarily mux the local single-ended pins to open-drain GPIO;
4. clock SCL up to nine times while observing SDA;
5. generate STOP only if the local segment can be controlled without disturbing
   a valid differential transaction;
6. restore the peripheral pin mux and I2C state; and
7. return true only if both lines and the hardware BUSBSY indication recover.

Do not enable this hook until the recovery policy has been validated with all
module power-state combinations.

## Bench validation plan

Use the LP-MSPM0G3519 as one controller and at least one second controller. A
target is also required; a logic analyzer should observe both single-ended and,
if possible, differential sides.

1. Verify one-module write, read, and combined write/read at 400 kHz using the
   maximum eight-byte total transaction.
2. Start two different module-ID packets on the same timer edge. Confirm exactly
   one arbitration-lost interrupt, no STOP from the loser, and later retry.
3. Sweep relative start time through at least one full byte at 400 kHz.
4. Send identical prefixes and then differing module IDs. Confirm arbitration
   occurs on the first differing `1` versus `0` bit.
5. Repeat simultaneous-start testing with all six controllers and verify that
   randomized backoff eventually completes every transfer.
6. Send completely identical frames and document the expected inherent I2C
   ambiguity.
7. Hold the target in reset to test address NACK. Hold SCL low to test transfer
   timeout. Hold SDA low while idle to test bus-idle timeout.
8. Cycle each PCA9615 VDD(B) and EN independently and measure connection time
   and current.
9. Repeat collision tests while different module buffers are connecting.
10. Run long-duration randomized traffic and track attempts, arbitration
    losses, NACKs, and timeouts.

## Current scope limits

- No 10-bit addressing, SMBus PEC, DMA, RTOS locking, target mode, or General
  Call support.
- The library does not configure clocks, pins, NVIC priority, or PCA9615 GPIOs.
- Hardware bench validation is still required. Compilation alone cannot prove
  behavior through the PCA9615 hot-swap state machine.

## Primary references

- Texas Instruments, MSPM0 G-Series 80-MHz Microcontrollers Technical Reference
  Manual, I2C arbitration and multiple-controller sections (SLAU846).
- Texas Instruments, MSPM0 SDK DriverLib `dl_i2c.h` and the
  `i2c_multicontroller_arbitration` example.
- NXP, PCA9615 data sheet, hot-swap/power-on sequence and timing (Rev. 2,
  16 September 2021).
