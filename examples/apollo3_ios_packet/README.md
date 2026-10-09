# MSPM0 to Apollo3 IOS packet example

This bare-metal example captures sensor data after either a button interrupt or
a periodic timer interrupt, serializes it into an eight-byte-maximum packet,
and writes it into the Apollo3 IOS direct area. No RTOS or dynamic allocation
is used.

## Wire protocol

The serialized packet is independent of C structure padding:

| IOS offset | Packet byte |
| --- | --- |
| `0x50` | `module_id` |
| `0x51` | `message_type` |
| `0x52` | `sequence_number` |
| `0x53` | `payload_len` |
| `0x54..0x57` | zero to four payload bytes |

After every packet byte has been acknowledged, the sender performs a separate
write of value `0x80` to IOS offset `0x01`.

For the Apollo3 IOS host protocol, bit 7 of the one-byte instruction selects a
write and bits 6:0 contain the logical LRAM offset. Consequently the logical
offsets `0x50`, `0x57`, and `0x01` appear on the wire as instruction bytes
`0xD0`, `0xD7`, and `0x81`, respectively.

An I2C write includes the IOS offset as its first byte. The multi-controller
library limits a complete transfer to eight data bytes, so one transfer can
carry at most seven packet bytes. An eight-byte packet is therefore sent as:

1. `[0xD0, packet bytes 0..6]` (logical offset `0x50`)
2. `[0xD7, packet byte 7]` (logical offset `0x57`)
3. `[0x81, 0x80]` (logical offset `0x01`, data value `0x80`)

The PCA9615 remains powered and enabled across all three transactions, then is
disconnected and powered down after the peripheral reports IDLE.

## Delivery mechanism

`ios_packet_sender_process()` is a nonblocking state machine. It provides the
following guarantees on the MSPM0 side:

- ISR code only queues a request; sensor access and I2C submission occur in the
  main loop.
- One call drains every immediate state transition. In particular, accepting a
  request, reading the sensor, serializing the packet, and submitting its first
  I2C chunk do not depend on a later SysTick or other unrelated interrupt.
  Processing returns only when it reaches an I2C wait, retry deadline, bridge
  release wait, an empty request queue, or a preparation failure.
- The request counter's critical section restores the previous PRIMASK before
  sensor acquisition. A sensor, payload-length, or serialization failure is
  recorded in `last_error` and returns explicitly to `IDLE`, so later and
  already-queued button/timer requests continue to run.
- The byte offset advances only after the relevant I2C transaction completes
  with `MMI2C_RESULT_OK`.
- A failed chunk is resent from the same offset, with up to three retries after
  the initial attempt. Each attempt still has the lower library's independent
  randomized arbitration-loss retries.
- The trigger register is never written until all data chunks succeed.
- The sequence number advances only after the trigger write succeeds and the
  bridge is safely released.
- `packets_completed`, `packets_failed`, `last_error`, `last_i2c_result`, retry
  state, and dropped request counts are retained for debugger/telemetry
  inspection.

An I2C ACK proves that the Apollo3 IOS peripheral accepted each byte; it does
not prove that Apollo3 firmware consumed the packet. Strict end-to-end delivery
requires an Apollo3-to-MSPM0 acknowledgement containing at least the sequence
number. That response is outside this example because no acknowledgement
register or format was specified.

## Assumptions requiring confirmation on the Apollo3

1. The unshifted seven-bit IOS address is `0x10`. Change `APP_IOS_ADDRESS` if
   the target uses another address.
2. IOS LRAM is configured with a direct-write region that includes `0x01` and
   `0x50..0x57`. An Ambiq configuration with `FIFOBase = 0x80` satisfies the
   data-region requirement. Sequential host writes auto-increment the IOS LRAM
   offset; retransmitting the same bytes at the same offsets is idempotent.
3. Register-access interrupt `AM_HAL_IOS_ACCESS_INT_01` is enabled by the
   Apollo3 firmware. Writing `0x80` to LRAM offset `0x01` does not create the
   desired CPU interrupt unless that register-access interrupt is configured.
4. The Apollo3 ISR treats `0x80` at `0x01` as the packet-ready indication and
   reads `payload_len + 4` bytes beginning at `0x50`.
5. The Apollo3 reads or copies the packet before accepting another packet. The
   current example sends one packet at a time but does not wait for a remote
   consumption acknowledgement.
6. The Apollo3 de-duplicates a repeated trigger carrying the same sequence
   number. A timeout can be ambiguous: the target may have accepted the trigger
   even if the controller did not observe successful completion.
7. `module_id` and a stable factory-derived arbitration seed are unique among
   simultaneously active modules.
8. Multi-byte sensor fields in the sample payload are little-endian. The four
   packet header fields are single bytes and have no endianness issue.
9. Button debounce is implemented in hardware or in the product's input layer;
   every interrupt in this example queues one sample request.

## MSPM0 SysConfig setup

Create an empty DriverLib project for either MSPM0G3519 or MSPM0L1116 and add
`src/mmi2c.c`, `include/mmi2c.h`, and the four `.c/.h` example files.

Configure these SysConfig components:

- `I2C_0`: controller mode, 400 kHz, TX/RX FIFO threshold one byte. Do not
  enable DMA for this I2C instance.
- `GPIO_BUTTON`: one falling-edge interrupt input named `BUTTON`, normally with
  a pull-up.
- `GPIO_BRIDGE`: two initially-low outputs named `EN` and `VDDB`. `VDDB` drives
  the external power-switch control, not the PCA9615 EN pin.

The aliases in `app_config.h` follow those component names. Adjust that one
file if SysConfig generates different symbols or assigns a different GPIO
group IRQ handler. Set the application I2C address, module ID, message type,
sample period, and unique random seed there as well.

The example uses a 1 ms SysTick for clarity. This quantizes timeouts/backoff to
1 ms and wakes the MCU periodically. For production low-power firmware,
replace `app_now_us()` and the periodic request source with a continuously
running low-power timer or RTC/alarm arrangement while preserving a monotonic
microsecond-compatible counter.

`sensor_stub.c` emits two deterministic 16-bit values. Replace
`sensor_read_payload()` with the real sensor driver; the callback runs in the
main loop and must return a payload length no greater than four.

## Bench checks

1. Inspect SDA/SCL and verify the expected `0xD0`, `0xD7`, then `0x81` wire
   instruction sequence for a four-byte payload.
2. Force a NACK during each phase and confirm that the exact failed phase is
   retried, without incrementing `bytes_sent` or the sequence number.
3. Start two modules together and confirm arbitration loss is handled within
   the multi-controller library before the application retry counter changes.
4. Confirm EN goes high only after VDD(B) has been on for at least 12 ms and
   that EN goes low before VDD(B) is switched off.
5. Verify the Apollo3 interrupt handler sees offset `0x01`, validates
   `payload_len`, and reads the matching sequence number from offset `0x52`.
