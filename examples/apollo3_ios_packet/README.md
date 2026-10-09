# FreeRTOS Apollo3 IOS packet sender

This example has one packet task and one one-entry request queue. A button or
timer ISR queues a request. The packet task wakes, reads the sensor, serializes
one packet, performs the I2C writes, and blocks again.

There is no application state machine, polling function, SysTick callback, or
custom timebase.

## Task flow

```text
button/timer ISR
    -> xQueueOverwriteFromISR()
    -> packet task wakes
    -> read sensor
    -> serialize packet
    -> write IOS data at 0x50...
    -> write 0x80 at 0x01
    -> power down PCA9615
    -> block on queue
```

The queue length is one, so repeated pending interrupts are coalesced into one
request. This prevents an input fault or button bounce from building an
unbounded backlog.

## Packet format

```c
typedef struct {
    uint8_t module_id;
    uint8_t message_type;
    uint8_t sequence_number;
    uint8_t payload_len;
    uint8_t payload[MAX_PAYLOAD_LEN];
} packet_t;
```

The struct is serialized explicitly, so compiler padding is never transmitted:

| IOS offset | Packet byte |
| --- | --- |
| `0x50` | `module_id` |
| `0x51` | `message_type` |
| `0x52` | `sequence_number` |
| `0x53` | `payload_len` |
| `0x54..0x57` | zero to four payload bytes |

Apollo3 IOS uses bit 7 of its instruction byte as the write bit. The logical
registers therefore appear on the bus as `0xD0` for `0x50` and `0x81` for
`0x01`.

The I2C library permits eight transmitted data bytes per transaction. Because
the IOS instruction consumes one byte, an eight-byte packet is sent as:

1. `[0xD0, packet bytes 0..6]`
2. `[0xD7, packet byte 7]`
3. `[0x81, 0x80]`

The PCA9615 remains enabled for all three transfers. The trigger write occurs
only if every data write returns `MMI2C_RESULT_OK`. The sequence number advances
only after the trigger succeeds and the bridge is safely released.

## ISR usage

The button handler is intentionally short:

```c
void APP_BUTTON_IRQHandler(void)
{
    if (button_interrupt_is_pending()) {
        clear_button_interrupt();
        ios_packet_sender_request_from_isr(&g_sender);
    }
}
```

A periodic hardware-timer ISR uses the same request call after clearing its
own interrupt flag. Sensor access and I2C transfers never run in an ISR.

## MSPM0 project setup

Start from the MSPM0 SDK `empty_freertos` example for LP-MSPM0G3519 or
LP-MSPM0L1117 and add the files from this repository.

Configure SysConfig components:

- `I2C_0`: controller, 400 kHz, TX/RX FIFO threshold one byte, DMA disabled.
- `GPIO_BUTTON`: falling-edge input interrupt named `BUTTON`.
- `GPIO_BRIDGE`: initially-low outputs named `EN` and `VDDB`.

No application timebase timer is required. FreeRTOS supplies transfer
timeouts, arbitration backoff, bridge delays, and task blocking.

The I2C and button IRQ priorities must be valid for FreeRTOS `FromISR` APIs.
`app_task()` enables both IRQs after the scheduler starts, then becomes the
packet-sender task.
The example uses static task, queue, and mutex allocation, so enable
`configSUPPORT_STATIC_ALLOCATION`, `configUSE_MUTEXES`, and
`configUSE_TASK_NOTIFICATIONS` in `FreeRTOSConfig.h`.

The example's task stack is 256 `StackType_t` words. Measure the stack high-
water mark with the real sensor driver and increase it if necessary.

## Apollo3 assumptions

1. The seven-bit IOS address is `0x10` unless changed in `app_config.h`.
2. IOS LRAM direct writes include `0x01` and `0x50..0x57`.
3. Register-access interrupt `AM_HAL_IOS_ACCESS_INT_01` is enabled.
4. The Apollo3 treats value `0x80` at `0x01` as packet-ready.
5. It reads `payload_len + 4` bytes beginning at `0x50`.
6. It consumes or copies a packet before another packet is accepted.
7. It de-duplicates a repeated trigger carrying the same sequence number.

An I2C ACK proves that IOS accepted the bytes; it does not prove that Apollo3
firmware consumed them. Strict end-to-end delivery requires a separate Apollo3
acknowledgement containing the sequence number.

## Bench checks

1. Confirm the expected `0xD0`, `0xD7`, and `0x81` instruction sequence.
2. Force a NACK and verify that configured library retries repeat the same IOS
   register without advancing the packet sequence number.
3. Start multiple modules together and confirm the loser waits for STOP before
   randomized retry.
4. Measure VDD(B)-to-EN delay and confirm EN falls before VDD(B) is removed.
5. Confirm no trigger write occurs after an exhausted data-write failure.
