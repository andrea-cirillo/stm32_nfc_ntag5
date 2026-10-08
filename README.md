# NTAG 5 NFC Monitor — P-NUCLEO-WB55 + FreeRTOS

Firmware for the **STM32WB55** (P-NUCLEO-WB55 board) that monitors an **NXP NTAG 5**
NFC tag over **I2C**. Every time an NFC reader, such as a smartphone, **reads** or
**writes** the tag over RF, the STM32 detects the operation, acquires the tag memory
through **I2C + DMA** and prints a decoded report on the serial port.

---

## 1. Goal

The NTAG 5 (NTP53x2 / NTA53x2) is a dual-interface NFC Forum Type 5 tag (ISO 15693).
It has an RF interface towards NFC readers and an I2C interface towards a host MCU.
It is **not** an NFC reader: it cannot read other tags. Its job is to bridge the two
worlds. Data written by a phone over RF becomes readable by the MCU over I2C, and the
other way round.

This project implements the **RF reader → I2C host** direction:

| Requirement | How it is met |
|---|---|
| Detect when a reader approaches and operates on the tag | ED (Event Detection) pin of the NTAG 5 on an EXTI interrupt, plus RF-field sampling |
| Acquire the tag content over I2C using DMA | `HAL_I2C_Master_Seq_Receive_DMA` on I2C1 / DMA1_Channel1 |
| One serial message for every read and every write | Each reader transaction is classified as READ or WRITE by comparing the memory with the previous snapshot |
| Decode the content | Built-in NDEF parser (Text, URI, MIME and other records) |
| FreeRTOS-based, non-blocking | Two static tasks, a queue, semaphores; the CPU never busy-waits on I2C |

---

## 2. Hardware

| Item | Details |
|---|---|
| MCU board | ST **P-NUCLEO-WB55** (STM32WB55RG, Cortex-M4 @ 64 MHz) |
| NFC tag board | NXP **NTAG 5** development board with Arduino-compatible header |
| Serial console | ST-LINK Virtual COM port (USART1, PB6/PB7), **115200 8N1** |

### Wiring (Arduino header)

| NTAG 5 signal | Arduino pin | STM32 pin | Notes |
|---|---|---|---|
| SCL | D15 | PB8 | I2C1, 100 kHz |
| SDA | D14 | PB9 | I2C1 |
| ED | D5 | **PA15** | EXTI15 (EXTI15_10_IRQn); open-drain, internal pull-up enabled |
| VCC | 3V3 | — | VCC is required: the SRAM and the I2C interface need it |
| GND | GND | — | |

The NTAG 5 answers on the default 7-bit I2C address **0x54**.

---

## 3. Software architecture

### 3.1 Project layout

```
NFC/
├── App/                     <- all NFC application code (this project)
│   ├── Inc/
│   │   ├── ntag5.h          NTAG 5 I2C driver: registers, memory, modes, DMA reads
│   │   ├── ndef.h           NDEF parser for Type 5 tags
│   │   ├── nfc_app.h        application configuration, event types, public API
│   │   ├── nfc_report.h     serial report formatter
│   │   └── app_log.h        thread-safe UART logger
│   └── Src/                 matching implementations
├── Core/                    STM32CubeMX generated code (main.c, HAL MSP, IRQs)
├── Drivers/                 STM32WBxx HAL + CMSIS
├── Middlewares/             FreeRTOS (CMSIS-RTOS v2)
└── NFC.ioc                  CubeMX configuration
```

`Core/` stays exactly as CubeMX generates it. The only hook is in `Core/Src/main.c`,
inside the `USER CODE` sections, so it survives a regeneration:

```c
/* USER CODE BEGIN Includes */
#include "nfc_app.h"
/* USER CODE BEGIN RTOS_THREADS */
NfcApp_Init(&hi2c1, &huart1);
```

`.cproject` already lists `App` as a source folder and `../App/Inc` as an include
path (Debug and Release).

### 3.2 Layers

```
 +-----------------------------------------------------------------------+
 |  nfc_report.c      formats events -> human-readable serial report      |
 +-----------------------------------------------------------------------+
 |  nfc_app.c         FreeRTOS tasks, ED/EXTI, RF-field sessions,         |
 |                    transaction classification (READ / WRITE / SRAM)    |
 +---------------------------+-------------------------------------------+
 |  ndef.c  NDEF parser      |  app_log.c  thread-safe UART logger       |
 +---------------------------+-------------------------------------------+
 |  ntag5.c           NTAG 5 driver: READ/WRITE REGISTER, memory reads,   |
 |                    IT + DMA transfers, chunking, bus recovery, diag    |
 +-----------------------------------------------------------------------+
 |  STM32 HAL (I2C1 + DMA1_Channel1, GPIO/EXTI, USART1)  |  FreeRTOS      |
 +-----------------------------------------------------------------------+
```

| Module | Responsibility |
|---|---|
| `ntag5` | Hardware abstraction of the NTAG 5. It knows the memory map and the session registers, and implements the I2C command formats. Transfers run in interrupt or DMA mode, and the calling task blocks on a semaphore released by the HAL callbacks. A mutex protects the bus. It also provides error mapping, bus recovery and diagnostic counters. |
| `ndef` | Pure C parser with no allocation and no HAL. It parses the Capability Container, scans the TLVs and iterates over NDEF records. Unit-tested on the host. |
| `nfc_app` | Application logic and FreeRTOS resources. Detects reader activity, acquires memory and SRAM, classifies each operation and publishes events on a queue. |
| `nfc_report` | Turns each event into readable serial output. It is the default implementation of the weak hook `NfcApp_OnEvent()`. |
| `app_log` | `printf`-style logging with a millisecond timestamp, serialized by a mutex, plus a hexdump helper. |

### 3.3 FreeRTOS objects (all statically allocated)

| Object | Type | Priority / size | Role |
|---|---|---|---|
| `nfcTask` | task | AboveNormal, 2 KB stack | ED edges, RF-field sampling, I2C acquisitions, classification |
| `nfcReport` | task | Normal, 3 KB stack | Drains the event queue and prints the reports |
| event queue | queue | 4 × `nfc_event_t` | Decouples acquisition from the slow UART output |
| ED semaphore | binary semaphore | — | Given by the EXTI ISR on each ED falling edge |
| I2C done | binary semaphore | — | Given by the HAL I2C callbacks (TX/RX complete, error) |
| I2C mutex / log mutex | mutex | — | Serialize the bus and the UART |

Interrupt priorities are numerically ≥ 5, which is `configMAX_SYSCALL_INTERRUPT_PRIORITY`,
so the ISRs can safely call FreeRTOS `FromISR` APIs: I2C1 EV/ER and DMA1_Channel1 at
priority 5, EXTI15_10 at priority 6.

---

## 4. How it works

### 4.1 Start-up sequence

1. **Probe:** the firmware waits until the NTAG 5 acknowledges address 0x54.
2. **Configure (session registers):**
   - `CONFIG_1` = normal arbiter mode, SRAM enabled (`0x03` under mask `0x0F`).
   - The ED event is disabled during the self-test.
3. **ED self-test:**
   - The NTAG 5 ED event is set to **software interrupt** (`1101b`), which drives ED low immediately. The firmware checks that PA15 reads 0.
   - It then releases ED through `ED_INTR_CLEAR`, trying **10ABh** (datasheet) and then **10ACh** (AN12364), and keeps the address that works.
   - If PA15 never sees ED, the firmware says so and relies on the RF field only.
4. **Final ED event:** `ED_CONFIG` = **Start of command** (`1010b`). ED goes low at the start of every RF command and is released automatically at the end of the response.
5. **Reference snapshot:** the NDEF area and the SRAM are read via DMA and printed as "Current tag content".

### 4.2 Detecting reader operations

Two complementary mechanisms run in `nfcTask`.

**A) RF command counting (ED pin, primary)**

```
ED (PA15)  ‾‾‾|_|‾|_|‾‾|_|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_|‾‾‾‾‾‾‾‾
              1   2    3   <- quiet >= 80 ms ->  presence check
              └── transaction (3+ commands) ─┘   (1 command: ignored)
```

1. Every falling edge on PA15 raises an EXTI interrupt. The ISR increments a command counter and gives the ED semaphore.
2. The task wakes on the first edge and waits until no new edge arrives for `NFC_APP_QUIET_MS` (80 ms). That closes one **reader transaction**.
3. Transactions with fewer than `NFC_APP_MIN_CMDS` (3) commands are the phone's periodic **presence checks** and are ignored.
4. Any other transaction is acquired and classified (§4.3).

**B) RF field sessions (STATUS0.NFC_FIELD_OK, safety net)**

1. Every `NFC_APP_FIELD_POLL_MS` (50 ms) the task reads `STATUS0`.
2. A **session** starts when the field appears and ends after `NFC_APP_FIELD_OFF_MS` (600 ms) of continuous absence. The long timeout merges the short field gaps that phones produce even when held still.
3. If a session of at least `NFC_APP_MIN_SESSION_MS` (100 ms) ends without any READ/WRITE message, the memory is acquired and classified anyway. This guarantees **one message per approach** even if the ED pin is missing or inactive.
4. In verbose mode a one-line session summary is printed: field duration, ED edges counted, STATUS1 bits seen.

### 4.3 Acquisition and classification

At the end of a transaction (or of a session without messages):

1. **NDEF area via I2C + DMA:**
   - First the 16-byte header: Capability Container plus the start of the TLV.
   - From the NDEF TLV length, it computes how many bytes the message really uses.
   - It reads only those bytes, up to `NFC_APP_TAG_READ_SIZE` = 256.
2. **SRAM via I2C + DMA:** 256 bytes, when `NFC_APP_WATCH_SRAM` = 1.
3. **Comparison with the previous snapshot:**

| Result | Event | Serial output |
|---|---|---|
| NDEF area changed | `NFC_EVT_TAG_WRITE` | **Tag WRITE** + decoded new content |
| SRAM changed | `NFC_EVT_SRAM_WRITE` | **SRAM WRITE (RF → I2C data)** + hexdump/text |
| nothing changed | `NFC_EVT_TAG_READ` | **Tag READ** + decoded content |

The snapshot is then updated, so the next comparison is against the latest content.

### 4.4 I2C driver details

**Command formats** (block addressing, 4 bytes per block):

```
READ memory    : S | 0xA8 | MEMA_H | MEMA_L | Sr | 0xA9 | D0..Dn | P
WRITE memory   : S | 0xA8 | MEMA_H | MEMA_L | D0 D1 D2 D3 | P
READ REGISTER  : S | 0xA8 | MEMA_H | MEMA_L | REGA | Sr | 0xA9 | REGDAT | P
WRITE REGISTER : S | 0xA8 | MEMA_H | MEMA_L | REGA | MASK | REGDAT | P
```

**Transfers**
- The address phase uses `HAL_I2C_Master_Seq_Transmit_IT` (FIRST_FRAME).
- The data phase uses `HAL_I2C_Master_Seq_Receive_DMA` (LAST_FRAME) with a repeated START.
- Reads of 16 bytes or more use DMA. Register accesses use interrupt mode.
- **Chunking:** long reads are split into 64-byte transactions (`NTAG5_READ_CHUNK`). Above 255 bytes the STM32 I2C peripheral needs RELOAD mode, which the sequential DMA path did not handle reliably on this setup.

**Error handling**
- NACK is retried; the arbiter may be busy serving the RF side.
- On a bus error or timeout, the driver clocks SCL 9 times, generates a STOP and re-initializes I2C1.
- A DMA chunk that fails is retried in interrupt mode, and a counter records it (`dma_fallback_it`).

### 4.5 Why the arbiter stays in normal mode

The NTAG 5 also offers an **SRAM pass-through** mode with a hardware handshake. Testing
on the hardware showed that in NFC→I2C pass-through the arbiter locks the I2C interface
out of the EEPROM until the reader fills the SRAM: I2C reads are NACKed and STATUS1
bit 0 = 1.

Because the main use case is reading what phones write as standard NDEF records into
the EEPROM, the application keeps the arbiter in **normal mode**. Both EEPROM and SRAM
stay readable, and SRAM writes from the reader are detected by comparison. The driver
still exposes the pass-through API (`ntag5_set_mode(NTAG5_MODE_PT_NFC_TO_I2C, …)`,
`ntag5_read_sram()`) for applications that need the handshake.

---

## 5. Serial output

Open the ST-LINK Virtual COM port at **115200 8N1**: Tera Term, PuTTY, or
`python -m serial.tools.miniterm COMx 115200`. Close any other program that holds the
port first.

Sample session (timestamps and values are illustrative):

```
       1 [I] NFC app: P-NUCLEO-WB55 + NTAG 5 (I2C 0x54)
      14 [I] [NFC] ED self-test on PA15 (Arduino D5): OK (release OK)
      14 [I] [NFC]     working ED_INTR_CLEAR register: 10ABh
      14 [I] [NFC] NTAG5 ready, ED = start of RF command. Bring an NFC reader close...
      42 [I] [TAG] Current tag content (20 B, I2C+DMA 3900 us, DMA ok=1 IT fallbacks=0):
      42 [I] [TAG]   CC E1 40 40 01 -> Type 5 v1.0, data area 512 B, R:free W:free
      42 [I] [TAG]   #1 Well-Known "U"  URI: http://www.nxp.com/nfc
--------------------------------------------------
    5120 [I] [RF ] #1 Tag READ  (9 RF commands in 62 ms, I2C+DMA acquisition 3900 us)
    5120 [I] [TAG]   #1 Well-Known "U"  URI: http://www.nxp.com/nfc
--------------------------------------------------
    9480 [I] [RF ] #2 Tag WRITE  (21 RF commands in 180 ms, I2C+DMA acquisition 4400 us)
    9480 [I] [TAG]   New content:
    9480 [I] [TAG]   #1 Well-Known "T"  lang=en  text: "hello"
   10150 [I] [RF ] reader session ended: field present ~1650 ms, ED edges=30, STATUS1 or=0xC0 and=0xC0
```

The first column is the time since boot in ms. `[I]`, `[W]` and `[E]` mark info,
warnings and errors.

### Quick test with a smartphone

1. Open an NFC app (NXP TagInfo, NXP TagWriter, NFC Tools) and hold the phone on the antenna. A **Tag READ** appears.
2. Move it away and back: a new READ appears with the next sequence number.
3. Write a Text or URI record with the app: a **Tag WRITE** appears with the new content.

Apps that keep the connection open (TagInfo, NFC Tools "Read" tab) give the most stable
field. The phone's own background scanning only pulses the field briefly, and those
pulses are filtered out.

---

## 6. Configuration (`App/Inc/nfc_app.h`)

| Macro | Default | Meaning |
|---|---|---|
| `NFC_APP_USE_ED_PIN` | 1 | Use the ED pin on EXTI (0 = RF-field sessions only) |
| `NFC_ED_GPIO_PORT` / `_PIN` | GPIOA / 15 | ED pin. If you change the EXTI line, also change `NFC_ED_EXTI_IRQn` and `NFC_ED_IRQHandler` |
| `NFC_APP_QUIET_MS` | 80 | RF silence that closes a reader transaction |
| `NFC_APP_MAX_BURST_MS` | 1500 | Maximum transaction length before it is processed anyway |
| `NFC_APP_MIN_CMDS` | 3 | Minimum RF commands for a real read/write |
| `NFC_APP_FIELD_POLL_MS` | 50 | RF-field sampling period |
| `NFC_APP_FIELD_OFF_MS` | 600 | Field absence that closes a reader session |
| `NFC_APP_MIN_SESSION_MS` | 100 | Shorter sessions are discovery pulses and are ignored |
| `NFC_APP_TAG_READ_SIZE` | 256 | Maximum NDEF bytes acquired |
| `NFC_APP_WATCH_SRAM` | 1 | Also detect RF writes into the SRAM |
| `NFC_APP_VERBOSE` | 1 | Print the per-session summary line |
| `NFC_APP_PERSIST_CONFIG` | 0 | Also store mode/ED event in the NTAG 5 configuration EEPROM |
| `NTAG5_READ_CHUNK` (`ntag5.c`) | 64 | Bytes per I2C/DMA read transaction |

### Custom processing

`NfcApp_OnEvent()` is a weak function. Define it in your own file to handle the events,
for example to forward the data to BLE or to act on a specific record:

```c
#include "nfc_app.h"

void NfcApp_OnEvent(const nfc_event_t *evt)
{
  if (evt->type == NFC_EVT_TAG_WRITE) {
    /* evt->data[0..evt->len-1] holds CC + NDEF message */
  }
}
```

It runs in the `nfcReport` task context, not in an ISR.

---

## 7. Build and flash

1. Import the project into **STM32CubeIDE**: *File → Open Projects from File System*.
2. Build with *Project → Build Project* (Debug or Release).
3. Flash and run with *Run → Debug* or *Run*, through the on-board ST-LINK.

Resource usage: about 58 KB of Flash and 19 KB of RAM. All application buffers and RTOS
objects are static, and the 4 KB FreeRTOS heap is used only by the CubeMX-generated
default task.

No CubeMX change is needed for the ED pin. PA15 is configured by `App/Src/nfc_app.c`
(input, pull-up, falling-edge EXTI, NVIC priority 6). If you assign PA15 or EXTI15_10 in
CubeMX later, remove the duplicate `EXTI15_10_IRQHandler` from `nfc_app.c`.

---

## 8. Limitations

- **Rewriting identical content** is reported as a READ, because classification is based on content changes.
- **Only the first `NFC_APP_TAG_READ_SIZE` bytes** of the NDEF area are acquired. Larger messages are reported as truncated.
- **The RF-field safety net** reports the operation when the session ends, about 600 ms after the phone is removed, not instantly.
- **The session-register configuration is volatile.** The firmware re-applies it if the NTAG 5 is power-cycled; use `NFC_APP_PERSIST_CONFIG` to make it permanent.

---

## 9. References

- NXP **NTP53x2** (NTAG 5 link/boost) datasheet, Rev. 3.3: memory map, session registers, ED events.
- NXP **AN12364**: NTAG 5 pass-through mode.
- NFC Forum **Type 5 Tag** specification and **NDEF** / RTD Text / RTD URI.
- ST **UM2435**: P-NUCLEO-WB55 user manual.
