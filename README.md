# 🚌 BusSim

> **A cost-effective open-source alternative to professional CAN/LIN bus interfaces** such as Vector CANalyzer/CANoe, PEAK PCAN, or Kvaser — built on a ~$2 STM32F103C8T6 Blue Pill board.

BusSim transmits and receives CAN frames — with a browser-based Web UI for live DBC-decoded traffic display and message transmission — and separately drives a LIN bus as master and/or slave-response node via CLI. Everything runs over a **USB CDC virtual COM port** — no UART adapter, no native drivers, no expensive licences.

---

## 💡 Motivation

Professional CAN/LIN bus tools — Vector CANalyzer, CANoe, PEAK PCAN, Kvaser — cost anywhere from **hundreds to thousands of dollars** and often require proprietary drivers or OS-specific software. BusSim was built to fill that gap for engineers, students, and hobbyists who need a capable CAN (and LIN) interface on a tight budget.

| | BusSim | Typical commercial tool |
|---|---|---|
| 💰 **Hardware cost** | ~$2–5 (Blue Pill + TJA1051); +MCP2004 for LIN | $200–$2000+ |
| 🚌 **Bus support** | CAN (Web UI + CLI) and LIN (CLI, master/slave-response) | Usually both, gated behind pricier tiers |
| 🔌 **Driver required** | None (USB CDC) | Vendor driver |
| 🌐 **UI** | Browser-based, no install (CAN) | Windows app / licence |
| 📋 **DBC decode** | ✅ built-in (CAN) | ✅ (paid licence) |
| 🔧 **Open source** | ✅ fully | ❌ proprietary |

---

## ✨ Features

| Feature | Detail |
|---------|--------|
| 🚗 **CAN bitrate** | 500 kbps (configurable in `MX_CAN_Init`) |
| 📦 **Default TX frame** | ID `0x123`, DLC 8, payload `DE AD BE EF 00 00 00 00`, 10 Hz |
| 🚌 **LIN bitrate** | 19200 baud (LIN 2.x default), master + slave-response node via MCP2004 |
| 🔌 **Host interface** | USB CDC virtual COM port (any baud setting is ignored) |
| ⌨️ **CLI commands** | `HELP STATUS CAN LIN SET LISTEN STATS` |
| 👁️ **RX display** | Received CAN and LIN frames printed when `LISTEN ON` |
| 🔀 **Pin conflict workaround** | CAN remapped to PB8/PB9 via AFIO; RX routed to FIFO1 to avoid shared USB/CAN IRQ |
| 🌐 **Web UI** | Single-file HTML app — loads DBC, decodes live traffic, transmits frames via Web Serial |
| 📋 **Demo DBC** | Automotive demo file (Engine, ABS, TCU, BMS, UDS) matching real frame formats |
| 🔌 **Arduino DUT** | Arduino Nano + MCP2515 sketch simulating a full drive-cycle for bench testing |

---

## 🔧 Hardware

| Part | Notes |
|------|-------|
| 🟦 STM32F103C8T6 Blue Pill | Main simulator board |
| 📡 TJA1051T CAN transceiver | 5 V bus; **tie S pin to GND** for normal mode; PB8 is 5 V-tolerant |
| 🚌 MCP2004 LIN transceiver | TXD/RXD on PA9/PA10 (USART1, hardware LIN mode); CS on PB1; VBB is a separate 12 V bus supply, not 3.3 V/5 V logic |
| 🔌 USB Mini-B cable | CDC virtual COM port to host |
| 🔩 120 Ω termination resistors | One at each physical end of the CAN bus |
| 🔩 1 kΩ + diode master pull-up | LIN bus, from LBUS pin to VBB (required at whichever node is currently mastering) |

> See [Tools/wiring_diagram.md](Tools/wiring_diagram.md) for full pin-out, block diagram, and TJA1051/MCP2004 connection details.

---

## 🚀 Quick Start

### 1 – Clone and import into STM32CubeIDE

```bash
git clone <repo-url>
```

In STM32CubeIDE: **File → New → STM32 Project → From an Existing .ioc File**  
Browse to `BusSim.ioc` → Finish → click **Generate Code** (Alt+G).

> The `Drivers/`, `Middlewares/`, and `USB_DEVICE/` folders are included in the repo so the project builds immediately after import. The `Debug/` build output folder is intentionally excluded.

### 2 – Build & Flash

| Action | How |
|--------|-----|
| 🔨 Build | Ctrl+B or Project → Build All |
| ⚡ Flash via ST-Link | Run → Debug (OpenOCD) |
| ⚡ Flash via DFU | Hold BOOT0 high on power-up, use STM32CubeProgrammer |

### 3 – Connect

Open the USB CDC port in any terminal at any baud (e.g. 115200 8N1).  
Type `HELP` to see available commands, or open [Tools/webui.html](Tools/webui.html) in Chrome/Edge for the graphical interface.

---

## 💻 CLI Reference

```
HELP
  Show available commands

STATUS
  Print current configuration and state

CAN ON / CAN OFF
  Start or stop periodic CAN frame transmission

SET ID <hex>          e.g.  SET ID 0x456
  Set the 11-bit standard frame ID

SET DLC <1-8>         e.g.  SET DLC 4
  Set data length

SET DATA <hex bytes>  e.g.  SET DATA 01 02 03 04
  Set payload (also updates DLC to the byte count given)

SET RATE <1-100>      e.g.  SET RATE 20
  Set CAN TX rate in Hz

LIN ON / LIN OFF
  Start or stop periodic LIN master frame transmission

LIN MODE MASTER / LIN MODE SLAVE
  Select LIN role. Boots in SLAVE mode by default — run LIN MODE MASTER
  before LIN ON / LIN SEND will do anything. LIN is strictly single-master
  — SLAVE mode disables this node's own header generation (LIN ON / LIN
  SEND are blocked) so it can be bench-tested against a real external LIN
  master without bus contention. Slave auto-response (LIN RESP) and LISTEN
  work in both modes.

LIN SEND <id> <hex bytes>    e.g.  LIN SEND 0x10 01 02 03
  Send one LIN frame immediately (id is 6-bit, 0x00-0x3F)

LIN RESP <id> <hex bytes>    e.g.  LIN RESP 0x20 AA BB CC
  Register auto-response data: when a header for <id> appears on the bus and
  this node isn't the one driving it, it replies with the stored data

LIN RESP CLEAR <id>
  Remove a registered auto-response

SET LINID <hex>       e.g.  SET LINID 0x10
  Set the periodic LIN schedule's 6-bit identifier

SET LINDLC <1-8>      e.g.  SET LINDLC 4
  Set LIN payload length

SET LINDATA <hex bytes>  e.g.  SET LINDATA 01 02 03 04
  Set LIN payload (also updates LINDLC to the byte count given)

SET LINRATE <1-100>   e.g.  SET LINRATE 20
  Set periodic LIN TX rate in Hz

LISTEN ON / LISTEN OFF
  Enable or disable printing of received CAN and LIN frames

STATS
  Show CAN/LIN TX frame count, RX frame count, error counts

STATS RESET
  Reset all counters
```

### 📋 Example session

```
> STATUS
  TX:       OFF
  Listen:   OFF
  Bitrate:  500 kbps
  ID:       0x123
  DLC:      8
  Rate:     10 Hz
  Data:     DE AD BE EF 00 00 00 00

> SET ID 0x7DF
ID set to 0x7DF

> SET DATA 02 10 03 AA BB CC DD EE
Data set: 02 10 03 AA BB CC DD EE

> LISTEN ON
Listen ON

> CAN ON
CAN TX started

RX  ID:0x7DF  DLC:8  DATA: 02 10 03 AA BB CC DD EE

> STATS
  TX frames: 12
  RX frames: 12
  TX errors: 0
```

---

## 🌐 Web UI

Open **[Tools/webui.html](Tools/webui.html)** directly in Chrome or Edge (no server needed).

| Feature | Detail |
|---------|--------|
| 📂 **Load DBC** | Click Load DBC → select a `.dbc` file; signals appear in the sidebar |
| 🔗 **Connect** | Click Connect → pick the Blue Pill CDC port via the Web Serial picker |
| 📊 **Traffic view** | Last-Value mode (one row per ID, updated live) or Log mode (append every frame) |
| 🔍 **Signal decode** | Intel/Motorola byte order, factor/offset, unit; click any row to expand all signals |
| 📡 **Transmit** | Manual hex bytes or DBC-guided (select message, fill physical values, auto-encode) |
| 🖥️ **Console** | Raw serial I/O; "Show CAN frames" checkbox to opt-in to frame echo |

> Uses the **Web Serial API** — Chrome 89+ / Edge 89+ required. Not supported in Firefox or Safari.

---

## 📋 Demo DBC

**[Tools/demo.dbc](Tools/demo.dbc)** defines six automotive messages that the Arduino DUT transmits:

| ID | Name | Rate | Key Signals |
|----|------|------|-------------|
| `0x100` | EngineData | 10 ms | RPM (×0.25), ThrottlePos, CoolantTemp, EngineLoad |
| `0x200` | VehicleSpeed | 10 ms | VehicleSpeed (km/h ×0.01), WheelSpeed FL/FR/RL/RR |
| `0x300` | TransmissionData | 20 ms | GearPosition, SelectedGear, TransmissionTemp, Mode |
| `0x400` | BatteryStatus | 100 ms | BattVoltage, BattCurrent (signed), BattSOC, CellVolt Max/Min |
| `0x123` | BusSimHeartbeat | 100 ms | Counter, TestValue, Flags, Temperature, Voltage |
| `0x7DF` | DiagRequest | event | UDS ServiceID, SubFunction, DataIdentifier, RequestData |

---

## 🔌 Arduino DUT (Device Under Test)

**[Tools/CANSim_DUT/CANSim_DUT.ino](Tools/CANSim_DUT/CANSim_DUT.ino)** — Arduino Nano + MCP2515 sketch for bench testing.

### Hardware

| Pin | Signal |
|-----|--------|
| D10 | MCP2515 CS |
| D2 | MCP2515 INT |
| D11/D12/D13 | SPI (MOSI/MISO/SCK) |

### Library

Install **autowp/arduino-mcp2515** via the Arduino Library Manager (search `mcp2515`).  
Set `MCP_8MHZ` or `MCP_16MHZ` to match the crystal on your module.

### Behaviour

- Transmits `0x100 / 0x200 / 0x300 / 0x400` at configured rates with a simulated 80-second drive cycle (Idle → Accelerate → Cruise → Decelerate)
- Responds to UDS requests on `0x7DF`: Session Control (0x10), Tester Present (0x3E), Read Data By ID (0x22)
- Prints a status summary to Serial every 2 seconds

---

## 🔌 Arduino LIN DUT (LIN bus test harness)

**[Tools/LINSim_DUT_LIN/LINSim_DUT_LIN.ino](Tools/LINSim_DUT_LIN/LINSim_DUT_LIN.ino)** — a second, dedicated Arduino Nano + [LINTTL3](Tools/wiring_diagram.md) transceiver, acting as a LIN master test harness for the STM32's LIN side.

Minimal test harness (not a full simulated LIN network):
- `0x10` — self-contained master-response frame, ~1 Hz incrementing counter
- `0x20` — header-only frame; reads back a response with a timeout — use this to exercise the STM32 in `LIN MODE SLAVE` (register `LIN RESP 0x20 ...` first)
- `0x30` — the sketch passively auto-responds to this ID when it isn't the one driving the header — use this to exercise the STM32 in `LIN MODE MASTER` (`SET LINID 0x30` + `LIN SEND`/`LIN ON`)

There's no Arduino LIN library, so the sketch implements LIN 2.x framing (break, sync, PID parity, checksum) from scratch, mirroring `Core/Src/lin_handler.c`. Because the Nano's single hardware UART is dedicated to the LIN bus (needed for reliable break detection), the sketch reports status via the onboard LED's blink patterns rather than live Serial Monitor output — see the header comment in the `.ino` for the full pattern legend.

---

## 📁 Project Structure

```
BusSim/
├── BusSim.ioc                   CubeMX project — open in STM32CubeIDE
├── STM32F103C8TX_FLASH.ld       Linker script
├── Core/
│   ├── Inc/
│   │   ├── main.h               LED macros, Error_Handler declaration
│   │   ├── can_handler.h        CAN API
│   │   ├── lin_handler.h        LIN API
│   │   ├── cli.h                CLI API
│   │   └── usb_cdc_handler.h    USB CDC ring-buffer API
│   └── Src/
│       ├── main.c               Clock, GPIO, CAN/LIN init + main loop
│       ├── stm32f1xx_hal_msp.c  CAN GPIO / AFIO remap / NVIC setup
│       ├── stm32f1xx_it.c       USB & CAN interrupt vectors
│       ├── can_handler.c        CAN TX/RX logic, FIFO1 callback
│       ├── lin_handler.c        LIN master/slave protocol (break, PID, checksum)
│       ├── cli.c                USB CDC command parser
│       └── usb_cdc_handler.c    Ring buffer + CDC_Transmit_FS wrapper
├── Drivers/                     STM32F1 HAL + CMSIS (included, ready to build)
├── Middlewares/                 STM32 USB Device Library
├── USB_DEVICE/                  CDC class instance
└── Tools/
    ├── webui.html               Single-file Web UI (no dependencies)
    ├── demo.dbc                 Automotive demo DBC
    ├── pyserialScript.py        Python host CLI script
    ├── wiring_diagram.md        Pin-out and transceiver wiring
    ├── CANSim_DUT/
    │   └── CANSim_DUT.ino       Arduino Nano + MCP2515 CAN DUT sketch
    └── LINSim_DUT_LIN/
        └── LINSim_DUT_LIN.ino   Arduino Nano + LINTTL3 LIN DUT sketch
```

---

## 🧠 Key Design Decisions

### 🔀 PA11/PA12 pin conflict: USB vs CAN

On STM32F103C8T6, PA11 = USB D− and PA12 = USB D+ — these are also the **default** CAN pins.  
`__HAL_AFIO_REMAP_CAN1_2()` in `stm32f1xx_hal_msp.c` remaps CAN to **PB8 (RX) / PB9 (TX)**.  
PB8 is 5 V-tolerant, so the TJA1051 TX line connects directly.

### 🚌 LIN via USART1 hardware LIN mode

USART1 (PA9/PA10) runs in CubeMX's **LIN mode** (`LINEN`), 19200-8N1, 11-bit
break detection length — not plain Asynchronous mode. PID/checksum handling
and the single-master/slave-response logic are still implemented in software
in `lin_handler.c`; only break generation/detection use the hardware:

- **Break TX**: `HAL_LIN_SendBreak()` — hardware-generated break of the
  configured length, instead of a software baud-rate trick.
- **Break RX**: the hardware `LBD` (LIN Break Detection) status flag,
  polled once per main-loop iteration alongside received bytes — not an
  IRQ, and not a framing-error heuristic.
- RX stays polled rather than IRQ-driven, since LIN's byte time (~520 µs at
  19200 baud) is long relative to the loop regardless of how break is
  detected.
- PA10 (`USART1_RX`) needs its internal pull-up enabled in CubeMX — the
  MCP2004's `RXD` is open-drain, so an unpulled line floats when the
  transceiver isn't actively driving it.
- The MCP2004's `CS` (transmitter enable, PB1) is GPIO-driven, not tied
  permanently to 3.3 V — held low except during an actual transmission, so
  an indeterminate USART1 TX line (e.g. around reset) can't jam the bus.

See `CLAUDE.md`'s "LIN architecture decisions" for the full reasoning behind
each of these choices, including debugging history and why an earlier
version of this project used a software-only implementation instead.

### ⚡ Shared IRQ: USB_LP and CAN FIFO0

`USB_LP_CAN1_RX0_IRQn` (vector 20) is shared between USB low-priority events and CAN FIFO0.  
CAN RX filters route all messages to **FIFO1**; `CAN1_RX1_IRQn` (vector 21) is exclusive to FIFO1 — no conflict.

### 🛡️ USB CDC reliability

Two failure modes discovered on Blue Pill hardware and mitigated:

| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| CDC stops after USB reconnect | `TxState` stuck after USB glitch | `tx_dead` flag + 100 ms TX timeout in `CDC_Handler_Write`; cleared on `SET_CONTROL_LINE_STATE` |
| No RX data after reconnect | `USBD_CDC_ReceivePacket` fails to re-arm OUT endpoint | Retry once in `CDC_Receive_FS` |

Python script uses `time.sleep(2)` before asserting DTR to allow the CDC enumeration to complete on macOS.

### ⏱️ Bitrate (500 kbps)

```
APB1  = 36 MHz   (SYSCLK 72 MHz ÷ 2)
TQ    = 9 ÷ 36 MHz = 250 ns
Bit   = (1 + BS1=5 + BS2=2) × 250 ns = 2 µs → 500 kbps
SP    = 75 %
```

Adjust `Prescaler`, `BS1`, `BS2` in `MX_CAN_Init()` to change bitrate.

---

## 📋 Requirements

| Tool | Version |
|------|---------|
| 🛠️ STM32CubeIDE | ≥ 1.13 (includes CubeMX 6.x) |
| ⚙️ arm-none-eabi-gcc | Bundled with CubeIDE |
| 🔗 ST-Link v2 | Or compatible debugger/programmer |
| 🌐 Chrome / Edge | ≥ 89 for Web Serial API in the Web UI |
| 🐍 Python | ≥ 3.8 + `pyserial` for `pyserialScript.py` |
| 🔌 Arduino IDE | ≥ 1.8 + autowp/arduino-mcp2515 library for the DUT sketch |
