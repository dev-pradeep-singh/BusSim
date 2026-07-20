# BusSim – Wiring & Hardware Notes

## Blue Pill (STM32F103C8T6) Pin Assignment

```
Blue Pill                     Function
─────────────────────────────────────────────────────
PA11   USB D-  ──────────────► USB connector (D-)
PA12   USB D+  ──────────────► USB connector (D+)

PB8    CAN_RX  ──────────────► CAN transceiver CRXD / RXD
PB9    CAN_TX  ──────────────► CAN transceiver CTXD / TXD

PA9    USART1_TX ────────────► MCP2004 TXD
PA10   USART1_RX ◄──────────── MCP2004 RXD (needs internal pull-up — RXD
                                 is open-drain)
PB1    GPIO out  ────────────► MCP2004 CS (transmitter enable)

PC13   LED     ──────────────► Onboard LED (active-low)

3.3 V  VCC     ──────────────► CAN transceiver VCC (if 3.3 V type)
GND    GND     ──────────────► CAN transceiver GND
```

> **Why PB8/PB9?**  
> The default CAN pins (PA11 / PA12) are the same physical pads as USB D− / D+.
> `__HAL_AFIO_REMAP_CAN1_2()` switches CAN to the alternate mapping on PB8/PB9,
> so both USB and CAN can be active at the same time.

---

## Recommended CAN Transceiver

| Part        | Supply | Notes                                   |
|-------------|--------|-----------------------------------------|
| SN65HVD230  | 3.3 V  | Preferred – directly 3.3 V compatible  |
| MCP2551     | 5 V    | Requires 5 V; add level-shifter or use the 5 V-tolerant I/O trick |
| TJA1050     | 5 V    | Same caution as MCP2551                 |

### SN65HVD230 Wiring (recommended)

```
Blue Pill 3.3 V ──── VCC (pin 3)
Blue Pill GND   ──── GND (pin 2)
PB9  (CAN_TX)   ──── D / TXD (pin 1)
PB8  (CAN_RX)   ──── R / RXD (pin 4)

CAN bus:
  CANH ──── CAN bus HIGH wire
  CANL ──── CAN bus LOW  wire

Bus termination (required at each physical end of the bus):
  120 Ω between CANH and CANL
```

### MCP2551 Wiring (5 V, with caution)

```
Blue Pill 5 V   ──── VDD (pin 3)
Blue Pill GND   ──── VSS (pin 4) + RS (pin 8) to GND
PB9  (CAN_TX)   ──── TXD (pin 1)   ← 5 V-tolerant I/O on F103, OK
PB8  (CAN_RX)   ──── RXD (pin 4)   ← output is 5 V; add 10 kΩ pull-down
                                        or use a voltage divider to stay ≤ 3.6 V
CANH            ──── CAN bus HIGH
CANL            ──── CAN bus LOW
```

---

## LIN Bus (MCP2004)

The MCP2004 is a Microchip LIN transceiver (SOIC-8). Pinout below confirmed
against <https://controllerstech.com/stm32-uart-9-lin-protocol-part-2/> — no
separate logic-supply (VDD/VIO) pin exists on this part; RXD is open-drain
and TXD has an internal pull-up, so both interface with the Blue Pill's
3.3 V logic directly with no level-shifting pin needed. CS drive pattern
(GPIO-toggled around each transmission, not tied permanently high) per
<https://controllerstech.com/stm32-uart-10-lin-protocol-part-3/>:

```
Blue Pill PA9  (USART1_TX) ──── TXD    (MCU → transceiver)
Blue Pill PA10 (USART1_RX) ◄─── RXD    (transceiver → MCU, open-drain output)
Blue Pill PB1  (GPIO out)  ──── CS     (chip select / transmitter enable —
                                          driven HIGH by lin_handler.c only
                                          during an actual transmission, LOW
                                          otherwise; NOT tied permanently
                                          high — an always-enabled TXD path
                                          can jam the bus if USART1's TX line
                                          is ever indeterminate, e.g. around
                                          MCU reset, before MX_USART1_UART_
                                          Init() has run. See lin_handler.c's
                                          lin_cs_enable/disable() and
                                          CLAUDE.md)
External 12 V supply       ──── VBB    (LIN bus supply — NOT the Blue Pill's
                                          3.3 V/5 V rail)
Common GND                 ──── VSS    (STM32 + MCP2004 + 12 V supply + any
                                          other LIN nodes must share ground)
FAULT/TXE                     leave unconnected — fault monitoring not used
VREN                           leave unconnected — no external voltage
                                regulator in this design

LIN bus:
  LBUS pin ──── bus wire

Master pull-up (only needed on whichever node is currently driving the LIN
master schedule — firmware defaults to LIN_MODE_SLAVE now, see CLAUDE.md):
  1 kΩ resistor + diode in series, from the LBUS pin to VBB.
  (Slave-only nodes rely on the active master for this; a transceiver's own
  internal pull-up, if present, is only the much weaker ~30 kΩ slave-side
  pull-up — not enough on its own to hold the bus recessive.)
```

> ⚠️ **Voltage domains**: unlike the CAN side (which shares the Blue Pill's
> 3.3 V rail with the SN65HVD230), LIN's bus voltage is nominally 12 V and
> must come from a separate supply — never tie VBB to the MCU's 3.3 V/5 V rail.

### LIN protocol notes

- Default baud: 19200 (LIN 2.x). USART1 is configured in CubeMX's **LIN
  mode** (`LINEN`), 11-bit break detection length — not plain Asynchronous
  — see the README's "LIN via USART1 hardware LIN mode" section for why,
  and CLAUDE.md for full implementation detail.
- No bus-hardware loopback test path exists for LIN the way `CAN_MODE_LOOPBACK`
  works for CAN — verify with a logic analyzer on TXD/RXD/LIN (break ≥13
  dominant bit-times, sync `0x55`, correct PID parity, correct checksum), or
  with a second LIN node / USB-LIN dongle on the bus.
- STM32 is strictly single-master on the bus at any moment — use
  `LIN MODE MASTER|SLAVE` in the CLI to pick the STM32's role before
  connecting a second real master (e.g. the Arduino LIN DUT below).

## LIN Bus — Arduino DUT (LINTTL3 module)

**[Tools/LINSim_DUT_LIN/LINSim_DUT_LIN.ino](LINSim_DUT_LIN/LINSim_DUT_LIN.ino)**
uses a second, dedicated Arduino Nano as a LIN master test harness — see the
sketch's header comment for the full test-ID/blink-pattern reference.

```
LINTTL3   Arduino Nano
TX     →  D0 (RX)      module's TTL data output into the Nano's UART RX
RX     ←  D1 (TX)      Nano's UART TX drives the module's TTL data input
                        (LINTTL3 TX/RX are labeled from the module's own
                        UART-transceiver perspective, confirmed against the
                        vendor's product page — standard cross-connect)
GND    →  GND          shared with the STM32 board's GND too
VIN    →  external 12 V bench supply (NOT the Nano's 5 V rail)
SLP    →  D7           driven HIGH at boot for normal/awake mode
INH       leave unconnected — transceiver output for an external
          regulator in real vehicle wiring, not a mode-select input

LIN bus:
  LINTTL3 LIN pin  ←→  MCP2004 LBUS pin (STM32 side), same bus wire
```

> ⚠️ The LINTTL3 is likely TJA1021-based; confirm `INH`/`SLP` pin functions
> against that datasheet before wiring — not verified against hardware in
> hand for this note. Same voltage-domain caution as the STM32/MCP2004 side:
> `VIN` is a separate 12 V bus supply, never the Nano's 5 V rail.

Like the MCP2004, the LINTTL3 is a pure level-shifter — it doesn't generate
LIN framing itself, so the Nano's sketch bit-bangs break/sync/PID/checksum in
software (mirroring `Core/Src/lin_handler.c`). Its single hardware UART is
dedicated entirely to the LIN bus for reliable break detection, so the
sketch reports status via the onboard LED rather than live Serial Monitor
output once it starts.

## USB Pull-up

The Blue Pill board has a **fixed 1.5 kΩ pull-up** resistor on PA12 (USB D+),
which signals Full-Speed USB to the host. No external pull-up is needed.  
Some clone boards omit this resistor – add 1.5 kΩ between PA12 and 3.3 V if
the device is not detected.

---

## Loopback Testing (no transceiver needed)

Set `CAN_MODE_LOOPBACK` in `MX_CAN_Init()` to test the firmware without
connecting any bus hardware:

```c
hcan.Init.Mode = CAN_MODE_LOOPBACK;   /* replace CAN_MODE_NORMAL */
```

Transmitted frames will be received back by the same peripheral; enable
`LISTEN ON` in the CLI to see them.

---

## Block Diagram

```
 ┌──────────────────────────────────────────────────┐
 │              STM32F103C8T6 (Blue Pill)           │
 │                                                  │
 │  PA11 ◄─── USB D─ ────────────────┐             │
 │  PA12 ◄─── USB D+ ──┬─1.5kΩ─3V3  │             │
 │              (USB FS CDC VCP)      │             │
 │                                    ▼             │
 │                               [USB Mini-B]       │
 │                                    │             │
 │                            Host PC / laptop      │
 │                                                  │
 │  PB8  ──► CAN_RX ──┐                            │
 │  PB9  ◄── CAN_TX ──┤  [SN65HVD230]             │
 │                     └──── CANH ────┬──── bus     │
 │                          CANL ─────┘             │
 │                           120Ω terminator        │
 └──────────────────────────────────────────────────┘
```

---

## IRQ Priority Map

| Priority | Peripheral       | IRQ vector                |
|----------|-----------------|---------------------------|
| 3        | USB low-priority | `USB_LP_CAN1_RX0_IRQn`   |
| 5        | CAN FIFO1 RX    | `CAN1_RX1_IRQn`           |
| 15       | SysTick (HAL)   | `SysTick_IRQn`            |

`USB_LP_CAN1_RX0_IRQn` is shared between USB_LP events and CAN FIFO0.
By routing all CAN RX to FIFO1, the `CAN1_RX1_IRQn` vector (exclusive to CAN)
is used instead, eliminating any shared-IRQ conflict.

LIN (USART1) doesn't use an interrupt at all — its RX is polled from the main
loop each iteration, since LIN's byte time is long relative to the loop period.
No IRQ conflict is possible with CAN or USB either way.
