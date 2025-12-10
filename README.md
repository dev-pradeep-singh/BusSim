# BusSim (STM32F103C8T6)

Single CAN + LIN rest-bus simulator for the STM32F103C8T6 (Blue Pill class). It exposes a UART CLI to start/stop dummy traffic and query status. The project is laid out for STM32CubeIDE/CubeMX so you can open the IOC, regenerate code, and keep the user sections.

## Features
- UART CLI on USART1 (default 115200 8N1) for commands: `HELP`, `STATUS`, `CAN ON|OFF`, `LIN ON|OFF`.
- Dummy CAN frame (Std ID 0x123, 8 bytes counter payload) at 10 Hz when enabled.
- Dummy LIN frame (break + sync + ID 0x12 + 4-byte counter payload) at 10 Hz when enabled.
- Simple status report over UART showing on/off state and counters.

## Quick start (CubeIDE)
1) Open `BusSim.ioc` in STM32CubeIDE (6.11+ recommended). Target MCU: `STM32F103C8Tx`, 72 MHz, HSE=8 MHz.
2) Generate the code (Project > Generate Code). Cube will create the HAL Drivers and startup files under this folder.
3) The repository already contains `Core/Src` and `Core/Inc` with user code sections filled. Regeneration will keep the code inside `/* USER CODE BEGIN */` blocks.
4) Build and flash from CubeIDE. Connect:
   - CAN: PA11 (RX) / PA12 (TX) with external CAN transceiver.
   - LIN: USART2 PA2 (TX) / PA3 (RX) with LIN transceiver; 19.2 kbps.
   - CLI: USART1 PA9 (TX) / PA10 (RX) at 115200 baud.

## Command set (UART CLI)
```
HELP           Show commands
STATUS         Show bus states and counters
CAN ON|OFF     Start/stop dummy CAN traffic
LIN ON|OFF     Start/stop dummy LIN traffic
```

## Notes
- CAN timing in `MX_CAN_Init` uses 72 MHz clock for 500 kbps (Prescaler 9, BS1 13, BS2 2). Adjust in CubeMX if you need a different rate.
- LIN uses 19.2 kbps; `HAL_LIN_Init` is used so keep the LIN break detect length at 10 bits unless your transceiver needs 11.
- If you change pins or baud rates in CubeMX, regenerate code and keep the custom logic in the user sections.
- The project does not include host-side tooling. Use a serial terminal for CLI and external CAN/LIN analyzers to view traffic.
