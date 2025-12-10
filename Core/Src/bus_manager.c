#include "bus_manager.h"
#include "app_cli.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

typedef struct
{
  bool can_active;
  bool lin_active;
  uint32_t can_tx_count;
  uint32_t lin_tx_count;
  uint32_t can_next_tick;
  uint32_t lin_next_tick;
  CAN_TxHeaderTypeDef can_header;
} bus_state_t;

static bus_state_t state;
static CAN_HandleTypeDef *can_if;
static UART_HandleTypeDef *lin_if;

static void Bus_ConfigCan(void);
static void Bus_SendCan(void);
static void Bus_SendLin(void);

void Bus_Init(CAN_HandleTypeDef *hcan_if, UART_HandleTypeDef *hlin_if)
{
  can_if = hcan_if;
  lin_if = hlin_if;

  Bus_ConfigCan();

  state.can_header.StdId = 0x123;
  state.can_header.ExtId = 0;
  state.can_header.RTR = CAN_RTR_DATA;
  state.can_header.IDE = CAN_ID_STD;
  state.can_header.DLC = 8;
  state.can_header.TransmitGlobalTime = DISABLE;
}

void Bus_Process(void)
{
  const uint32_t now = HAL_GetTick();

  if (state.can_active && (int32_t)(now - state.can_next_tick) >= 0)
  {
    state.can_next_tick = now + 100U;
    Bus_SendCan();
  }

  if (state.lin_active && (int32_t)(now - state.lin_next_tick) >= 0)
  {
    state.lin_next_tick = now + 100U;
    Bus_SendLin();
  }
}

void Bus_StartCan(void)
{
  state.can_active = true;
  state.can_next_tick = HAL_GetTick();
}

void Bus_StopCan(void)
{
  state.can_active = false;
}

void Bus_StartLin(void)
{
  state.lin_active = true;
  state.lin_next_tick = HAL_GetTick();
}

void Bus_StopLin(void)
{
  state.lin_active = false;
}

void Bus_FormatStatus(char *out, size_t len)
{
  snprintf(out, len,
           "CAN: %s tx=%lu\r\nLIN: %s tx=%lu\r\n",
           state.can_active ? "ON" : "OFF", (unsigned long)state.can_tx_count,
           state.lin_active ? "ON" : "OFF", (unsigned long)state.lin_tx_count);
}

static void Bus_ConfigCan(void)
{
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(can_if, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(can_if) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Bus_SendCan(void)
{
  uint8_t data[8] = {0};
  uint32_t mailbox = 0;
  memcpy(data, &state.can_tx_count, sizeof(state.can_tx_count));
  memcpy(&data[4], &state.lin_tx_count, sizeof(state.lin_tx_count));

  if (HAL_CAN_AddTxMessage(can_if, &state.can_header, data, &mailbox) == HAL_OK)
  {
    state.can_tx_count++;
  }
}

static void Bus_SendLin(void)
{
  uint8_t payload[4];
  uint8_t frame[6];

  payload[0] = (uint8_t)(state.lin_tx_count & 0xFF);
  payload[1] = (uint8_t)((state.lin_tx_count >> 8) & 0xFF);
  payload[2] = (uint8_t)(state.can_tx_count & 0xFF);
  payload[3] = (uint8_t)((state.can_tx_count >> 8) & 0xFF);

  frame[0] = 0x55; /* sync */
  frame[1] = 0x12; /* protected ID without parity (dummy) */
  frame[2] = payload[0];
  frame[3] = payload[1];
  frame[4] = payload[2];
  frame[5] = payload[3];

  /* Send LIN break then the rest of the frame */
  (void)HAL_LIN_SendBreak(lin_if);
  (void)HAL_UART_Transmit(lin_if, frame, sizeof(frame), HAL_MAX_DELAY);

  state.lin_tx_count++;
}
