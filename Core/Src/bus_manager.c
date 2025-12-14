#include "bus_manager.h"
#include "app_cli.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

typedef struct
{
  bool can_active;
  bool lin_active;
  uint32_t can_period_ms;
  uint32_t can_tx_count;
  uint32_t lin_tx_count;
  uint32_t can_next_tick;
  uint32_t lin_next_tick;
  CAN_TxHeaderTypeDef can_header;
  uint8_t can_data[8];
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
  memset(state.can_data, 0, sizeof(state.can_data));
  state.can_period_ms = 100;
}

void Bus_Process(void)
{
  const uint32_t now = HAL_GetTick();

  if (state.can_active && state.can_period_ms > 0 && (int32_t)(now - state.can_next_tick) >= 0)
  {
    state.can_next_tick = now + state.can_period_ms;
    Bus_SendCan();
  }

  if (state.lin_active && (int32_t)(now - state.lin_next_tick) >= 0)
  {
    state.lin_next_tick = now + 100U;
    Bus_SendLin();
  }
}

void Bus_SetCanMessage(uint32_t std_id, uint8_t dlc, const uint8_t *data)
{
  if (std_id <= 0x7FFU)
  {
    state.can_header.StdId = std_id;
    state.can_header.IDE = CAN_ID_STD;
  }

  state.can_header.DLC = (dlc <= 8U) ? dlc : 8U;
  memcpy(state.can_data, data, state.can_header.DLC);
}

void Bus_SendCanOnce(void)
{
  Bus_SendCan();
}

void Bus_StartCan(uint32_t period_ms)
{
  if (period_ms > 0U)
  {
    state.can_period_ms = period_ms;
  }
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
           "CAN: %s tx=%lu id=0x%03lX dlc=%u period=%lums\r\nLIN: %s tx=%lu\r\n",
           state.can_active ? "ON" : "OFF",
           (unsigned long)state.can_tx_count,
           (unsigned long)state.can_header.StdId,
           (unsigned int)state.can_header.DLC,
           (unsigned long)state.can_period_ms,
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
  uint32_t mailbox = 0;

  if (HAL_CAN_AddTxMessage(can_if, &state.can_header, state.can_data, &mailbox) == HAL_OK)
  {
    state.can_tx_count++;
    return;
  }

  /* If mailboxes stuck or CAN off, try to recover and retry once */
  uint32_t err = HAL_CAN_GetError(can_if);
  if (err != HAL_CAN_ERROR_NONE)
  {
    (void)HAL_CAN_AbortTxRequest(can_if,
                                 CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    (void)HAL_CAN_Stop(can_if);
    (void)HAL_CAN_Start(can_if);
    (void)HAL_CAN_AddTxMessage(can_if, &state.can_header, state.can_data, &mailbox);
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
