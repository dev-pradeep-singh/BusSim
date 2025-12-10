#ifndef BUS_MANAGER_H
#define BUS_MANAGER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stddef.h>

void Bus_Init(CAN_HandleTypeDef *hcan_if, UART_HandleTypeDef *hlin_if);
void Bus_Process(void);
void Bus_StartCan(void);
void Bus_StopCan(void);
void Bus_StartLin(void);
void Bus_StopLin(void);
void Bus_FormatStatus(char *out, size_t len);

#endif /* BUS_MANAGER_H */
