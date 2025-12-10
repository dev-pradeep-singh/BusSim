#ifndef APP_CLI_H
#define APP_CLI_H

#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdbool.h>

void CLI_Init(UART_HandleTypeDef *huart);
void CLI_Process(void);
void CLI_Print(const char *msg);

#endif /* APP_CLI_H */
