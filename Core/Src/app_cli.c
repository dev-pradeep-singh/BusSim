#include "app_cli.h"
#include "bus_manager.h"
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#define CLI_BUFFER 64

static UART_HandleTypeDef *cli_uart;
static volatile uint8_t rx_byte;
static char line[CLI_BUFFER];
static size_t line_pos;
static volatile bool line_ready;

static void CLI_HandleLine(const char *cmd);
static void CLI_Trim(char *s);
static int CLI_CaseCmp(const char *a, const char *b);

void CLI_Init(UART_HandleTypeDef *huart)
{
  cli_uart = huart;
  line_pos = 0;
  line_ready = false;
  HAL_UART_Receive_IT(cli_uart, (uint8_t *)&rx_byte, 1);
}

void CLI_Process(void)
{
  if (!line_ready)
  {
    return;
  }

  line_ready = false;
  line[line_pos] = '\0';
  CLI_HandleLine(line);
  line_pos = 0;
}

void CLI_Print(const char *msg)
{
  if (cli_uart == NULL)
  {
    return;
  }
  HAL_UART_Transmit(cli_uart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != cli_uart)
  {
    return;
  }

  if (rx_byte == '\r' || rx_byte == '\n')
  {
    line_ready = true;
  }
  else if (line_pos < (CLI_BUFFER - 1U))
  {
    line[line_pos++] = (char)rx_byte;
  }

  HAL_UART_Receive_IT(cli_uart, (uint8_t *)&rx_byte, 1);
}

static void CLI_HandleLine(const char *cmd_in)
{
  char cmd[CLI_BUFFER];
  strncpy(cmd, cmd_in, sizeof(cmd));
  cmd[CLI_BUFFER - 1] = '\0';
  CLI_Trim(cmd);

  if (cmd[0] == '\0')
  {
    return;
  }

  if (CLI_CaseCmp(cmd, "HELP") == 0)
  {
    CLI_Print("Commands:\r\n");
    CLI_Print("HELP\r\nSTATUS\r\nCAN ON\r\nCAN OFF\r\nLIN ON\r\nLIN OFF\r\n");
    return;
  }

  if (CLI_CaseCmp(cmd, "STATUS") == 0)
  {
    char buf[96];
    Bus_FormatStatus(buf, sizeof(buf));
    CLI_Print(buf);
    return;
  }

  if (CLI_CaseCmp(cmd, "CAN ON") == 0)
  {
    Bus_StartCan();
    CLI_Print("CAN ON successful!\r\n");
    return;
  }

  if (CLI_CaseCmp(cmd, "CAN OFF") == 0)
  {
    Bus_StopCan();
    CLI_Print("CAN OFF successful!\r\n");
    return;
  }

  if (CLI_CaseCmp(cmd, "LIN ON") == 0)
  {
    Bus_StartLin();
    CLI_Print("LIN ON successful!\r\n");
    return;
  }

  if (CLI_CaseCmp(cmd, "LIN OFF") == 0)
  {
    Bus_StopLin();
    CLI_Print("LIN OFF successful!\r\n");
    return;
  }

}

static void CLI_Trim(char *s)
{
  size_t n = strlen(s);
  while (n > 0 && isspace((int)s[n - 1]))
  {
    s[--n] = '\0';
  }

  size_t start = 0;
  while (s[start] != '\0' && isspace((int)s[start]))
  {
    start++;
  }

  if (start > 0)
  {
    memmove(s, &s[start], strlen(&s[start]) + 1U);
  }
}

static int CLI_CaseCmp(const char *a, const char *b)
{
  while (*a && *b)
  {
    const int da = tolower((int)*a);
    const int db = tolower((int)*b);
    if (da != db)
    {
      return da - db;
    }
    a++;
    b++;
  }
  return (int)((unsigned char)*a) - (int)((unsigned char)*b);
}
