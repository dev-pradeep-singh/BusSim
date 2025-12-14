#include "app_cli.h"
#include "bus_manager.h"
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
  CLI_Print("> ");
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
    CLI_Print("HELP\r\nSTATUS\r\nCAN TX <id> <dlc> <b0..b7> [PER <ms>|ONCE]\r\nCAN START [period_ms]\r\nCAN STOP\r\nLIN ON\r\nLIN OFF\r\n");
    return;
  }

  if (CLI_CaseCmp(cmd, "STATUS") == 0)
  {
    char buf[96];
    Bus_FormatStatus(buf, sizeof(buf));
    CLI_Print(buf);
    return;
  }

  if (strncmp(cmd, "CAN TX", 6) == 0)
  {
    char *save = NULL;
    char *tok = strtok_r(cmd + 6, " ", &save);
    if (tok == NULL)
    {
      CLI_Print("ERR: id\r\n");
      return;
    }
    uint32_t id = strtoul(tok, NULL, 0);

    tok = strtok_r(NULL, " ", &save);
    if (tok == NULL)
    {
      CLI_Print("ERR: dlc\r\n");
      return;
    }
    uint8_t dlc = (uint8_t)strtoul(tok, NULL, 0);
    if (dlc > 8U)
    {
      CLI_Print("ERR: dlc>8\r\n");
      return;
    }

    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < dlc; i++)
    {
      tok = strtok_r(NULL, " ", &save);
      if (tok == NULL)
      {
        CLI_Print("ERR: data\r\n");
        return;
      }
      data[i] = (uint8_t)strtoul(tok, NULL, 0);
    }

    /* Optional: PER <ms> or ONCE (default) */
    tok = strtok_r(NULL, " ", &save);
    uint32_t per = 0;
    bool do_periodic = false;
    if (tok != NULL)
    {
      if (CLI_CaseCmp(tok, "PER") == 0 || CLI_CaseCmp(tok, "PERIOD") == 0)
      {
        char *val = strtok_r(NULL, " ", &save);
        if (val == NULL)
        {
          CLI_Print("ERR: period\r\n");
          return;
        }
        per = strtoul(val, NULL, 0);
        do_periodic = true;
      }
      else if (CLI_CaseCmp(tok, "ONCE") == 0)
      {
        do_periodic = false;
      }
    }

    Bus_SetCanMessage(id, dlc, data);
    if (do_periodic && per > 0U)
    {
      Bus_StartCan(per);
      CLI_Print("CAN periodic start\r\n");
    }
    else
    {
      Bus_SendCanOnce();
      CLI_Print("CAN once\r\n");
    }
    return;
  }

  if (strncmp(cmd, "CAN START", 9) == 0)
  {
    char *save = NULL;
    char *tok = strtok_r(cmd + 9, " ", &save);
    uint32_t per = 0;
    if (tok != NULL)
    {
      per = strtoul(tok, NULL, 0);
    }
    Bus_StartCan(per);
    CLI_Print("CAN start\r\n");
    return;
  }

  if (CLI_CaseCmp(cmd, "CAN STOP") == 0)
  {
    Bus_StopCan();
    CLI_Print("CAN stop\r\n");
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
