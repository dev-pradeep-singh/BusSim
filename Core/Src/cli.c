#include "cli.h"
#include "can_handler.h"
#include "lin_handler.h"
#include "usb_cdc_handler.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define CMD_BUF_SIZE 128U
#define MAX_ARGS     11U     /* SET DATA <8 bytes> needs 10 tokens + cmd */

/* ── State ───────────────────────────────────────────────────────────────── */
static char    cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_len;

/* ── Forward declarations ────────────────────────────────────────────────── */
static void cmd_help(int argc, char **argv);
static void cmd_status(int argc, char **argv);
static void cmd_can(int argc, char **argv);
static void cmd_lin(int argc, char **argv);
static void cmd_set(int argc, char **argv);
static void cmd_listen(int argc, char **argv);
static void cmd_stats(int argc, char **argv);

typedef struct { const char *name; void (*fn)(int, char **); } Cmd_t;

static const Cmd_t cmd_table[] = {
    {"HELP",   cmd_help  },
    {"STATUS", cmd_status},
    {"CAN",    cmd_can   },
    {"LIN",    cmd_lin   },
    {"SET",    cmd_set   },
    {"LISTEN", cmd_listen},
    {"STATS",  cmd_stats },
};

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static void str_upper(char *s)
{
    for (; *s; s++) *s = (char)toupper((unsigned char)*s);
}

static void print(const char *s) { CDC_Handler_WriteStr(s); }

static void print_banner(void)
{
    print("\r\n"
          "╔══════════════════════════════╗\r\n"
          "║  BusSim v1.0  STM32F103C8T6 ║\r\n"
          "║  CAN @ 500 kbps  USB CDC     ║\r\n"
          "╚══════════════════════════════╝\r\n"
          "Type HELP for commands\r\n> ");
}

/* ── Public ──────────────────────────────────────────────────────────────── */
void CLI_Init(void)
{
    cmd_len = 0;
    /* Banner is NOT sent here – USB is not connected yet at boot.
     * It is sent in CLI_Process() when DTR is asserted by the host. */
}

void CLI_Process(void)
{
    /* Send banner the moment the host opens the COM port (DTR asserted) */
    if (CDC_Handler_NewConnection())
    {
        cmd_len = 0;   /* discard any partial command from a previous session */
        print_banner();
    }

    int c;
    while ((c = CDC_Handler_GetChar()) >= 0)
    {
        if (c == '\r' || c == '\n')
        {
            print("\r\n");
            if (cmd_len > 0)
            {
                cmd_buf[cmd_len] = '\0';
                cmd_len = 0;

                /* tokenise */
                char *args[MAX_ARGS];
                int   argc = 0;
                char *tok  = strtok(cmd_buf, " \t");
                while (tok && argc < (int)MAX_ARGS)
                {
                    args[argc++] = tok;
                    tok = strtok(NULL, " \t");
                }
                if (argc > 0)
                {
                    str_upper(args[0]);
                    bool found = false;
                    for (size_t i = 0; i < sizeof(cmd_table) / sizeof(cmd_table[0]); i++)
                    {
                        if (strcmp(args[0], cmd_table[i].name) == 0)
                        {
                            cmd_table[i].fn(argc, args);
                            found = true;
                            break;
                        }
                    }
                    if (!found) print("Unknown command. Type HELP.\r\n");
                }
            }
            print("> ");
        }
        else if (c == '\b' || c == 127)   /* backspace / DEL */
        {
            if (cmd_len > 0)
            {
                cmd_len--;
                print("\b \b");
            }
        }
        else if (cmd_len < CMD_BUF_SIZE - 1U)
        {
            cmd_buf[cmd_len++] = (char)c;
            uint8_t echo = (uint8_t)c;
            CDC_Handler_Write(&echo, 1U);
        }
    }
}

/* ── Command handlers ────────────────────────────────────────────────────── */
static void cmd_help(int argc, char **argv)
{
    (void)argc; (void)argv;
    print(
        "  HELP                    Show this help\r\n"
        "  STATUS                  Current config (ID, DLC, rate, state)\r\n"
        "  CAN ON|OFF              Start / stop periodic CAN TX\r\n"
        "  SET ID <hex>            CAN frame ID, 11-bit (e.g. 0x123 or 7FF)\r\n"
        "  SET DLC <1-8>           CAN data length code\r\n"
        "  SET DATA <bytes>        CAN payload, up to 8 hex bytes\r\n"
        "  SET RATE <1-100>        CAN TX rate in Hz\r\n"
        "  LIN ON|OFF              Start / stop periodic LIN master TX\r\n"
        "  LIN MODE MASTER|SLAVE   Select LIN role (SLAVE disables master TX;\r\n"
        "                          LIN is single-master, use before a real\r\n"
        "                          external master is on the bus)\r\n"
        "  LIN SEND <id> <bytes>   Send one LIN frame now (id is 6-bit, e.g. 0x10)\r\n"
        "  LIN RESP <id> <bytes>   Register slave auto-response data for an ID\r\n"
        "  LIN RESP CLEAR <id>     Remove a registered slave response\r\n"
        "  SET LINID <hex>         LIN frame ID, 6-bit (0x00-0x3F)\r\n"
        "  SET LINDLC <1-8>        LIN data length\r\n"
        "  SET LINDATA <bytes>     LIN payload, up to 8 hex bytes\r\n"
        "  SET LINRATE <1-100>     LIN TX rate in Hz\r\n"
        "  LISTEN ON|OFF           Print received CAN and LIN frames to terminal\r\n"
        "  STATS [RESET]           Show (or reset) TX / RX counters\r\n"
    );
}

static void cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t data[8]; uint8_t dlc;
    CAN_GetData(data, &dlc);

    char buf[220];
    int n = snprintf(buf, sizeof(buf),
        "  CAN TX:   %s\r\n"
        "  Bitrate:  500 kbps\r\n"
        "  ID:       0x%03lX\r\n"
        "  DLC:      %u\r\n"
        "  Rate:     %lu Hz\r\n"
        "  Data:    ",
        CAN_IsTxActive()   ? "ON"  : "OFF",
        (unsigned long)CAN_GetId(),
        (unsigned)dlc,
        (unsigned long)CAN_GetRateHz());

    for (uint8_t i = 0; i < dlc; i++)
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", data[i]);

    n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\r\n");
    CDC_Handler_Write((uint8_t *)buf, (uint16_t)n);

    uint8_t lin_data[8]; uint8_t lin_dlc;
    LIN_GetData(lin_data, &lin_dlc);

    n = snprintf(buf, sizeof(buf),
        "  LIN Mode: %s\r\n"
        "  LIN TX:   %s\r\n"
        "  LIN ID:   0x%02X\r\n"
        "  LIN DLC:  %u\r\n"
        "  LIN Rate: %lu Hz\r\n"
        "  LIN Data:",
        (LIN_GetMode() == LIN_MODE_SLAVE) ? "SLAVE" : "MASTER",
        LIN_IsTxActive() ? "ON" : "OFF",
        (unsigned)LIN_GetId(),
        (unsigned)lin_dlc,
        (unsigned long)LIN_GetRateHz());

    for (uint8_t i = 0; i < lin_dlc; i++)
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", lin_data[i]);

    n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\r\n  Listen:   %s\r\n",
                  CAN_IsListenMode() ? "ON" : "OFF");
    CDC_Handler_Write((uint8_t *)buf, (uint16_t)n);
}

static void cmd_can(int argc, char **argv)
{
    if (argc < 2) { print("Usage: CAN ON|OFF\r\n"); return; }
    str_upper(argv[1]);
    if      (strcmp(argv[1], "ON")  == 0) { CAN_StartTx(); print("CAN TX started\r\n"); }
    else if (strcmp(argv[1], "OFF") == 0) { CAN_StopTx();  print("CAN TX stopped\r\n"); }
    else                                   print("Usage: CAN ON|OFF\r\n");
}

static void cmd_lin(int argc, char **argv)
{
    if (argc < 2) { print("Usage: LIN ON|OFF|SEND|RESP|MODE\r\n"); return; }
    str_upper(argv[1]);

    if (strcmp(argv[1], "ON") == 0)
    {
        if (LIN_StartTx()) print("LIN TX started\r\n");
        else                print("Blocked: node is in LIN MODE SLAVE\r\n");
        return;
    }
    else if (strcmp(argv[1], "OFF") == 0) { LIN_StopTx();  print("LIN TX stopped\r\n"); return; }

    if (strcmp(argv[1], "MODE") == 0)
    {
        if (argc < 3) { print("Usage: LIN MODE MASTER|SLAVE\r\n"); return; }
        str_upper(argv[2]);
        if (strcmp(argv[2], "MASTER") == 0)
        {
            LIN_SetMode(LIN_MODE_MASTER);
            print("LIN mode: MASTER\r\n");
        }
        else if (strcmp(argv[2], "SLAVE") == 0)
        {
            LIN_SetMode(LIN_MODE_SLAVE);
            print("LIN mode: SLAVE (master TX disabled)\r\n");
        }
        else print("Usage: LIN MODE MASTER|SLAVE\r\n");
        return;
    }

    if (strcmp(argv[1], "SEND") == 0)
    {
        if (argc < 3) { print("Usage: LIN SEND <id> <bytes...>\r\n"); return; }
        uint32_t id = strtoul(argv[2], NULL, 16);
        if (id > 0x3FU) { print("LIN ID must be 6-bit (0x00-0x3F)\r\n"); return; }

        uint8_t data[8]; uint8_t count = 0;
        for (int i = 3; i < argc && count < 8U; i++)
            data[count++] = (uint8_t)strtoul(argv[i], NULL, 16);

        if (!LIN_SendOnce((uint8_t)id, data, count)) { print("Blocked: node is in LIN MODE SLAVE\r\n"); return; }
        char buf[48];
        snprintf(buf, sizeof(buf), "LIN frame 0x%02X sent (%u bytes)\r\n", (unsigned)id, count);
        print(buf);
        return;
    }

    if (strcmp(argv[1], "RESP") == 0)
    {
        if (argc < 3) { print("Usage: LIN RESP <id> <bytes...> | LIN RESP CLEAR <id>\r\n"); return; }

        char id_tok[8];
        strncpy(id_tok, argv[2], sizeof(id_tok) - 1U);
        id_tok[sizeof(id_tok) - 1U] = '\0';
        str_upper(id_tok);

        if (strcmp(id_tok, "CLEAR") == 0)
        {
            if (argc < 4) { print("Usage: LIN RESP CLEAR <id>\r\n"); return; }
            uint32_t id = strtoul(argv[3], NULL, 16);
            if (LIN_ClearSlaveResponse((uint8_t)id)) print("Response cleared\r\n");
            else                                      print("No response registered for that ID\r\n");
            return;
        }

        uint32_t id = strtoul(argv[2], NULL, 16);
        if (id > 0x3FU) { print("LIN ID must be 6-bit (0x00-0x3F)\r\n"); return; }

        uint8_t data[8]; uint8_t count = 0;
        for (int i = 3; i < argc && count < 8U; i++)
            data[count++] = (uint8_t)strtoul(argv[i], NULL, 16);

        if (count == 0) { print("Usage: LIN RESP <id> <bytes...>\r\n"); return; }

        if (LIN_SetSlaveResponse((uint8_t)id, data, count))
        {
            char buf[56];
            snprintf(buf, sizeof(buf), "Response for 0x%02X registered (%u bytes)\r\n", (unsigned)id, count);
            print(buf);
        }
        else print("Response table full (max 8 entries)\r\n");
        return;
    }

    print("Usage: LIN ON|OFF|SEND|RESP|MODE\r\n");
}

static void cmd_set(int argc, char **argv)
{
    if (argc < 3) { print("Usage: SET ID|DLC|DATA|RATE|LINID|LINDLC|LINDATA|LINRATE ...\r\n"); return; }
    str_upper(argv[1]);

    if (strcmp(argv[1], "ID") == 0)
    {
        uint32_t id = (uint32_t)strtoul(argv[2], NULL, 16);
        if (id > 0x7FFU) { print("ID must be 11-bit (0x000-0x7FF)\r\n"); return; }
        CAN_SetId(id);
        char buf[32];
        snprintf(buf, sizeof(buf), "ID set to 0x%03lX\r\n", (unsigned long)id);
        print(buf);
    }
    else if (strcmp(argv[1], "DLC") == 0)
    {
        unsigned dlc = (unsigned)strtoul(argv[2], NULL, 10);
        if (dlc < 1U || dlc > 8U) { print("DLC must be 1-8\r\n"); return; }
        CAN_SetDLC((uint8_t)dlc);
        char buf[24];
        snprintf(buf, sizeof(buf), "DLC set to %u\r\n", dlc);
        print(buf);
    }
    else if (strcmp(argv[1], "RATE") == 0)
    {
        unsigned hz = (unsigned)strtoul(argv[2], NULL, 10);
        if (hz < 1U || hz > 100U) { print("Rate must be 1-100 Hz\r\n"); return; }
        CAN_SetRateHz((uint32_t)hz);
        char buf[28];
        snprintf(buf, sizeof(buf), "Rate set to %u Hz\r\n", hz);
        print(buf);
    }
    else if (strcmp(argv[1], "DATA") == 0)
    {
        uint8_t data[8]; uint8_t count = 0;
        for (int i = 2; i < argc && count < 8U; i++)
            data[count++] = (uint8_t)strtoul(argv[i], NULL, 16);

        if (count == 0) { print("Provide 1-8 hex bytes\r\n"); return; }
        CAN_SetData(data, count);
        CAN_SetDLC(count);

        char buf[64]; int n = snprintf(buf, sizeof(buf), "Data set:");
        for (uint8_t i = 0; i < count; i++)
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", data[i]);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\r\n");
        CDC_Handler_Write((uint8_t *)buf, (uint16_t)n);
    }
    else if (strcmp(argv[1], "LINID") == 0)
    {
        uint32_t id = (uint32_t)strtoul(argv[2], NULL, 16);
        if (id > 0x3FU) { print("LIN ID must be 6-bit (0x00-0x3F)\r\n"); return; }
        LIN_SetId((uint8_t)id);
        char buf[32];
        snprintf(buf, sizeof(buf), "LIN ID set to 0x%02lX\r\n", (unsigned long)id);
        print(buf);
    }
    else if (strcmp(argv[1], "LINDLC") == 0)
    {
        unsigned dlc = (unsigned)strtoul(argv[2], NULL, 10);
        if (dlc < 1U || dlc > 8U) { print("LIN DLC must be 1-8\r\n"); return; }
        LIN_SetDLC((uint8_t)dlc);
        char buf[28];
        snprintf(buf, sizeof(buf), "LIN DLC set to %u\r\n", dlc);
        print(buf);
    }
    else if (strcmp(argv[1], "LINRATE") == 0)
    {
        unsigned hz = (unsigned)strtoul(argv[2], NULL, 10);
        if (hz < 1U || hz > 100U) { print("Rate must be 1-100 Hz\r\n"); return; }
        LIN_SetRateHz((uint32_t)hz);
        char buf[32];
        snprintf(buf, sizeof(buf), "LIN rate set to %u Hz\r\n", hz);
        print(buf);
    }
    else if (strcmp(argv[1], "LINDATA") == 0)
    {
        uint8_t data[8]; uint8_t count = 0;
        for (int i = 2; i < argc && count < 8U; i++)
            data[count++] = (uint8_t)strtoul(argv[i], NULL, 16);

        if (count == 0) { print("Provide 1-8 hex bytes\r\n"); return; }
        LIN_SetData(data, count);
        LIN_SetDLC(count);

        char buf[64]; int n = snprintf(buf, sizeof(buf), "LIN data set:");
        for (uint8_t i = 0; i < count; i++)
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", data[i]);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\r\n");
        CDC_Handler_Write((uint8_t *)buf, (uint16_t)n);
    }
    else
    {
        print("Usage: SET ID|DLC|DATA|RATE|LINID|LINDLC|LINDATA|LINRATE ...\r\n");
    }
}

static void cmd_listen(int argc, char **argv)
{
    if (argc < 2) { print("Usage: LISTEN ON|OFF\r\n"); return; }
    str_upper(argv[1]);
    if (strcmp(argv[1], "ON") == 0)
    {
        CAN_SetListenMode(true);
        LIN_SetListenMode(true);
        print("Listen ON\r\n");
    }
    else if (strcmp(argv[1], "OFF") == 0)
    {
        CAN_SetListenMode(false);
        LIN_SetListenMode(false);
        print("Listen OFF\r\n");
    }
    else print("Usage: LISTEN ON|OFF\r\n");
}

static void cmd_stats(int argc, char **argv)
{
    if (argc >= 2)
    {
        str_upper(argv[1]);
        if (strcmp(argv[1], "RESET") == 0)
        {
            CAN_ResetStats();
            LIN_ResetStats();
            print("Stats reset\r\n");
            return;
        }
    }
    const CAN_Stats_t *s = CAN_GetStats();
    char buf[220];
    int n = snprintf(buf, sizeof(buf),
        "  CAN TX frames:   %lu\r\n"
        "  CAN RX frames:   %lu\r\n"
        "  CAN TX errors:   %lu\r\n",
        (unsigned long)s->tx_count,
        (unsigned long)s->rx_count,
        (unsigned long)s->tx_errors);

    const LIN_Stats_t *ls = LIN_GetStats();
    n += snprintf(buf + n, sizeof(buf) - (size_t)n,
        "  LIN TX frames:   %lu\r\n"
        "  LIN RX frames:   %lu\r\n"
        "  LIN checksum errors: %lu\r\n",
        (unsigned long)ls->tx_count,
        (unsigned long)ls->rx_count,
        (unsigned long)ls->checksum_errors);
    CDC_Handler_Write((uint8_t *)buf, (uint16_t)n);
}
