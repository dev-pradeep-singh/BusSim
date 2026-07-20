#include "lin_handler.h"
#include "usb_cdc_handler.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* LIN uses USART1's hardware LIN mode (LINEN, 19200-8N1, PA9/PA10, 11-bit
 * break detection — configured via CubeMX, see CLAUDE.md) for break
 * generation (HAL_LIN_SendBreak()) and break detection (the UART_FLAG_LBD
 * status flag), rather than the software break-generation/framing-error
 * heuristic this file used before. RX is still polled from the main loop
 * rather than IRQ-driven — LBD is a plain status flag, no NVIC line is
 * needed for it, and LIN's byte time (~520 us at 19200 baud) is long
 * relative to the loop regardless. */

extern UART_HandleTypeDef huart1;

/* ── Transmitter enable (MCP2004 CS) ─────────────────────────────────────
 * Kept LOW (transmitter disabled) except during an actual transmission
 * window. Tying CS permanently HIGH would let any indeterminate/glitchy
 * state on USART1's TX line (e.g. around MCU reset, before MX_USART1_UART_
 * Init() has run) force the single-wire bus dominant even when this node
 * has nothing to say — and since the transceiver echoes bus state back onto
 * its own RXD, that's indistinguishable from a permanent break to this
 * node's own receiver. GPIO init (LIN_CS_Pin/LIN_CS_GPIO_Port, PB1) is now
 * CubeMX-labeled and lives in the auto-generated part of main.c's
 * MX_GPIO_Init() — unlike the rest of this file, this one pin is not
 * isolated from CubeMX regen, by deliberate choice (see CLAUDE.md). */
static inline void lin_cs_enable(void)  { HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_SET);   }
static inline void lin_cs_disable(void) { HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_RESET); }

/* ── Print ring buffer (ISR-free here, but same drain pattern as CAN) ────── */
#define PRINT_BUF_SIZE 512U
#define PRINT_BUF_MASK (PRINT_BUF_SIZE - 1U)
static struct {
    char              buf[PRINT_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} pb;

static void pb_write(const char *s, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next = (pb.head + 1U) & PRINT_BUF_MASK;
        if (next == pb.tail) return;   /* full — drop rest of this message */
        pb.buf[pb.head] = s[i];
        pb.head = next;
    }
}

static void pb_drain(void)
{
    uint16_t h = pb.head;
    uint16_t t = pb.tail;

    if (h == t) return;

    if (h > t)
    {
        CDC_Handler_Write((uint8_t *)&pb.buf[t], (uint16_t)(h - t));
        pb.tail = h;
    }
    else
    {
        CDC_Handler_Write((uint8_t *)&pb.buf[t], (uint16_t)(PRINT_BUF_SIZE - t));
        pb.tail = 0;
    }
}

/* ── Slave auto-response table ───────────────────────────────────────────── */
typedef struct {
    bool    used;
    uint8_t id;
    uint8_t data[8];
    uint8_t len;
} LIN_Resp_t;

/* ── Module state ────────────────────────────────────────────────────────── */
static struct {
    LIN_Mode_t  mode;
    bool        tx_active;
    bool        listen_mode;
    uint8_t     id;
    uint8_t     dlc;
    uint8_t     data[8];
    uint32_t    rate_hz;
    uint32_t    last_tx_tick;
    volatile bool tx_in_progress;   /* guards RX state machine against our own bus echo */
    LIN_Stats_t stats;
    LIN_Resp_t  responses[LIN_MAX_RESPONSES];
} ctx = {
    .mode    = LIN_MODE_SLAVE,
    .tx_active = false,
    .listen_mode = false,
    .id      = LIN_DEFAULT_ID,
    .dlc     = LIN_DEFAULT_DLC,
    .data    = {0},
    .rate_hz = LIN_DEFAULT_RATE_HZ,
};

static const LIN_Resp_t *find_response(uint8_t id)
{
    for (uint8_t i = 0; i < LIN_MAX_RESPONSES; i++)
        if (ctx.responses[i].used && ctx.responses[i].id == id)
            return &ctx.responses[i];
    return NULL;
}

/* ── PID / checksum (LIN 2.x) ────────────────────────────────────────────── */
static uint8_t lin_calc_pid(uint8_t id)
{
    id &= 0x3FU;
    uint8_t p0 = ((id >> 0) & 1U) ^ ((id >> 1) & 1U) ^ ((id >> 2) & 1U) ^ ((id >> 4) & 1U);
    uint8_t p1 = (uint8_t)!(((id >> 1) & 1U) ^ ((id >> 3) & 1U) ^ ((id >> 4) & 1U) ^ ((id >> 5) & 1U));
    return (uint8_t)(id | (p0 << 6) | (p1 << 7));
}

/* Enhanced checksum (PID + data), except diagnostic frames 0x3C/0x3D which
 * always use the classic (data-only) checksum regardless of LIN version. */
static uint8_t lin_calc_checksum(uint8_t pid, const uint8_t *data, uint8_t len)
{
    uint8_t  id6 = pid & 0x3FU;
    uint16_t sum = (id6 == 0x3CU || id6 == 0x3DU) ? 0U : pid;

    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
        if (sum > 0xFFU) sum -= 0xFFU;
    }
    return (uint8_t)(~sum & 0xFFU);
}

/* ── Master frame TX: break + sync + PID [+ data + checksum] ─────────────── */
static void lin_tx_frame(uint8_t id, const uint8_t *data, uint8_t len)
{
    uint8_t pid  = lin_calc_pid(id);
    uint8_t sync = 0x55U;

    ctx.tx_in_progress = true;
    lin_cs_enable();
    HAL_LIN_SendBreak(&huart1);   /* hardware break, per USART1's LIN mode (LINEN) */
    /* HAL_LIN_SendBreak() only sets SBK and returns immediately (checked
     * against Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c) — it
     * does not wait for the break to actually go out. SBK is cleared by
     * hardware once the break has been sent, so wait for that before
     * queuing sync/PID, or they could race the break on the wire. */
    while (READ_BIT(huart1.Instance->CR1, USART_CR1_SBK) != 0U) { }
    HAL_UART_Transmit(&huart1, &sync, 1U, 5U);
    HAL_UART_Transmit(&huart1, &pid, 1U, 5U);

    if (data != NULL && len > 0U)
    {
        uint8_t chk = lin_calc_checksum(pid, data, len);
        HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 10U);
        HAL_UART_Transmit(&huart1, &chk, 1U, 5U);
    }
    lin_cs_disable();
    ctx.tx_in_progress = false;

    ctx.stats.tx_count++;
    if (ctx.listen_mode)
    {
        char buf[72];
        int  n = snprintf(buf, sizeof(buf), "LIN TX  ID:0x%02X  DATA:", id & 0x3FU);
        for (uint8_t i = 0; i < len; i++)
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", data[i]);
        buf[n++] = '\r'; buf[n++] = '\n';
        pb_write(buf, (uint16_t)n);
    }
}

/* ── RX bus-monitor state machine ─────────────────────────────────────────
 * No length field exists on the wire in LIN, so a passive listener can't
 * know a frame's data length ahead of time. Frames are delimited by the next
 * break (or a hard cap of 8 data + 1 checksum byte, per the LIN 2.x max). */
#define LIN_RX_MAX_BYTES 9U

typedef enum { LIN_RX_IDLE, LIN_RX_SYNC, LIN_RX_PID, LIN_RX_DATA } LinRxState_t;

static struct {
    LinRxState_t state;
    uint8_t      pid;
    uint8_t      id;
    uint8_t      buf[LIN_RX_MAX_BYTES];
    uint8_t      count;
} rx;

static void lin_rx_finalize(void)
{
    if (rx.count < 2U) return;   /* need at least 1 data byte + checksum */

    uint8_t len    = rx.count - 1U;
    uint8_t chk_rx = rx.buf[len];
    uint8_t chk    = lin_calc_checksum(rx.pid, rx.buf, len);

    if (chk != chk_rx) { ctx.stats.checksum_errors++; return; }

    ctx.stats.rx_count++;
    if (ctx.listen_mode)
    {
        char buf[72];
        int  n = snprintf(buf, sizeof(buf), "\r\nLIN RX  ID:0x%02X  DATA:", rx.id);
        for (uint8_t i = 0; i < len; i++)
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", rx.buf[i]);
        buf[n++] = '\r'; buf[n++] = '\n';
        pb_write(buf, (uint16_t)n);
    }
}

static void lin_rx_feed(uint8_t byte)
{
    switch (rx.state)
    {
    case LIN_RX_SYNC:
        rx.state = (byte == 0x55U) ? LIN_RX_PID : LIN_RX_IDLE;
        break;

    case LIN_RX_PID:
    {
        rx.pid   = byte;
        rx.id    = byte & 0x3FU;
        rx.count = 0U;
        rx.state = LIN_RX_DATA;

        /* Our own master header echoing back on the bus — already handled
         * (and printed, if LISTEN is on) by lin_tx_frame(). */
        if (ctx.tx_in_progress) { rx.state = LIN_RX_IDLE; break; }

        const LIN_Resp_t *r = find_response(rx.id);
        if (r != NULL)
        {
            uint8_t chk = lin_calc_checksum(rx.pid, r->data, r->len);
            lin_cs_enable();
            HAL_UART_Transmit(&huart1, (uint8_t *)r->data, r->len, 10U);
            HAL_UART_Transmit(&huart1, &chk, 1U, 5U);
            lin_cs_disable();
            ctx.stats.tx_count++;
            rx.state = LIN_RX_IDLE;   /* our reply echoes back too — don't recapture it */
        }
        break;
    }

    case LIN_RX_DATA:
        if (rx.count < LIN_RX_MAX_BYTES) rx.buf[rx.count++] = byte;
        if (rx.count >= LIN_RX_MAX_BYTES) { lin_rx_finalize(); rx.state = LIN_RX_IDLE; }
        break;

    default:
        break;
    }
}

static void lin_poll_rx(void)
{
    /* Hardware break detection (LINEN + 11-bit break length, set via CubeMX's
     * USART1 LIN-mode config) replaces the old framing-error/0x00 heuristic.
     * LBD is a direct write-0-to-clear SR bit on F1, not part of the
     * read-SR-then-read-DR sequence used for PE/FE/NE/ORE. */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_LBD))
    {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_LBD);
        if (rx.state == LIN_RX_DATA) lin_rx_finalize();
        rx.state = LIN_RX_SYNC;
        rx.count = 0U;
    }

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFFU);   /* read clears RXNE */
        lin_rx_feed(byte);
    }
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */
void LIN_Handler_Init(void)
{
    memset(&rx, 0, sizeof(rx));
    rx.state = LIN_RX_IDLE;
    /* huart1 (19200-8N1, PA9/PA10) is already configured by MX_USART1_UART_Init(). */
}

void LIN_Handler_Process(void)
{
    pb_drain();
    lin_poll_rx();

    if (!ctx.tx_active) return;

    uint32_t now         = HAL_GetTick();
    uint32_t interval_ms = 1000U / ctx.rate_hz;

    if ((now - ctx.last_tx_tick) < interval_ms) return;
    ctx.last_tx_tick = now;

    lin_tx_frame(ctx.id, ctx.data, ctx.dlc);
}

/* ── Mode ────────────────────────────────────────────────────────────────── */
void LIN_SetMode(LIN_Mode_t mode)
{
    ctx.mode = mode;
    if (mode == LIN_MODE_SLAVE) ctx.tx_active = false;
}

LIN_Mode_t LIN_GetMode(void) { return ctx.mode; }

/* ── Control ─────────────────────────────────────────────────────────────── */
bool LIN_StartTx(void)
{
    if (ctx.mode == LIN_MODE_SLAVE) return false;
    ctx.tx_active = true;
    return true;
}
void LIN_StopTx(void)     { ctx.tx_active = false; }
bool LIN_IsTxActive(void) { return ctx.tx_active;  }

void LIN_SetId(uint8_t id)   { ctx.id  = id & 0x3FU; }   /* clamp to 6-bit LIN identifier */
void LIN_SetDLC(uint8_t dlc) { ctx.dlc = (dlc > 8U) ? 8U : dlc; }

void LIN_SetData(const uint8_t *data, uint8_t len)
{
    uint8_t n = (len > 8U) ? 8U : len;
    memcpy(ctx.data, data, n);
}

void LIN_SetRateHz(uint32_t hz) { ctx.rate_hz = (hz == 0U) ? 1U : hz; }

uint8_t  LIN_GetId(void)     { return ctx.id;      }
uint8_t  LIN_GetDLC(void)    { return ctx.dlc;      }
uint32_t LIN_GetRateHz(void) { return ctx.rate_hz;  }

void LIN_GetData(uint8_t *out, uint8_t *len)
{
    memcpy(out, ctx.data, ctx.dlc);
    *len = ctx.dlc;
}

bool LIN_SendOnce(uint8_t id, const uint8_t *data, uint8_t len)
{
    if (ctx.mode == LIN_MODE_SLAVE) return false;
    lin_tx_frame(id & 0x3FU, data, (len > 8U) ? 8U : len);
    return true;
}

bool LIN_SetSlaveResponse(uint8_t id, const uint8_t *data, uint8_t len)
{
    id = id & 0x3FU;
    if (len > 8U) len = 8U;

    for (uint8_t i = 0; i < LIN_MAX_RESPONSES; i++)
        if (ctx.responses[i].used && ctx.responses[i].id == id)
        {
            memcpy(ctx.responses[i].data, data, len);
            ctx.responses[i].len = len;
            return true;
        }

    for (uint8_t i = 0; i < LIN_MAX_RESPONSES; i++)
        if (!ctx.responses[i].used)
        {
            ctx.responses[i].used = true;
            ctx.responses[i].id   = id;
            memcpy(ctx.responses[i].data, data, len);
            ctx.responses[i].len  = len;
            return true;
        }

    return false;   /* table full */
}

bool LIN_ClearSlaveResponse(uint8_t id)
{
    id = id & 0x3FU;
    for (uint8_t i = 0; i < LIN_MAX_RESPONSES; i++)
        if (ctx.responses[i].used && ctx.responses[i].id == id)
        {
            ctx.responses[i].used = false;
            return true;
        }
    return false;
}

const LIN_Stats_t *LIN_GetStats(void) { return &ctx.stats; }
void LIN_ResetStats(void)             { memset(&ctx.stats, 0, sizeof(ctx.stats)); }

void LIN_SetListenMode(bool en) { ctx.listen_mode = en; }
bool LIN_IsListenMode(void)     { return ctx.listen_mode; }
