#ifndef LIN_HANDLER_H
#define LIN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Defaults ────────────────────────────────────────────────────────────── */
#define LIN_DEFAULT_ID       0x10U
#define LIN_DEFAULT_DLC      8U
#define LIN_DEFAULT_RATE_HZ  10U
#define LIN_MAX_RESPONSES    8U   /* slave auto-response table size */

/* ── Types ───────────────────────────────────────────────────────────────── */
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t checksum_errors;
} LIN_Stats_t;

/* LIN is strictly single-master: only one node may drive break/sync/PID
 * headers. SLAVE mode blocks this node's own header generation so it can be
 * bench-tested against a real external master without bus contention. */
typedef enum {
    LIN_MODE_MASTER,
    LIN_MODE_SLAVE,
} LIN_Mode_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */
void LIN_Handler_Init(void);
void LIN_Handler_Process(void);   /* call from main loop */

/* ── Mode ─────────────────────────────────────────────────────────────────
 * Switching to SLAVE also stops any active periodic schedule. */
void       LIN_SetMode(LIN_Mode_t mode);
LIN_Mode_t LIN_GetMode(void);

/* ── Master TX control (periodic schedule) ───────────────────────────────── */
bool LIN_StartTx(void);   /* false if blocked by LIN_MODE_SLAVE */
void LIN_StopTx(void);
bool LIN_IsTxActive(void);

/* ── Frame configuration (periodic schedule) ─────────────────────────────── */
void     LIN_SetId(uint8_t id);
void     LIN_SetDLC(uint8_t dlc);
void     LIN_SetData(const uint8_t *data, uint8_t len);
void     LIN_SetRateHz(uint32_t hz);

uint8_t  LIN_GetId(void);
uint8_t  LIN_GetDLC(void);
void     LIN_GetData(uint8_t *out, uint8_t *len);
uint32_t LIN_GetRateHz(void);

/* ── One-shot master frame (LIN SEND) ─────────────────────────────────────
 * Returns false if blocked by LIN_MODE_SLAVE. */
bool LIN_SendOnce(uint8_t id, const uint8_t *data, uint8_t len);

/* ── Slave auto-response table (LIN RESP) ────────────────────────────────── */
bool LIN_SetSlaveResponse(uint8_t id, const uint8_t *data, uint8_t len);
bool LIN_ClearSlaveResponse(uint8_t id);

/* ── Statistics ──────────────────────────────────────────────────────────── */
const LIN_Stats_t *LIN_GetStats(void);
void                LIN_ResetStats(void);

/* ── Listen mode (print RX/TX frames over USB CDC) ───────────────────────── */
void LIN_SetListenMode(bool enable);
bool LIN_IsListenMode(void);

#ifdef __cplusplus
}
#endif

#endif /* LIN_HANDLER_H */
