#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Defaults ────────────────────────────────────────────────────────────── */
#define CAN_DEFAULT_ID        0x123U
#define CAN_DEFAULT_DLC       8U
#define CAN_DEFAULT_RATE_HZ   10U

/* ── Types ───────────────────────────────────────────────────────────────── */
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_errors;
} CAN_Stats_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */
void CAN_Handler_Init(void);
void CAN_Handler_Process(void);   /* call from main loop */

/* ── TX control ─────────────────────────────────────────────────────────── */
void CAN_StartTx(void);
void CAN_StopTx(void);
bool CAN_IsTxActive(void);

/* ── Frame configuration ─────────────────────────────────────────────────── */
void     CAN_SetId(uint32_t id);
void     CAN_SetDLC(uint8_t dlc);
void     CAN_SetData(const uint8_t *data, uint8_t len);
void     CAN_SetRateHz(uint32_t hz);

uint32_t CAN_GetId(void);
uint8_t  CAN_GetDLC(void);
void     CAN_GetData(uint8_t *out, uint8_t *len);
uint32_t CAN_GetRateHz(void);

/* ── Statistics ──────────────────────────────────────────────────────────── */
const CAN_Stats_t *CAN_GetStats(void);
void               CAN_ResetStats(void);

/* ── Listen mode (print received frames over USB CDC) ────────────────────── */
void CAN_SetListenMode(bool enable);
bool CAN_IsListenMode(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_HANDLER_H */
