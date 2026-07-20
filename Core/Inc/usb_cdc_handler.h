#ifndef USB_CDC_HANDLER_H
#define USB_CDC_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void CDC_Handler_Init(void);

/* Called from CDC_Receive_FS() in usbd_cdc_if.c (USER CODE section) */
void CDC_Handler_RxCallback(const uint8_t *data, uint32_t len);

/* Called from CDC_SET_CONTROL_LINE_STATE when DTR is asserted (host opened port).
 * Safe to call from USB interrupt context. */
void CDC_Handler_SetConnected(void);

/* Returns true once per new host connection. Call from main loop only. */
bool CDC_Handler_NewConnection(void);

/* Main-loop consumers */
int CDC_Handler_GetChar(void);                          /* -1 if empty */
int CDC_Handler_Write(const uint8_t *data, uint16_t len);
int CDC_Handler_WriteStr(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_HANDLER_H */
