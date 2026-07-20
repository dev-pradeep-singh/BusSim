#ifndef CLI_H
#define CLI_H

#ifdef __cplusplus
extern "C" {
#endif

void CLI_Init(void);
void CLI_Process(void);   /* call from main loop */

#ifdef __cplusplus
}
#endif

#endif /* CLI_H */
