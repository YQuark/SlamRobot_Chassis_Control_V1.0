#ifndef ESP01S_COMM_H
#define ESP01S_COMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t rx_frames;
  uint32_t rx_checksum_errors;
  uint32_t rx_length_errors;
  uint32_t tx_frames;
} esp01s_comm_state_t;

void Esp01sComm_Init(void);
void Esp01sComm_Update(void);
void Esp01sComm_GetState(esp01s_comm_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
