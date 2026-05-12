#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t left_count;
  int32_t right_count;
  int32_t left_delta;
  int32_t right_delta;
  float left_speed_mps;
  float right_speed_mps;
  uint8_t speed_valid;
  uint32_t last_update_ms;
} encoder_state_t;

void EncoderDriver_Init(void);
void EncoderDriver_Update(uint32_t now_ms);
void EncoderDriver_GetState(encoder_state_t *state);
float EncoderDriver_GetCountsPerRev(void);

#ifdef __cplusplus
}
#endif

#endif
