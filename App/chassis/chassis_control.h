#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float left_target_mps;
  float right_target_mps;
  float left_actual_mps;
  float right_actual_mps;
  uint8_t output_enabled;
} chassis_control_state_t;

void ChassisControl_Init(void);
void ChassisControl_Step(uint32_t now_ms);
void ChassisControl_EmergencyStop(void);
void ChassisControl_OpenLoopTest(int16_t left_permille, int16_t right_permille);
void ChassisControl_GetState(chassis_control_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
