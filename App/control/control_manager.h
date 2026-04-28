#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  CONTROL_SOURCE_NONE = 0,
  CONTROL_SOURCE_UPPER = 1,
  CONTROL_SOURCE_PS2 = 2,
  CONTROL_SOURCE_ESP01S = 3
} control_source_t;

typedef struct
{
  float linear_x;
  float angular_z;
  uint8_t enable;
  uint8_t source;
  uint32_t timestamp_ms;
} chassis_cmd_t;

void ControlManager_Init(void);
void ControlManager_SetCommand(const chassis_cmd_t *cmd);
void ControlManager_SetEmergencyStop(uint8_t enabled);
uint8_t ControlManager_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms);
uint8_t ControlManager_IsEmergencyStop(void);
uint8_t ControlManager_GetActiveSource(void);

#ifdef __cplusplus
}
#endif

#endif
