#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYSTEM_ERROR_LOW_BATTERY      (1UL << 0)
#define SYSTEM_ERROR_LEFT_OVERCURRENT (1UL << 1)
#define SYSTEM_ERROR_RIGHT_OVERCURRENT (1UL << 2)
#define SYSTEM_ERROR_ESTOP            (1UL << 3)

typedef struct
{
  float battery_voltage;
  float left_current_a;
  float right_current_a;
  uint32_t error_flags;
  uint8_t control_mode;
} system_monitor_state_t;

void SystemMonitor_Init(void);
void SystemMonitor_Update(void);
void SystemMonitor_GetState(system_monitor_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
