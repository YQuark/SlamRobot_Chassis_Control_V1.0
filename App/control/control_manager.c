#include "control_manager.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "main.h"

static chassis_cmd_t active_cmd;
static uint8_t emergency_stop;

void ControlManager_Init(void)
{
  active_cmd = (chassis_cmd_t){0};
  emergency_stop = 0U;
}

void ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
  if (cmd != 0)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    active_cmd = *cmd;
    __set_PRIMASK(primask);
  }
}

void ControlManager_SetEmergencyStop(uint8_t enabled)
{
  emergency_stop = (enabled != 0U) ? 1U : 0U;
}

uint8_t ControlManager_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
{
  chassis_cmd_t snapshot;
  uint32_t age_ms;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  snapshot = active_cmd;
  __set_PRIMASK(primask);

  age_ms = now_ms - snapshot.timestamp_ms;
  if (cmd != 0)
  {
    *cmd = snapshot;
  }

  if (emergency_stop != 0U || snapshot.enable == 0U || snapshot.source == CONTROL_SOURCE_NONE)
  {
    return 0U;
  }

  return (age_ms <= CHASSIS_CMD_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
  return emergency_stop;
}

uint8_t ControlManager_GetActiveSource(void)
{
  uint8_t source;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  source = active_cmd.source;
  __set_PRIMASK(primask);

  return source;
}
