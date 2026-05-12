#include "control_manager.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "main.h"

static chassis_cmd_t active_cmd;
static uint8_t emergency_stop;
static uint8_t fault_stop;

static uint8_t ControlManager_IsFiniteFloat(float value)
{
  const float max_float = 3.402823466e+38f;
  return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

static float ControlManager_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float ControlManager_ClampFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static uint8_t ControlManager_KinematicsValid(void)
{
  return (CHASSIS_WHEEL_RADIUS_M > 0.0f && CHASSIS_WHEEL_BASE_M > 0.0f) ? 1U : 0U;
}

void ControlManager_Init(void)
{
  active_cmd = (chassis_cmd_t){0};
  emergency_stop = 0U;
  fault_stop = 0U;
}

void ControlManager_ClearCommand(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  active_cmd = (chassis_cmd_t){0};
  __set_PRIMASK(primask);
}

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
  if (cmd != 0)
  {
    chassis_cmd_t sanitized = *cmd;
    uint32_t primask = __get_PRIMASK();

    if (emergency_stop != 0U || fault_stop != 0U)
    {
      return CONTROL_COMMAND_REJECTED;
    }

    if (ControlManager_IsFiniteFloat(sanitized.linear_x) == 0U ||
        ControlManager_IsFiniteFloat(sanitized.angular_z) == 0U)
    {
      ControlManager_ClearCommand();
      return CONTROL_COMMAND_REJECTED_AND_STOPPED;
    }

    if (sanitized.enable == 0U || sanitized.source == CONTROL_SOURCE_NONE)
    {
      ControlManager_ClearCommand();
      return CONTROL_COMMAND_REJECTED_AND_STOPPED;
    }

    sanitized.linear_x = ControlManager_ClampFloat(sanitized.linear_x, CHASSIS_MAX_LINEAR_MPS);
    if (ControlManager_KinematicsValid() == 0U &&
        ControlManager_AbsFloat(sanitized.angular_z) > CHASSIS_ANGULAR_EPSILON_RPS)
    {
      ControlManager_ClearCommand();
      return CONTROL_COMMAND_REJECTED_AND_STOPPED;
    }

    __disable_irq();
    active_cmd = sanitized;
    __set_PRIMASK(primask);
    return CONTROL_COMMAND_ACCEPTED;
  }

  return CONTROL_COMMAND_REJECTED;
}

void ControlManager_SetEmergencyStop(uint8_t enabled)
{
  emergency_stop = (enabled != 0U) ? 1U : 0U;
  ControlManager_ClearCommand();
}

void ControlManager_SetFaultStop(uint8_t enabled)
{
  fault_stop = (enabled != 0U) ? 1U : 0U;
  ControlManager_ClearCommand();
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

  if (emergency_stop != 0U || fault_stop != 0U || snapshot.enable == 0U || snapshot.source == CONTROL_SOURCE_NONE)
  {
    return 0U;
  }

  return (age_ms <= CHASSIS_CMD_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
  return emergency_stop;
}

uint8_t ControlManager_IsFaultStop(void)
{
  return fault_stop;
}

uint8_t ControlManager_GetActiveSource(void)
{
  chassis_cmd_t snapshot;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  snapshot = active_cmd;
  __set_PRIMASK(primask);

  if (emergency_stop != 0U ||
      fault_stop != 0U ||
      snapshot.enable == 0U ||
      snapshot.source == CONTROL_SOURCE_NONE ||
      (osKernelGetTickCount() - snapshot.timestamp_ms) > CHASSIS_CMD_TIMEOUT_MS)
  {
    return CONTROL_SOURCE_NONE;
  }

  return snapshot.source;
}
