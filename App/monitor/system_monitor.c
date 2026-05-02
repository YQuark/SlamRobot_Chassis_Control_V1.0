#include "system_monitor.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "control_manager.h"

static system_monitor_state_t monitor_state;

void SystemMonitor_Init(void)
{
  monitor_state = (system_monitor_state_t){0};
}

void SystemMonitor_Update(void)
{
  adc_monitor_state_t adc_state;

  monitor_state.error_flags = 0U;
  AdcMonitor_GetState(&adc_state);
  monitor_state.battery_voltage = adc_state.battery_voltage;
  monitor_state.left_current_a = adc_state.left_current_a;
  monitor_state.right_current_a = adc_state.right_current_a;
  monitor_state.control_mode = ControlManager_GetActiveSource();

  if (ADC_MONITOR_CALIBRATION_ENABLED != 0U)
  {
    if (monitor_state.battery_voltage > 0.1f && monitor_state.battery_voltage < BATTERY_LOW_WARN_V)
    {
      monitor_state.error_flags |= SYSTEM_ERROR_LOW_BATTERY;
    }
    if (monitor_state.left_current_a > MOTOR_STALL_CURRENT_A)
    {
      monitor_state.error_flags |= SYSTEM_ERROR_LEFT_OVERCURRENT;
    }
    if (monitor_state.right_current_a > MOTOR_STALL_CURRENT_A)
    {
      monitor_state.error_flags |= SYSTEM_ERROR_RIGHT_OVERCURRENT;
    }
  }
  if (ControlManager_IsEmergencyStop() != 0U)
  {
    monitor_state.error_flags |= SYSTEM_ERROR_ESTOP;
  }
}

void SystemMonitor_GetState(system_monitor_state_t *state)
{
  if (state != 0)
  {
    *state = monitor_state;
  }
}
