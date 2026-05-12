#include "system_monitor.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "control_manager.h"
#include "encoder_driver.h"

static system_monitor_state_t monitor_state;
static uint8_t left_overcurrent_count;
static uint8_t right_overcurrent_count;

static uint8_t SystemMonitor_CurrentBelowFaultThreshold(float current_a)
{
  return (current_a <= MOTOR_STALL_CURRENT_A) ? 1U : 0U;
}

static void SystemMonitor_UpdateOvercurrentLatch(const adc_monitor_state_t *adc_state)
{
  if (adc_state->current_valid == 0U)
  {
    left_overcurrent_count = 0U;
    right_overcurrent_count = 0U;
    return;
  }

  if (adc_state->left_current_a > MOTOR_STALL_CURRENT_A)
  {
    if (left_overcurrent_count < MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
    {
      left_overcurrent_count++;
    }
    if (left_overcurrent_count >= MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
    {
      monitor_state.latched_error_flags |= SYSTEM_ERROR_LEFT_OVERCURRENT;
    }
  }
  else
  {
    left_overcurrent_count = 0U;
  }

  if (adc_state->right_current_a > MOTOR_STALL_CURRENT_A)
  {
    if (right_overcurrent_count < MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
    {
      right_overcurrent_count++;
    }
    if (right_overcurrent_count >= MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
    {
      monitor_state.latched_error_flags |= SYSTEM_ERROR_RIGHT_OVERCURRENT;
    }
  }
  else
  {
    right_overcurrent_count = 0U;
  }
}

void SystemMonitor_Init(void)
{
  monitor_state = (system_monitor_state_t){0};
  left_overcurrent_count = 0U;
  right_overcurrent_count = 0U;
}

void SystemMonitor_Update(void)
{
  adc_monitor_state_t adc_state;
  encoder_state_t encoder_state;

  AdcMonitor_GetState(&adc_state);
  EncoderDriver_GetState(&encoder_state);

  monitor_state.error_flags = monitor_state.latched_error_flags;
  monitor_state.battery_voltage = adc_state.battery_voltage;
  monitor_state.left_current_a = adc_state.left_current_a;
  monitor_state.right_current_a = adc_state.right_current_a;

  SystemMonitor_UpdateOvercurrentLatch(&adc_state);
  monitor_state.error_flags |= monitor_state.latched_error_flags;

  if (ADC_MONITOR_CALIBRATION_ENABLED != 0U)
  {
    if (BATTERY_LOW_MONITOR_ENABLED != 0U &&
        monitor_state.battery_voltage > 0.1f &&
        monitor_state.battery_voltage < BATTERY_LOW_WARN_V)
    {
      monitor_state.error_flags |= SYSTEM_ERROR_LOW_BATTERY;
    }
  }
  if (ControlManager_IsEmergencyStop() != 0U)
  {
    monitor_state.error_flags |= SYSTEM_ERROR_ESTOP;
  }
  if (ControlManager_IsFaultStop() != 0U)
  {
    monitor_state.error_flags |= SYSTEM_ERROR_FAULT_STOP;
  }
  if (encoder_state.speed_valid == 0U)
  {
    monitor_state.error_flags |= SYSTEM_ERROR_ENCODER_INVALID;
  }
  if ((monitor_state.latched_error_flags & (SYSTEM_ERROR_LEFT_OVERCURRENT | SYSTEM_ERROR_RIGHT_OVERCURRENT)) != 0U)
  {
    ControlManager_SetFaultStop(1U);
    monitor_state.error_flags |= SYSTEM_ERROR_FAULT_STOP;
  }
  monitor_state.control_mode = ControlManager_GetActiveSource();
}

void SystemMonitor_GetState(system_monitor_state_t *state)
{
  if (state != 0)
  {
    *state = monitor_state;
  }
}

void SystemMonitor_ClearLatchedFaults(uint32_t mask)
{
  uint32_t clearable = mask;

  if ((mask & SYSTEM_ERROR_LEFT_OVERCURRENT) != 0U &&
      SystemMonitor_CurrentBelowFaultThreshold(monitor_state.left_current_a) == 0U)
  {
    clearable &= ~SYSTEM_ERROR_LEFT_OVERCURRENT;
  }
  if ((mask & SYSTEM_ERROR_RIGHT_OVERCURRENT) != 0U &&
      SystemMonitor_CurrentBelowFaultThreshold(monitor_state.right_current_a) == 0U)
  {
    clearable &= ~SYSTEM_ERROR_RIGHT_OVERCURRENT;
  }

  monitor_state.latched_error_flags &= ~clearable;
  if ((monitor_state.latched_error_flags & (SYSTEM_ERROR_LEFT_OVERCURRENT | SYSTEM_ERROR_RIGHT_OVERCURRENT)) == 0U)
  {
    left_overcurrent_count = 0U;
    right_overcurrent_count = 0U;
    ControlManager_SetFaultStop(0U);
  }
}

uint8_t SystemMonitor_HasLatchedFault(void)
{
  return (monitor_state.latched_error_flags != 0U) ? 1U : 0U;
}
