#include "adc_monitor.h"

#include "adc.h"
#include "chassis_config.h"

static uint16_t adc_dma_buffer[ADC_MONITOR_CHANNEL_COUNT];
static adc_monitor_state_t adc_state;
static uint8_t current_filter_initialized;

static float AdcMonitor_RawToVoltage(uint16_t raw)
{
  return ((float)raw * ADC_MONITOR_VREF_V) / ADC_MONITOR_RESOLUTION_COUNTS;
}

static float AdcMonitor_VoltageToCurrent(float voltage)
{
  float current = 0.0f;

  if (MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f)
  {
    current = (voltage - MOTOR_CURRENT_ZERO_V) / MOTOR_CURRENT_VOLTS_PER_AMP;
    if (current < 0.0f)
    {
      current = -current;
    }
  }

  return current;
}

static float AdcMonitor_FilterCurrent(float previous, float sample)
{
  float alpha = MOTOR_CURRENT_FILTER_ALPHA;

  if (alpha <= 0.0f)
  {
    return previous;
  }
  if (alpha >= 1.0f || current_filter_initialized == 0U)
  {
    return sample;
  }
  return previous + (alpha * (sample - previous));
}

void AdcMonitor_Init(void)
{
  adc_state = (adc_monitor_state_t){0};
  current_filter_initialized = 0U;
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_MONITOR_CHANNEL_COUNT);
}

void AdcMonitor_Update(void)
{
  float left_v;
  float right_v;
  float left_current;
  float right_current;

  adc_state.raw_battery = adc_dma_buffer[0];
  adc_state.raw_left_current = adc_dma_buffer[1];
  adc_state.raw_right_current = adc_dma_buffer[2];

  adc_state.battery_voltage = AdcMonitor_RawToVoltage(adc_state.raw_battery) * ADC_MONITOR_BATTERY_DIVIDER;
  left_v = AdcMonitor_RawToVoltage(adc_state.raw_left_current);
  right_v = AdcMonitor_RawToVoltage(adc_state.raw_right_current);

  if (ADC_MONITOR_CALIBRATION_ENABLED != 0U && MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f)
  {
    left_current = AdcMonitor_VoltageToCurrent(left_v);
    right_current = AdcMonitor_VoltageToCurrent(right_v);
    adc_state.left_current_a = AdcMonitor_FilterCurrent(adc_state.left_current_a, left_current);
    adc_state.right_current_a = AdcMonitor_FilterCurrent(adc_state.right_current_a, right_current);
    adc_state.current_valid = 1U;
    current_filter_initialized = 1U;
  }
  else
  {
    adc_state.left_current_a = 0.0f;
    adc_state.right_current_a = 0.0f;
    adc_state.current_valid = 0U;
    current_filter_initialized = 0U;
  }
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
  if (state != 0)
  {
    *state = adc_state;
  }
}
