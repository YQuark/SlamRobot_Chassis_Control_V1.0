#include "adc_monitor.h"

#include "adc.h"
#include "chassis_config.h"

static uint16_t adc_dma_buffer[ADC_MONITOR_CHANNEL_COUNT];
static adc_monitor_state_t adc_state;

static float AdcMonitor_RawToVoltage(uint16_t raw)
{
  return ((float)raw * ADC_MONITOR_VREF_V) / ADC_MONITOR_RESOLUTION_COUNTS;
}

void AdcMonitor_Init(void)
{
  adc_state = (adc_monitor_state_t){0};
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_MONITOR_CHANNEL_COUNT);
}

void AdcMonitor_Update(void)
{
  float left_v;
  float right_v;

  adc_state.raw_battery = adc_dma_buffer[0];
  adc_state.raw_left_current = adc_dma_buffer[1];
  adc_state.raw_right_current = adc_dma_buffer[2];

  adc_state.battery_voltage = AdcMonitor_RawToVoltage(adc_state.raw_battery) * ADC_MONITOR_BATTERY_DIVIDER;
  left_v = AdcMonitor_RawToVoltage(adc_state.raw_left_current);
  right_v = AdcMonitor_RawToVoltage(adc_state.raw_right_current);

  if (ADC_MONITOR_CURRENT_V_PER_A > 0.0f)
  {
    adc_state.left_current_a = (left_v - ADC_MONITOR_CURRENT_ZERO_V) / ADC_MONITOR_CURRENT_V_PER_A;
    adc_state.right_current_a = (right_v - ADC_MONITOR_CURRENT_ZERO_V) / ADC_MONITOR_CURRENT_V_PER_A;
  }
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
  if (state != 0)
  {
    *state = adc_state;
  }
}
