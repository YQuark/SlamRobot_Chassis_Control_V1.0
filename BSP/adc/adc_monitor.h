#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t raw_battery;
  uint16_t raw_left_current;
  uint16_t raw_right_current;
  float battery_voltage;
  float left_current_a;
  float right_current_a;
} adc_monitor_state_t;

void AdcMonitor_Init(void);
void AdcMonitor_Update(void);
void AdcMonitor_GetState(adc_monitor_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
