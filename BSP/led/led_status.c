#include "led_status.h"

#include "main.h"

static led_status_mode_t led_mode = LED_STATUS_NORMAL;
static uint32_t led_tick_ms;
static uint32_t led_pwm_slot_ms;
static int16_t led_breath_level;
static int8_t led_breath_dir;

void LedStatus_Init(void)
{
  led_mode = LED_STATUS_NORMAL;
  led_tick_ms = 0U;
  led_pwm_slot_ms = 0U;
  led_breath_level = 0;
  led_breath_dir = 1;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LedStatus_SetMode(led_status_mode_t mode)
{
  led_mode = mode;
}

void LedStatus_TaskStep(uint32_t period_ms)
{
  uint32_t period;
  uint32_t on_time;
  uint32_t duty_ms;

  led_tick_ms += period_ms;

  switch (led_mode)
  {
    case LED_STATUS_LOW_BATTERY:
    case LED_STATUS_FAULT:
    case LED_STATUS_ESTOP:
      period = 200U;
      on_time = 100U;
      break;
    case LED_STATUS_UPPER_LINK:
      period = 600U;
      on_time = 300U;
      break;
    case LED_STATUS_NORMAL:
    default:
      led_pwm_slot_ms += period_ms;
      if (led_pwm_slot_ms >= 20U)
      {
        led_pwm_slot_ms = 0U;
      }

      if (led_tick_ms >= 20U)
      {
        led_tick_ms = 0U;
        led_breath_level = (int16_t)(led_breath_level + led_breath_dir);
        if (led_breath_level >= 100)
        {
          led_breath_level = 100;
          led_breath_dir = -1;
        }
        else if (led_breath_level <= 0)
        {
          led_breath_level = 0;
          led_breath_dir = 1;
        }
      }

      duty_ms = ((uint32_t)led_breath_level * 20U) / 100U;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (led_pwm_slot_ms < duty_ms) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      return;
  }

  if (led_tick_ms >= period)
  {
    led_tick_ms = 0U;
  }

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (led_tick_ms < on_time) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
