#include "encoder_driver.h"

#include "chassis_config.h"
#include "tim.h"

#define TWO_PI_F 6.28318530718f

static encoder_state_t encoder_state;
static uint16_t left_last;
static uint16_t right_last;
static uint32_t last_update_ms;
static uint8_t has_last_update;

static int32_t EncoderDriver_Diff16(uint16_t now, uint16_t last)
{
  return (int32_t)(int16_t)(now - last);
}

float EncoderDriver_GetCountsPerRev(void)
{
  return CHASSIS_ENCODER_BASE_PPR * CHASSIS_ENCODER_QUADRATURE_MULT * CHASSIS_MOTOR_GEAR_RATIO;
}

void EncoderDriver_Init(void)
{
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  left_last = 0U;
  right_last = 0U;
  last_update_ms = 0U;
  has_last_update = 0U;
  encoder_state = (encoder_state_t){0};
}

void EncoderDriver_Update(uint32_t now_ms)
{
  uint16_t left_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  uint16_t right_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
  int32_t left_delta = EncoderDriver_Diff16(left_now, left_last) * CHASSIS_LEFT_ENCODER_DIR;
  int32_t right_delta = EncoderDriver_Diff16(right_now, right_last) * CHASSIS_RIGHT_ENCODER_DIR;
  uint32_t dt_ms = now_ms - last_update_ms;
  float dt_s = (float)dt_ms / 1000.0f;
  float counts_per_rev = EncoderDriver_GetCountsPerRev();
  float meters_per_rev = TWO_PI_F * CHASSIS_WHEEL_RADIUS_M;

  left_last = left_now;
  right_last = right_now;

  encoder_state.left_delta = left_delta;
  encoder_state.right_delta = right_delta;
  encoder_state.left_count += left_delta;
  encoder_state.right_count += right_delta;
  encoder_state.last_update_ms = now_ms;

  if (has_last_update == 0U ||
      dt_ms <= CHASSIS_MIN_ENCODER_DT_MS ||
      dt_ms > CHASSIS_MAX_ENCODER_DT_MS ||
      counts_per_rev <= 0.0f ||
      meters_per_rev <= 0.0f)
  {
    encoder_state.left_speed_mps = 0.0f;
    encoder_state.right_speed_mps = 0.0f;
    encoder_state.speed_valid = 0U;
  }
  else
  {
    encoder_state.left_speed_mps = ((float)left_delta / counts_per_rev) * meters_per_rev / dt_s;
    encoder_state.right_speed_mps = ((float)right_delta / counts_per_rev) * meters_per_rev / dt_s;
    encoder_state.speed_valid = 1U;
  }

  last_update_ms = now_ms;
  has_last_update = 1U;
}

void EncoderDriver_GetState(encoder_state_t *state)
{
  if (state != 0)
  {
    *state = encoder_state;
  }
}
