#include "motor_driver.h"

#include "chassis_config.h"
#include "tim.h"

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t forward_channel;
  uint32_t reverse_channel;
  int8_t direction;
} motor_hw_t;

static const motor_hw_t motor_hw[] = {
  { &htim1, TIM_CHANNEL_1, TIM_CHANNEL_4, CHASSIS_LEFT_MOTOR_DIR },
  { &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, CHASSIS_RIGHT_MOTOR_DIR },
};

static uint32_t MotorDriver_PulseFromPermille(const motor_hw_t *motor, int16_t permille)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->htim);
  if (permille < 0)
  {
    permille = (int16_t)-permille;
  }
  if (permille > CHASSIS_PWM_MAX_PERMILLE)
  {
    permille = CHASSIS_PWM_MAX_PERMILLE;
  }
  if (permille > 0 && permille < CHASSIS_PWM_DEADBAND_PERMILLE)
  {
    permille = CHASSIS_PWM_DEADBAND_PERMILLE;
  }
  return ((arr + 1U) * (uint32_t)permille) / 1000U;
}

static void MotorDriver_SetRaw(const motor_hw_t *motor, uint32_t forward_pulse, uint32_t reverse_pulse)
{
  __HAL_TIM_SET_COMPARE(motor->htim, motor->forward_channel, forward_pulse);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->reverse_channel, reverse_pulse);
}

void MotorDriver_Init(void)
{
  for (uint32_t i = 0U; i < (uint32_t)(sizeof(motor_hw) / sizeof(motor_hw[0])); ++i)
  {
    (void)HAL_TIM_PWM_Start(motor_hw[i].htim, motor_hw[i].forward_channel);
    (void)HAL_TIM_PWM_Start(motor_hw[i].htim, motor_hw[i].reverse_channel);
    MotorDriver_SetRaw(&motor_hw[i], 0U, 0U);
  }
}

void MotorDriver_SetPermille(motor_side_t side, int16_t permille)
{
  if (side > MOTOR_SIDE_RIGHT)
  {
    return;
  }

  const motor_hw_t *motor = &motor_hw[(uint32_t)side];
  int16_t corrected = (int16_t)(permille * motor->direction);
  uint32_t pulse = MotorDriver_PulseFromPermille(motor, corrected);

  if (corrected > 0)
  {
    MotorDriver_SetRaw(motor, pulse, 0U);
  }
  else if (corrected < 0)
  {
    MotorDriver_SetRaw(motor, 0U, pulse);
  }
  else
  {
    MotorDriver_SetRaw(motor, 0U, 0U);
  }
}

void MotorDriver_Stop(motor_side_t side, motor_stop_mode_t mode)
{
  if (side > MOTOR_SIDE_RIGHT)
  {
    return;
  }

  const motor_hw_t *motor = &motor_hw[(uint32_t)side];
  if (mode == MOTOR_STOP_BRAKE)
  {
    uint32_t pulse = MotorDriver_PulseFromPermille(motor, CHASSIS_PWM_MAX_PERMILLE);
    MotorDriver_SetRaw(motor, pulse, pulse);
  }
  else
  {
    MotorDriver_SetRaw(motor, 0U, 0U);
  }
}

void MotorDriver_StopAll(motor_stop_mode_t mode)
{
  MotorDriver_Stop(MOTOR_SIDE_LEFT, mode);
  MotorDriver_Stop(MOTOR_SIDE_RIGHT, mode);
}
