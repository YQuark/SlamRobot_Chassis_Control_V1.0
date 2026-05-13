#include "chassis_control.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"

static chassis_control_state_t chassis_state;
static uint8_t open_loop_test_enabled;
static int16_t open_loop_left;
static int16_t open_loop_right;

static pid_state_t pid_left;
static pid_state_t pid_right;

static const pid_params_t pid_params_left = {
  .kp = CHASSIS_PID_KP_L,
  .ki = CHASSIS_PID_KI_L,
  .kd = CHASSIS_PID_KD_L,
  .integral_limit = CHASSIS_PID_INTEGRAL_LIMIT_L,
  .output_limit = CHASSIS_PID_CORRECTION_LIMIT,
};

static const pid_params_t pid_params_right = {
  .kp = CHASSIS_PID_KP_R,
  .ki = CHASSIS_PID_KI_R,
  .kd = CHASSIS_PID_KD_R,
  .integral_limit = CHASSIS_PID_INTEGRAL_LIMIT_R,
  .output_limit = CHASSIS_PID_CORRECTION_LIMIT,
};

static int16_t ChassisControl_ClampPermille(int32_t permille)
{
  if (permille > CHASSIS_PWM_MAX_PERMILLE)
  {
    return CHASSIS_PWM_MAX_PERMILLE;
  }
  if (permille < -CHASSIS_PWM_MAX_PERMILLE)
  {
    return -CHASSIS_PWM_MAX_PERMILLE;
  }
  return (int16_t)permille;
}

static void ChassisControl_ResolveWheelTargets(const chassis_cmd_t *cmd)
{
  chassis_state.left_target_mps = cmd->linear_x - (cmd->angular_z * CHASSIS_WHEEL_BASE_M * 0.5f);
  chassis_state.right_target_mps = cmd->linear_x + (cmd->angular_z * CHASSIS_WHEEL_BASE_M * 0.5f);
}

static int16_t ChassisControl_MpsToPermille(float target_mps)
{
  int32_t permille;

  if (CHASSIS_OPENLOOP_FULL_MPS <= 0.0f)
  {
    return 0;
  }

  if (target_mps > CHASSIS_OPENLOOP_FULL_MPS)
  {
    target_mps = CHASSIS_OPENLOOP_FULL_MPS;
  }
  else if (target_mps < -CHASSIS_OPENLOOP_FULL_MPS)
  {
    target_mps = -CHASSIS_OPENLOOP_FULL_MPS;
  }

  permille = (int32_t)((target_mps / CHASSIS_OPENLOOP_FULL_MPS) * (float)CHASSIS_PWM_MAX_PERMILLE);
  if (permille > CHASSIS_PWM_MAX_PERMILLE)
  {
    permille = CHASSIS_PWM_MAX_PERMILLE;
  }
  else if (permille < -CHASSIS_PWM_MAX_PERMILLE)
  {
    permille = -CHASSIS_PWM_MAX_PERMILLE;
  }

  return (int16_t)permille;
}

static int16_t ChassisControl_ApplyCurrentLimit(int16_t permille, float current_a, uint8_t *limited)
{
  int32_t scaled;

  if (limited != 0)
  {
    *limited = 0U;
  }
  if (MOTOR_CURRENT_LIMIT_A <= 0.0f || current_a <= MOTOR_CURRENT_LIMIT_A || permille == 0)
  {
    return permille;
  }

  scaled = (int32_t)((float)permille * (MOTOR_CURRENT_LIMIT_A / current_a));
  if (limited != 0)
  {
    *limited = 1U;
  }
  return ChassisControl_ClampPermille(scaled);
}

static void ChassisControl_SetOutputs(int16_t left_permille, int16_t right_permille)
{
  adc_monitor_state_t adc_state;

  AdcMonitor_GetState(&adc_state);
  chassis_state.left_current_limited = 0U;
  chassis_state.right_current_limited = 0U;

  if (adc_state.current_valid != 0U)
  {
    left_permille = ChassisControl_ApplyCurrentLimit(left_permille,
                                                    adc_state.left_current_a,
                                                    &chassis_state.left_current_limited);
    right_permille = ChassisControl_ApplyCurrentLimit(right_permille,
                                                     adc_state.right_current_a,
                                                     &chassis_state.right_current_limited);
  }

  chassis_state.left_output_permille = left_permille;
  chassis_state.right_output_permille = right_permille;
  MotorDriver_SetPermille(MOTOR_SIDE_LEFT, left_permille);
  MotorDriver_SetPermille(MOTOR_SIDE_RIGHT, right_permille);
  chassis_state.output_enabled = ((left_permille != 0) || (right_permille != 0)) ? 1U : 0U;
}

void ChassisControl_Init(void)
{
  MotorDriver_Init();
  ControlManager_Init();
  PidController_Init(&pid_left, &pid_params_left);
  PidController_Init(&pid_right, &pid_params_right);
  chassis_state = (chassis_control_state_t){0};
  open_loop_test_enabled = 0U;
  open_loop_left = 0;
  open_loop_right = 0;
  MotorDriver_StopAll(MOTOR_STOP_COAST);
}

void ChassisControl_Step(uint32_t now_ms)
{
  chassis_cmd_t cmd;
  encoder_state_t encoder_state;
  uint8_t valid_cmd;

  valid_cmd = ControlManager_GetCommand(&cmd, now_ms);

  EncoderDriver_GetState(&encoder_state);
  chassis_state.left_actual_mps = encoder_state.left_speed_mps;
  chassis_state.right_actual_mps = encoder_state.right_speed_mps;

  if (ControlManager_IsEmergencyStop() != 0U || ControlManager_IsFaultStop() != 0U)
  {
    ChassisControl_EmergencyStop();
    return;
  }

  if (open_loop_test_enabled != 0U)
  {
    ChassisControl_SetOutputs(open_loop_left, open_loop_right);
    return;
  }

  if (valid_cmd != 0U)
  {
    int16_t left_permille;
    int16_t right_permille;

    ChassisControl_ResolveWheelTargets(&cmd);

    if (CHASSIS_PID_ENABLED != 0U)
    {
      float dt_s = (float)CHASSIS_CONTROL_PERIOD_MS / 1000.0f;
      float pid_out_l;
      float pid_out_r;

      if (encoder_state.speed_valid == 0U)
      {
        ChassisControl_EmergencyStop();
        return;
      }

      pid_out_l = PidController_Step(&pid_left, chassis_state.left_target_mps, chassis_state.left_actual_mps, dt_s);
      pid_out_r = PidController_Step(&pid_right, chassis_state.right_target_mps, chassis_state.right_actual_mps, dt_s);
      left_permille = ChassisControl_ClampPermille((int32_t)ChassisControl_MpsToPermille(chassis_state.left_target_mps) + (int32_t)pid_out_l);
      right_permille = ChassisControl_ClampPermille((int32_t)ChassisControl_MpsToPermille(chassis_state.right_target_mps) + (int32_t)pid_out_r);
    }
    else
    {
      left_permille = ChassisControl_MpsToPermille(chassis_state.left_target_mps);
      right_permille = ChassisControl_MpsToPermille(chassis_state.right_target_mps);
    }

    ChassisControl_SetOutputs(left_permille, right_permille);
  }
  else
  {
    chassis_state.left_target_mps = 0.0f;
    chassis_state.right_target_mps = 0.0f;
    chassis_state.left_output_permille = 0;
    chassis_state.right_output_permille = 0;
    chassis_state.left_current_limited = 0U;
    chassis_state.right_current_limited = 0U;
    MotorDriver_StopAll(MOTOR_STOP_COAST);
    chassis_state.output_enabled = 0U;
  }
}

void ChassisControl_EmergencyStop(void)
{
  PidController_Reset(&pid_left);
  PidController_Reset(&pid_right);
  open_loop_test_enabled = 0U;
  chassis_state.left_target_mps = 0.0f;
  chassis_state.right_target_mps = 0.0f;
  chassis_state.left_output_permille = 0;
  chassis_state.right_output_permille = 0;
  chassis_state.left_current_limited = 0U;
  chassis_state.right_current_limited = 0U;
  chassis_state.output_enabled = 0U;
  MotorDriver_StopAll(MOTOR_STOP_BRAKE);
}

void ChassisControl_OpenLoopTest(int16_t left_permille, int16_t right_permille)
{
  open_loop_left = left_permille;
  open_loop_right = right_permille;
  open_loop_test_enabled = ((left_permille != 0) || (right_permille != 0)) ? 1U : 0U;
}

void ChassisControl_GetState(chassis_control_state_t *state)
{
  if (state != 0)
  {
    *state = chassis_state;
  }
}
