#include "chassis_control.h"

#include "chassis_config.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "motor_driver.h"

static chassis_control_state_t chassis_state;
static uint8_t open_loop_test_enabled;
static int16_t open_loop_left;
static int16_t open_loop_right;

static void ChassisControl_ResolveWheelTargets(const chassis_cmd_t *cmd)
{
  chassis_state.left_target_mps = cmd->linear_x - (cmd->angular_z * CHASSIS_WHEEL_BASE_M * 0.5f);
  chassis_state.right_target_mps = cmd->linear_x + (cmd->angular_z * CHASSIS_WHEEL_BASE_M * 0.5f);
}

void ChassisControl_Init(void)
{
  MotorDriver_Init();
  ControlManager_Init();
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
  uint8_t valid_cmd = ControlManager_GetCommand(&cmd, now_ms);

  EncoderDriver_GetState(&encoder_state);
  chassis_state.left_actual_mps = encoder_state.left_speed_mps;
  chassis_state.right_actual_mps = encoder_state.right_speed_mps;

  if (ControlManager_IsEmergencyStop() != 0U)
  {
    ChassisControl_EmergencyStop();
    return;
  }

  if (open_loop_test_enabled != 0U)
  {
    MotorDriver_SetPermille(MOTOR_SIDE_LEFT, open_loop_left);
    MotorDriver_SetPermille(MOTOR_SIDE_RIGHT, open_loop_right);
    chassis_state.output_enabled = 1U;
    return;
  }

  if (valid_cmd != 0U)
  {
    ChassisControl_ResolveWheelTargets(&cmd);
    /* PID is intentionally not enabled in this hardware-framework slice. */
  }
  else
  {
    chassis_state.left_target_mps = 0.0f;
    chassis_state.right_target_mps = 0.0f;
  }

  MotorDriver_StopAll(MOTOR_STOP_COAST);
  chassis_state.output_enabled = 0U;
}

void ChassisControl_EmergencyStop(void)
{
  open_loop_test_enabled = 0U;
  chassis_state.left_target_mps = 0.0f;
  chassis_state.right_target_mps = 0.0f;
  chassis_state.output_enabled = 0U;
  MotorDriver_StopAll(MOTOR_STOP_COAST);
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
