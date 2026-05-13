#include "ps2_control.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "main.h"

#define PS2_FRAME_LEN 9U
#define PS2_RETRY_LIMIT 3U

typedef struct
{
  uint8_t mode;
  uint8_t btn1;
  uint8_t btn2;
  uint8_t right_x;
  uint8_t right_y;
  uint8_t left_x;
  uint8_t left_y;
} ps2_sample_t;

static const uint8_t ps2_poll_frame[PS2_FRAME_LEN] = {
  0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

static ps2_control_state_t ps2_state;
static uint8_t ps2_rx[PS2_FRAME_LEN];
static uint8_t swap_cmd_dat;
static uint8_t reconfig_count;

static void Ps2Control_DwtDelayInit(void)
{
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Ps2Control_DelayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock / 1000000U) * us;

  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

static GPIO_TypeDef *Ps2Control_CmdPort(void)
{
  return (swap_cmd_dat != 0U) ? PS2_DAT_GPIO_Port : PS2_CMD_GPIO_Port;
}

static uint16_t Ps2Control_CmdPin(void)
{
  return (swap_cmd_dat != 0U) ? PS2_DAT_Pin : PS2_CMD_Pin;
}

static GPIO_TypeDef *Ps2Control_DatPort(void)
{
  return (swap_cmd_dat != 0U) ? PS2_CMD_GPIO_Port : PS2_DAT_GPIO_Port;
}

static uint16_t Ps2Control_DatPin(void)
{
  return (swap_cmd_dat != 0U) ? PS2_CMD_Pin : PS2_DAT_Pin;
}

static void Ps2Control_SetCmd(uint8_t high)
{
  HAL_GPIO_WritePin(Ps2Control_CmdPort(),
                    Ps2Control_CmdPin(),
                    (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Ps2Control_SetClk(uint8_t high)
{
  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port,
                    PS2_CLK_Pin,
                    (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Ps2Control_SetCs(uint8_t high)
{
  HAL_GPIO_WritePin(PS2_CS_GPIO_Port,
                    PS2_CS_Pin,
                    (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Ps2Control_ReadDat(void)
{
  return (HAL_GPIO_ReadPin(Ps2Control_DatPort(), Ps2Control_DatPin()) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Ps2Control_ConfigurePins(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = Ps2Control_DatPin();
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Ps2Control_DatPort(), &gpio);

  gpio.Pin = Ps2Control_CmdPin();
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Ps2Control_CmdPort(), &gpio);

  gpio.Pin = PS2_CLK_Pin;
  HAL_GPIO_Init(PS2_CLK_GPIO_Port, &gpio);

  gpio.Pin = PS2_CS_Pin;
  HAL_GPIO_Init(PS2_CS_GPIO_Port, &gpio);

  Ps2Control_SetCmd(1U);
  Ps2Control_SetClk(1U);
  Ps2Control_SetCs(1U);
}

static uint8_t Ps2Control_TransferByte(uint8_t tx)
{
  uint8_t rx = 0U;

  for (uint8_t bit = 0x01U; bit != 0U; bit <<= 1U)
  {
    Ps2Control_SetCmd(((tx & bit) != 0U) ? 1U : 0U);
    Ps2Control_SetClk(1U);
    Ps2Control_DelayUs(5U);
    Ps2Control_SetClk(0U);
    Ps2Control_DelayUs(5U);
    Ps2Control_SetClk(1U);

    if (Ps2Control_ReadDat() != 0U)
    {
      rx |= bit;
    }
  }

  Ps2Control_DelayUs(16U);
  Ps2Control_SetCmd(1U);
  return rx;
}

static void Ps2Control_Send(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
  Ps2Control_SetCs(0U);
  Ps2Control_DelayUs(20U);

  for (uint8_t i = 0U; i < len; ++i)
  {
    uint8_t value = Ps2Control_TransferByte(tx[i]);
    if (rx != 0)
    {
      rx[i] = value;
    }
    Ps2Control_DelayUs(16U);
  }

  Ps2Control_SetCs(1U);
  Ps2Control_DelayUs(20U);
}

static uint8_t Ps2Control_HandshakeOk(const uint8_t *rx)
{
  return (rx[1] != 0xFFU && rx[2] == 0x5AU) ? 1U : 0U;
}

static uint8_t Ps2Control_IsAnalogMode(uint8_t mode)
{
  return (mode == 0x73U || mode == 0x79U) ? 1U : 0U;
}

static void Ps2Control_ShortPoll(void)
{
  static const uint8_t poll[5] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U};
  Ps2Control_Send(poll, 0, (uint8_t)sizeof(poll));
}

static void Ps2Control_ConfigAnalog(void)
{
  static const uint8_t enter_cfg[PS2_FRAME_LEN] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t analog_cfg[PS2_FRAME_LEN] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t vibration_cfg[5] = {0x01U, 0x4DU, 0x00U, 0x00U, 0x01U};
  static const uint8_t exit_cfg[PS2_FRAME_LEN] = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};

  Ps2Control_ShortPoll();
  Ps2Control_ShortPoll();
  Ps2Control_ShortPoll();
  HAL_Delay(2U);
  Ps2Control_Send(enter_cfg, 0, (uint8_t)sizeof(enter_cfg));
  HAL_Delay(2U);
  Ps2Control_Send(analog_cfg, 0, (uint8_t)sizeof(analog_cfg));
  HAL_Delay(2U);
  Ps2Control_Send(vibration_cfg, 0, (uint8_t)sizeof(vibration_cfg));
  HAL_Delay(2U);
  Ps2Control_Send(exit_cfg, 0, (uint8_t)sizeof(exit_cfg));
  HAL_Delay(2U);
}

static uint8_t Ps2Control_Probe(uint8_t require_analog)
{
  uint8_t rx[PS2_FRAME_LEN] = {0};

  for (uint8_t i = 0U; i < 6U; ++i)
  {
    Ps2Control_Send(ps2_poll_frame, rx, (uint8_t)sizeof(rx));
    if (Ps2Control_HandshakeOk(rx) != 0U &&
        (require_analog == 0U || Ps2Control_IsAnalogMode(rx[1]) != 0U))
    {
      return 1U;
    }
    HAL_Delay(2U);
  }
  return 0U;
}

static uint8_t Ps2Control_InitMapping(void)
{
  for (uint8_t i = 0U; i < PS2_RETRY_LIMIT; ++i)
  {
    Ps2Control_ConfigAnalog();
    if (Ps2Control_Probe(1U) != 0U)
    {
      return 1U;
    }
  }
  return Ps2Control_Probe(0U);
}

static uint8_t Ps2Control_ReadSample(ps2_sample_t *sample)
{
  if (sample == 0)
  {
    return 0U;
  }

  Ps2Control_Send(ps2_poll_frame, ps2_rx, (uint8_t)sizeof(ps2_poll_frame));
  if (Ps2Control_HandshakeOk(ps2_rx) == 0U)
  {
    if (reconfig_count < PS2_RETRY_LIMIT)
    {
      Ps2Control_ConfigAnalog();
      reconfig_count++;
    }
    return 0U;
  }

  sample->mode = ps2_rx[1];
  sample->btn1 = (uint8_t)~ps2_rx[3];
  sample->btn2 = (uint8_t)~ps2_rx[4];
  if (Ps2Control_IsAnalogMode(sample->mode) != 0U)
  {
    sample->right_x = ps2_rx[5];
    sample->right_y = ps2_rx[6];
    sample->left_x = ps2_rx[7];
    sample->left_y = ps2_rx[8];
    reconfig_count = 0U;
  }
  else
  {
    sample->right_x = PS2_AXIS_CENTER;
    sample->right_y = PS2_AXIS_CENTER;
    sample->left_x = PS2_AXIS_CENTER;
    sample->left_y = PS2_AXIS_CENTER;
    if (reconfig_count < PS2_RETRY_LIMIT)
    {
      Ps2Control_ConfigAnalog();
      reconfig_count++;
    }
  }

  return 1U;
}

static float Ps2Control_NormalizeAxis(uint8_t raw)
{
  int32_t delta = (int32_t)raw - PS2_AXIS_CENTER;
  int32_t magnitude = (delta < 0) ? -delta : delta;

  if (magnitude <= PS2_AXIS_DEADZONE)
  {
    return 0.0f;
  }

  if (delta > 0)
  {
    return (float)(delta - PS2_AXIS_DEADZONE) / (float)(127 - PS2_AXIS_DEADZONE);
  }
  return (float)(delta + PS2_AXIS_DEADZONE) / (float)(128 - PS2_AXIS_DEADZONE);
}

static float Ps2Control_ClampFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static void Ps2Control_SubmitCommand(float linear_x, float angular_z)
{
  chassis_cmd_t cmd = {
    .linear_x = linear_x,
    .angular_z = angular_z,
    .enable = 1U,
    .source = CONTROL_SOURCE_PS2,
    .timestamp_ms = osKernelGetTickCount(),
  };

  if (linear_x == 0.0f && angular_z == 0.0f)
  {
    ControlManager_ClearSource(CONTROL_SOURCE_PS2);
    return;
  }

  (void)ControlManager_SetCommand(&cmd);
}

void Ps2Control_Init(void)
{
  Ps2Control_DwtDelayInit();
  ps2_state = (ps2_control_state_t){0};
  ps2_state.left_x = PS2_AXIS_CENTER;
  ps2_state.left_y = PS2_AXIS_CENTER;
  ps2_state.right_x = PS2_AXIS_CENTER;
  ps2_state.right_y = PS2_AXIS_CENTER;
  reconfig_count = 0U;

  swap_cmd_dat = 0U;
  Ps2Control_ConfigurePins();
  if (Ps2Control_InitMapping() == 0U)
  {
    swap_cmd_dat = 1U;
    Ps2Control_ConfigurePins();
    (void)Ps2Control_InitMapping();
  }
  ps2_state.cmd_dat_swapped = swap_cmd_dat;
}

void Ps2Control_Update(void)
{
  ps2_sample_t sample;
  float linear_x = 0.0f;
  float angular_z = 0.0f;
  uint8_t drive_enabled;

  if (Ps2Control_ReadSample(&sample) == 0U)
  {
    ps2_state.online = 0U;
    ps2_state.drive_enabled = 0U;
    ps2_state.linear_x = 0.0f;
    ps2_state.angular_z = 0.0f;
    ControlManager_ClearSource(CONTROL_SOURCE_PS2);
    return;
  }

  drive_enabled = ((sample.btn2 & PS2_ENABLE_BUTTON_MASK) != 0U) ? 1U : 0U;
  linear_x = -Ps2Control_NormalizeAxis(sample.left_y) * PS2_LINEAR_MAX_MPS;
  angular_z = Ps2Control_NormalizeAxis(sample.right_x) * PS2_ANGULAR_MAX_RPS;

  if ((sample.btn1 & 0x10U) != 0U)
  {
    linear_x = PS2_LINEAR_MAX_MPS;
  }
  else if ((sample.btn1 & 0x40U) != 0U)
  {
    linear_x = -PS2_LINEAR_MAX_MPS;
  }
  if ((sample.btn1 & 0x80U) != 0U)
  {
    angular_z = PS2_ANGULAR_MAX_RPS;
  }
  else if ((sample.btn1 & 0x20U) != 0U)
  {
    angular_z = -PS2_ANGULAR_MAX_RPS;
  }

  linear_x = Ps2Control_ClampFloat(linear_x, PS2_LINEAR_MAX_MPS);
  angular_z = Ps2Control_ClampFloat(angular_z, PS2_ANGULAR_MAX_RPS);

  ps2_state.online = 1U;
  ps2_state.analog_mode = Ps2Control_IsAnalogMode(sample.mode);
  ps2_state.drive_enabled = drive_enabled;
  ps2_state.btn1 = sample.btn1;
  ps2_state.btn2 = sample.btn2;
  ps2_state.left_x = sample.left_x;
  ps2_state.left_y = sample.left_y;
  ps2_state.right_x = sample.right_x;
  ps2_state.right_y = sample.right_y;
  ps2_state.linear_x = (drive_enabled != 0U) ? linear_x : 0.0f;
  ps2_state.angular_z = (drive_enabled != 0U) ? angular_z : 0.0f;

  if (drive_enabled == 0U)
  {
    ControlManager_ClearSource(CONTROL_SOURCE_PS2);
    return;
  }

  Ps2Control_SubmitCommand(ps2_state.linear_x, ps2_state.angular_z);
}

void Ps2Control_GetState(ps2_control_state_t *state)
{
  if (state != 0)
  {
    *state = ps2_state;
  }
}
