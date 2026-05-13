#include "usart1_debug_console.h"

#include "adc_monitor.h"
#include "chassis_control.h"
#include "chassis_version.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "imu_mpu6050.h"
#include "system_monitor.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_CONSOLE_RX_LINE_SIZE       64U
#define DEBUG_CONSOLE_RX_RING_SIZE       128U
#define DEBUG_CONSOLE_TX_LINE_SIZE       512U
#define DEBUG_CONSOLE_TASK_PERIOD_MS     10U
#define DEBUG_CONSOLE_LOG_PERIOD_MS      500U

static char rx_line[DEBUG_CONSOLE_RX_LINE_SIZE];
static uint8_t rx_len;
static uint8_t stream_enabled;
static uint8_t debug_velocity_enabled;
static chassis_cmd_t debug_velocity_cmd;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[DEBUG_CONSOLE_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static void DebugConsole_Write(const char *text)
{
  if (text != 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
  }
}

static int32_t DebugConsole_Milli(float value)
{
  return (int32_t)(value * 1000.0f);
}

static void DebugConsole_PrintVofaHeader(void)
{
  DebugConsole_Write("t_ms,left_mms,right_mms,left_target_mms,right_target_mms,left_pwm,right_pwm,vbat_mv,left_current_ma,right_current_ma,acc_x_mg,acc_y_mg,acc_z_mg,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,imu_online,imu_addr,imu_who,imu_err,imu_errcnt,imu_transfer,imu_i2cerr,imu_read_ms,error_flags\r\n");
}

static void DebugConsole_PrintHelp(void)
{
  DebugConsole_Write(
    "\r\nUSART1 debug console\r\n"
    "help              show commands\r\n"
    "status            print one status frame\r\n"
    "header            print VOFA+ CSV header\r\n"
    "i2cscan           probe IMU I2C addresses 0x68/0x69\r\n"
    "imutest           read WHO_AM_I at 0x68/0x69\r\n"
    "imu 0|1           disable/enable periodic IMU read\r\n"
    "imuprobe          read-only IMU probe\r\n"
    "imuinit           read-only init, no register write\r\n"
    "imuread           start one async IMU sample read\r\n"
    "imuwrite R V      write one IMU register, hex or decimal\r\n"
    "log 0|1           disable/enable 500ms status log\r\n"
    "motor L R         open-loop permille, range -900..900\r\n"
    "left P            set left motor permille\r\n"
    "right P           set right motor permille\r\n"
    "vel V [W]         closed-loop mm/s and optional mrad/s\r\n"
    "stop              stop open-loop motor test\r\n"
    "estop 0|1         clear/set emergency stop\r\n"
    "\r\n");
}

static void DebugConsole_PrintStatus(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  adc_monitor_state_t adc_state;
  encoder_state_t encoder_state;
  imu_mpu6050_state_t imu_state;
  chassis_control_state_t chassis_state;

  AdcMonitor_GetState(&adc_state);
  EncoderDriver_GetState(&encoder_state);
  ImuMpu6050_GetState(&imu_state);
  ChassisControl_GetState(&chassis_state);

  (void)snprintf(tx, sizeof(tx),
                 "ENC L=%ld d=%ld %ldmm/s R=%ld d=%ld %ldmm/s valid=%u\r\n",
                 (long)encoder_state.left_count,
                 (long)encoder_state.left_delta,
                 (long)DebugConsole_Milli(encoder_state.left_speed_mps),
                 (long)encoder_state.right_count,
                 (long)encoder_state.right_delta,
                 (long)DebugConsole_Milli(encoder_state.right_speed_mps),
                 encoder_state.speed_valid);
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "ADC raw_vbat=%u vbat=%ldmV raw_il=%u il=%ldmA raw_ir=%u ir=%ldmA current_valid=%u\r\n",
                 adc_state.raw_battery,
                 (long)DebugConsole_Milli(adc_state.battery_voltage),
                 adc_state.raw_left_current,
                 (long)DebugConsole_Milli(adc_state.left_current_a),
                 adc_state.raw_right_current,
                 (long)DebugConsole_Milli(adc_state.right_current_a),
                 adc_state.current_valid);
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "IMU enabled=%u online=%u addr=0x%02X who=0x%02X err=%u errcnt=%lu xfer=%u i2cerr=0x%08lX read_ms=%lu acc_raw=%d,%d,%d acc_mg=%ld,%ld,%ld temp_mC=%ld gyro_raw=%d,%d,%d gyro_mdps=%ld,%ld,%ld\r\n",
                 imu_state.enabled,
                 imu_state.online,
                 imu_state.detected_addr,
                 imu_state.who_am_i,
                 imu_state.last_error,
                 (unsigned long)imu_state.error_count,
                 imu_state.transfer_state,
                 (unsigned long)imu_state.last_i2c_error,
                 (unsigned long)imu_state.last_read_ms,
                 imu_state.accel_x,
                 imu_state.accel_y,
                 imu_state.accel_z,
                 (long)DebugConsole_Milli(imu_state.accel_g[0]),
                 (long)DebugConsole_Milli(imu_state.accel_g[1]),
                 (long)DebugConsole_Milli(imu_state.accel_g[2]),
                 (long)DebugConsole_Milli(imu_state.temperature_c),
                 imu_state.gyro_x,
                 imu_state.gyro_y,
                 imu_state.gyro_z,
                 (long)DebugConsole_Milli(imu_state.gyro_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[2]));
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "CHASSIS target=%ld,%ldmm/s actual=%ld,%ldmm/s pwm=%d,%d lim=%u,%u out=%u estop=%u fault=%u\r\n",
                 (long)DebugConsole_Milli(chassis_state.left_target_mps),
                 (long)DebugConsole_Milli(chassis_state.right_target_mps),
                 (long)DebugConsole_Milli(chassis_state.left_actual_mps),
                 (long)DebugConsole_Milli(chassis_state.right_actual_mps),
                 chassis_state.left_output_permille,
                 chassis_state.right_output_permille,
                 chassis_state.left_current_limited,
                 chassis_state.right_current_limited,
                 chassis_state.output_enabled,
                 ControlManager_IsEmergencyStop(),
                 ControlManager_IsFaultStop());
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintVofaFrame(uint32_t now_ms)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  adc_monitor_state_t adc_state;
  imu_mpu6050_state_t imu_state;
  chassis_control_state_t chassis_state;
  system_monitor_state_t monitor_state;

  AdcMonitor_GetState(&adc_state);
  ImuMpu6050_GetState(&imu_state);
  ChassisControl_GetState(&chassis_state);
  SystemMonitor_GetState(&monitor_state);

  (void)snprintf(tx, sizeof(tx),
                 "%lu,%ld,%ld,%ld,%ld,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%lu,%u,%lu,%lu,%lu\r\n",
                 (unsigned long)now_ms,
                 (long)DebugConsole_Milli(chassis_state.left_actual_mps),
                 (long)DebugConsole_Milli(chassis_state.right_actual_mps),
                 (long)DebugConsole_Milli(chassis_state.left_target_mps),
                 (long)DebugConsole_Milli(chassis_state.right_target_mps),
                 chassis_state.left_output_permille,
                 chassis_state.right_output_permille,
                 (long)DebugConsole_Milli(adc_state.battery_voltage),
                 (long)DebugConsole_Milli(adc_state.left_current_a),
                 (long)DebugConsole_Milli(adc_state.right_current_a),
                 (long)DebugConsole_Milli(imu_state.accel_g[0]),
                 (long)DebugConsole_Milli(imu_state.accel_g[1]),
                 (long)DebugConsole_Milli(imu_state.accel_g[2]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[2]),
                 imu_state.online,
                 imu_state.detected_addr,
                 imu_state.who_am_i,
                 imu_state.last_error,
                 (unsigned long)imu_state.error_count,
                 imu_state.transfer_state,
                 (unsigned long)imu_state.last_i2c_error,
                 (unsigned long)imu_state.last_read_ms,
                 (unsigned long)monitor_state.error_flags);
  DebugConsole_Write(tx);
}

static void DebugConsole_I2cScan(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  const uint8_t addrs[] = {0x68U, 0x69U};

  for (uint32_t i = 0U; i < (sizeof(addrs) / sizeof(addrs[0])); ++i)
  {
    (void)snprintf(tx, sizeof(tx), "i2c 0x%02X ready=%u\r\n",
                   addrs[i],
                   ImuMpu6050_IsDeviceReady(addrs[i]));
    DebugConsole_Write(tx);
  }
}

static void DebugConsole_ImuTest(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  uint8_t who = 0U;
  uint32_t hal_error = 0U;
  uint8_t ok68;
  uint8_t ok69;
  imu_mpu6050_state_t imu_state;

  ok68 = ImuMpu6050_TestAddress(0x68U, &who, &hal_error);
  (void)snprintf(tx, sizeof(tx), "imutest 0x68 ok=%u who=0x%02X halerr=0x%08lX\r\n",
                 ok68,
                 who,
                 (unsigned long)hal_error);
  DebugConsole_Write(tx);

  who = 0U;
  hal_error = 0U;
  ok69 = ImuMpu6050_TestAddress(0x69U, &who, &hal_error);
  (void)snprintf(tx, sizeof(tx), "imutest 0x69 ok=%u who=0x%02X halerr=0x%08lX\r\n",
                 ok69,
                 who,
                 (unsigned long)hal_error);
  DebugConsole_Write(tx);

  ImuMpu6050_GetState(&imu_state);
  (void)snprintf(tx, sizeof(tx), "imu state enabled=%u online=%u addr=0x%02X who=0x%02X err=%u errcnt=%lu\r\n",
                 imu_state.enabled,
                 imu_state.online,
                 imu_state.detected_addr,
                 imu_state.who_am_i,
                 imu_state.last_error,
                 (unsigned long)imu_state.error_count);
  DebugConsole_Write(tx);
}

static void DebugConsole_ImuProbe(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  imu_mpu6050_state_t imu_state;
  uint8_t ok;

  ok = ImuMpu6050_ProbeNow();
  ImuMpu6050_GetState(&imu_state);
  (void)snprintf(tx, sizeof(tx), "imuprobe ok=%u enabled=%u online=%u addr=0x%02X who=0x%02X err=%u errcnt=%lu\r\n",
                 ok,
                 imu_state.enabled,
                 imu_state.online,
                 imu_state.detected_addr,
                 imu_state.who_am_i,
                 imu_state.last_error,
                 (unsigned long)imu_state.error_count);
  DebugConsole_Write(tx);
}

static void DebugConsole_ImuInit(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  imu_mpu6050_state_t imu_state;
  uint8_t ok;

  ok = ImuMpu6050_ConfigNow();
  ImuMpu6050_GetState(&imu_state);
  (void)snprintf(tx, sizeof(tx), "imuinit ok=%u enabled=%u online=%u addr=0x%02X who=0x%02X err=%u errcnt=%lu\r\n",
                 ok,
                 imu_state.enabled,
                 imu_state.online,
                 imu_state.detected_addr,
                 imu_state.who_am_i,
                 imu_state.last_error,
                 (unsigned long)imu_state.error_count);
  DebugConsole_Write(tx);
}

static void DebugConsole_ImuRead(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  imu_mpu6050_state_t imu_state;
  uint8_t ok;

  ok = ImuMpu6050_RequestRead();
  ImuMpu6050_GetState(&imu_state);
  (void)snprintf(tx, sizeof(tx), "imuread start=%u enabled=%u online=%u xfer=%u err=%u i2cerr=0x%08lX\r\n",
                 ok,
                 imu_state.enabled,
                 imu_state.online,
                 imu_state.transfer_state,
                 imu_state.last_error,
                 (unsigned long)imu_state.last_i2c_error);
  DebugConsole_Write(tx);
}

static uint8_t DebugConsole_ParseU8(const char *text, unsigned int *value)
{
  char *end = 0;
  unsigned long parsed;

  if (text == 0 || value == 0)
  {
    return 0U;
  }

  parsed = strtoul(text, &end, 0);
  if ((end == text) || (parsed > 0xFFUL))
  {
    return 0U;
  }

  *value = (unsigned int)parsed;
  return 1U;
}

static void DebugConsole_ImuWrite(char *line)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  char reg_text[16];
  char value_text[16];
  unsigned int reg = 0U;
  unsigned int value = 0U;
  uint8_t readback = 0U;
  uint32_t hal_error = 0U;
  uint8_t ok;

  if (sscanf(line, "imuwrite %15s %15s", reg_text, value_text) != 2 ||
      DebugConsole_ParseU8(reg_text, &reg) == 0U ||
      DebugConsole_ParseU8(value_text, &value) == 0U)
  {
    DebugConsole_Write("usage: imuwrite REG VALUE\r\n");
    return;
  }

  ok = ImuMpu6050_WriteRegTest((uint8_t)reg, (uint8_t)value, &readback, &hal_error);
  (void)snprintf(tx, sizeof(tx), "imuwrite reg=0x%02X val=0x%02X ok=%u rb=0x%02X halerr=0x%08lX\r\n",
                 reg,
                 value,
                 ok,
                 readback,
                 (unsigned long)hal_error);
  DebugConsole_Write(tx);
}

static int16_t DebugConsole_ClampPermille(int32_t value)
{
  if (value > 900)
  {
    return 900;
  }
  if (value < -900)
  {
    return -900;
  }
  return (int16_t)value;
}

static void DebugConsole_HandleLine(char *line)
{
  int left;
  int right;
  int value;
  int linear_mm_s;
  int angular_mrad_s = 0;

  if ((strcmp(line, "help") == 0) || (strcmp(line, "h") == 0))
  {
    DebugConsole_PrintHelp();
  }
  else if ((strcmp(line, "status") == 0) || (strcmp(line, "s") == 0))
  {
    DebugConsole_PrintStatus();
  }
  else if (strcmp(line, "header") == 0)
  {
    DebugConsole_PrintVofaHeader();
  }
  else if (strcmp(line, "i2cscan") == 0)
  {
    DebugConsole_I2cScan();
  }
  else if (strcmp(line, "imutest") == 0)
  {
    DebugConsole_ImuTest();
  }
  else if (strcmp(line, "imuprobe") == 0)
  {
    DebugConsole_ImuProbe();
  }
  else if (strcmp(line, "imuinit") == 0)
  {
    DebugConsole_ImuInit();
  }
  else if (strcmp(line, "imuread") == 0)
  {
    DebugConsole_ImuRead();
  }
  else if (strncmp(line, "imuwrite ", 9) == 0)
  {
    DebugConsole_ImuWrite(line);
  }
  else if (sscanf(line, "imu %d", &value) == 1)
  {
    if (ImuMpu6050_SetEnabled((value != 0) ? 1U : 0U) != 0U)
    {
      DebugConsole_Write((value != 0) ? "imu enabled\r\n" : "imu disabled\r\n");
    }
    else
    {
      DebugConsole_Write((value != 0) ? "imu enable failed\r\n" : "imu disabled\r\n");
    }
  }
  else if (sscanf(line, "log %d", &value) == 1)
  {
    stream_enabled = (value != 0) ? 1U : 0U;
    if (stream_enabled != 0U)
    {
      DebugConsole_PrintVofaHeader();
    }
    else
    {
      DebugConsole_Write("log off\r\n");
    }
  }
  else if (sscanf(line, "motor %d %d", &left, &right) == 2)
  {
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_OpenLoopTest(DebugConsole_ClampPermille(left), DebugConsole_ClampPermille(right));
    DebugConsole_Write("motor test updated\r\n");
  }
  else if (sscanf(line, "left %d", &value) == 1)
  {
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_OpenLoopTest(DebugConsole_ClampPermille(value), 0);
    DebugConsole_Write("left motor test updated\r\n");
  }
  else if (sscanf(line, "right %d", &value) == 1)
  {
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_OpenLoopTest(0, DebugConsole_ClampPermille(value));
    DebugConsole_Write("right motor test updated\r\n");
  }
  else if (sscanf(line, "vel %d %d", &linear_mm_s, &angular_mrad_s) == 2 ||
           sscanf(line, "vel %d", &linear_mm_s) == 1)
  {
    debug_velocity_cmd = (chassis_cmd_t){
      .linear_x = (float)linear_mm_s / 1000.0f,
      .angular_z = (float)angular_mrad_s / 1000.0f,
      .enable = 1U,
      .source = CONTROL_SOURCE_DEBUG,
      .timestamp_ms = osKernelGetTickCount(),
    };

    ChassisControl_OpenLoopTest(0, 0);
    debug_velocity_cmd.timestamp_ms = osKernelGetTickCount();
    if (ControlManager_SetCommand(&debug_velocity_cmd) == CONTROL_COMMAND_ACCEPTED)
    {
      debug_velocity_enabled = 1U;
      DebugConsole_Write("velocity command accepted\r\n");
    }
    else
    {
      DebugConsole_Write("velocity command rejected\r\n");
    }
  }
  else if (strcmp(line, "stop") == 0)
  {
    debug_velocity_enabled = 0U;
    ChassisControl_OpenLoopTest(0, 0);
    ControlManager_ClearCommand();
    DebugConsole_Write("chassis stopped\r\n");
  }
  else if (sscanf(line, "estop %d", &value) == 1)
  {
    ControlManager_SetEmergencyStop((value != 0) ? 1U : 0U);
    DebugConsole_Write((value != 0) ? "estop set\r\n" : "estop cleared\r\n");
  }
  else
  {
    DebugConsole_Write("unknown command, type help\r\n");
  }
}

static void DebugConsole_PollRx(void)
{
  uint8_t ch;

  while (rx_tail != rx_head)
  {
    ch = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % DEBUG_CONSOLE_RX_RING_SIZE);

    if ((ch == '\r') || (ch == '\n'))
    {
      if (rx_len > 0U)
      {
        rx_line[rx_len] = '\0';
        DebugConsole_HandleLine(rx_line);
        rx_len = 0U;
      }
    }
    else if (rx_len < (DEBUG_CONSOLE_RX_LINE_SIZE - 1U))
    {
      rx_line[rx_len++] = (char)ch;
    }
    else
    {
      rx_len = 0U;
      DebugConsole_Write("line too long\r\n");
    }
  }
}

void Usart1DebugConsole_Init(void)
{
  rx_len = 0U;
  stream_enabled = 0U;
  debug_velocity_enabled = 0U;
  debug_velocity_cmd = (chassis_cmd_t){0};
  rx_head = 0U;
  rx_tail = 0U;
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  DebugConsole_Write("\r\nF407 chassis firmware v" CHASSIS_FIRMWARE_VERSION "\r\n");
  DebugConsole_Write("USART1 debug console ready, type help\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    uint16_t next_head = (uint16_t)((rx_head + 1U) % DEBUG_CONSOLE_RX_RING_SIZE);

    if (next_head != rx_tail)
    {
      rx_ring[rx_head] = rx_byte;
      rx_head = next_head;
    }

    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

void Task_Usart1DebugConsole(void *argument)
{
  uint32_t last_log_ms = 0U;

  (void)argument;
  for (;;)
  {
    uint32_t now_ms = osKernelGetTickCount();
    DebugConsole_PollRx();

    if (debug_velocity_enabled != 0U)
    {
      debug_velocity_cmd.timestamp_ms = now_ms;
      if (ControlManager_SetCommand(&debug_velocity_cmd) != CONTROL_COMMAND_ACCEPTED)
      {
        debug_velocity_enabled = 0U;
        DebugConsole_Write("velocity command stopped\r\n");
      }
    }

    if ((stream_enabled != 0U) && ((now_ms - last_log_ms) >= DEBUG_CONSOLE_LOG_PERIOD_MS))
    {
      last_log_ms = now_ms;
      DebugConsole_PrintVofaFrame(now_ms);
    }

    osDelay(DEBUG_CONSOLE_TASK_PERIOD_MS);
  }
}
