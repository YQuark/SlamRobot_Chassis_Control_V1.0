#include "esp01s_comm.h"

#include "chassis_config.h"
#include "chassis_control.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "imu_mpu6050.h"
#include "system_monitor.h"
#include "upper_protocol.h"
#include "usart.h"

#define ESP01S_RX_BUFFER_SIZE 128U

typedef enum
{
  ESP01S_RX_WAIT_HEAD0 = 0,
  ESP01S_RX_WAIT_HEAD1,
  ESP01S_RX_WAIT_LEN,
  ESP01S_RX_WAIT_BODY
} esp01s_rx_state_t;

static uint8_t esp01s_rx_dma_buffer[ESP01S_RX_BUFFER_SIZE];
static uint16_t esp01s_rx_read_pos;
static esp01s_rx_state_t esp01s_rx_state;
static uint8_t esp01s_frame_buf[UPPER_PROTOCOL_MAX_PAYLOAD + 3U];
static uint8_t esp01s_frame_len;
static uint8_t esp01s_frame_index;
static uint8_t esp01s_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint8_t esp01s_status_payload[UPPER_PROTOCOL_STATUS_PAYLOAD_LEN];
static uint32_t esp01s_last_status_ms;
static esp01s_comm_state_t esp01s_state;

static void Esp01sComm_ResetParser(void)
{
  esp01s_rx_state = ESP01S_RX_WAIT_HEAD0;
  esp01s_frame_len = 0U;
  esp01s_frame_index = 0U;
}

static void Esp01sComm_HandleFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  esp01s_state.rx_frames++;
  if (cmd == UPPER_CMD_SET_VELOCITY)
  {
    upper_velocity_payload_t velocity;
    if (UpperProtocol_ParseVelocityPayload(payload, payload_len, &velocity) != 0U)
    {
      chassis_cmd_t chassis_cmd = {
        .linear_x = velocity.linear_x,
        .angular_z = velocity.angular_z,
        .enable = velocity.enable,
        .source = CONTROL_SOURCE_ESP01S,
        .timestamp_ms = osKernelGetTickCount(),
      };
      (void)velocity.mode;
      (void)ControlManager_SetCommand(&chassis_cmd);
    }
  }
  else if (cmd == UPPER_CMD_ESTOP && payload_len == UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN)
  {
    ControlManager_SetEmergencyStop(payload[0]);
  }
}

static void Esp01sComm_ProcessByte(uint8_t byte)
{
  switch (esp01s_rx_state)
  {
    case ESP01S_RX_WAIT_HEAD0:
      if (byte == UPPER_PROTOCOL_HEAD_0)
      {
        esp01s_rx_state = ESP01S_RX_WAIT_HEAD1;
      }
      break;

    case ESP01S_RX_WAIT_HEAD1:
      esp01s_rx_state = (byte == UPPER_PROTOCOL_HEAD_1) ? ESP01S_RX_WAIT_LEN : ESP01S_RX_WAIT_HEAD0;
      break;

    case ESP01S_RX_WAIT_LEN:
      if (byte == 0U || byte > UPPER_PROTOCOL_CMD_LEN(UPPER_PROTOCOL_MAX_PAYLOAD))
      {
        esp01s_state.rx_length_errors++;
        Esp01sComm_ResetParser();
      }
      else
      {
        esp01s_frame_buf[0] = byte;
        esp01s_frame_len = byte;
        esp01s_frame_index = 0U;
        esp01s_rx_state = ESP01S_RX_WAIT_BODY;
      }
      break;

    case ESP01S_RX_WAIT_BODY:
      esp01s_frame_index++;
      esp01s_frame_buf[esp01s_frame_index] = byte;
      if (esp01s_frame_index >= (uint8_t)(esp01s_frame_len + 1U))
      {
        uint8_t checksum = esp01s_frame_buf[esp01s_frame_index];
        uint8_t expect = UpperProtocol_Checksum8(esp01s_frame_buf, (uint16_t)esp01s_frame_len + 1U);
        if (checksum == expect)
        {
          uint8_t cmd = esp01s_frame_buf[1];
          const uint8_t *payload = &esp01s_frame_buf[2];
          uint8_t payload_len = (uint8_t)(esp01s_frame_len - 1U);
          Esp01sComm_HandleFrame(cmd, payload, payload_len);
        }
        else
        {
          esp01s_state.rx_checksum_errors++;
        }
        Esp01sComm_ResetParser();
      }
      break;

    default:
      Esp01sComm_ResetParser();
      break;
  }
}

static void Esp01sComm_PollRx(void)
{
  uint16_t write_pos;

  if (huart2.hdmarx == 0)
  {
    return;
  }

  write_pos = (uint16_t)(ESP01S_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
  if (write_pos >= ESP01S_RX_BUFFER_SIZE)
  {
    write_pos = 0U;
  }

  while (esp01s_rx_read_pos != write_pos)
  {
    Esp01sComm_ProcessByte(esp01s_rx_dma_buffer[esp01s_rx_read_pos]);
    esp01s_rx_read_pos++;
    if (esp01s_rx_read_pos >= ESP01S_RX_BUFFER_SIZE)
    {
      esp01s_rx_read_pos = 0U;
    }
  }
}

static void Esp01sComm_SendStatus(uint32_t now_ms)
{
  upper_status_payload_t status = {0};
  chassis_control_state_t chassis_state;
  encoder_state_t encoder_state;
  system_monitor_state_t monitor_state;
  imu_mpu6050_state_t imu_state;
  uint8_t payload_len;
  uint16_t frame_len;

  if ((now_ms - esp01s_last_status_ms) < ESP01S_STATUS_PERIOD_MS)
  {
    return;
  }
  if (huart2.gState != HAL_UART_STATE_READY)
  {
    return;
  }

  esp01s_last_status_ms = now_ms;
  ChassisControl_GetState(&chassis_state);
  EncoderDriver_GetState(&encoder_state);
  SystemMonitor_GetState(&monitor_state);
  ImuMpu6050_GetState(&imu_state);

  status.left_speed = chassis_state.left_actual_mps;
  status.right_speed = chassis_state.right_actual_mps;
  status.left_encoder = encoder_state.left_count;
  status.right_encoder = encoder_state.right_count;
  status.battery_voltage = monitor_state.battery_voltage;
  status.left_current = monitor_state.left_current_a;
  status.right_current = monitor_state.right_current_a;
  status.imu_accel[0] = imu_state.accel_x;
  status.imu_accel[1] = imu_state.accel_y;
  status.imu_accel[2] = imu_state.accel_z;
  status.imu_gyro[0] = imu_state.gyro_x;
  status.imu_gyro[1] = imu_state.gyro_y;
  status.imu_gyro[2] = imu_state.gyro_z;
  status.error_flags = monitor_state.error_flags;
  status.control_mode = monitor_state.control_mode;

  payload_len = UpperProtocol_BuildStatusPayload(&status, esp01s_status_payload, sizeof(esp01s_status_payload));
  frame_len = UpperProtocol_BuildFrame(UPPER_CMD_STATUS, esp01s_status_payload, payload_len, esp01s_tx_frame, sizeof(esp01s_tx_frame));
  if (frame_len > 0U)
  {
    (void)HAL_UART_Transmit_DMA(&huart2, esp01s_tx_frame, frame_len);
    esp01s_state.tx_frames++;
  }
}

void Esp01sComm_Init(void)
{
  esp01s_rx_read_pos = 0U;
  esp01s_last_status_ms = 0U;
  esp01s_state = (esp01s_comm_state_t){0};
  Esp01sComm_ResetParser();
  (void)HAL_UART_Receive_DMA(&huart2, esp01s_rx_dma_buffer, ESP01S_RX_BUFFER_SIZE);
}

void Esp01sComm_Update(void)
{
  uint32_t now_ms = osKernelGetTickCount();
  Esp01sComm_PollRx();
  Esp01sComm_SendStatus(now_ms);
}

void Esp01sComm_GetState(esp01s_comm_state_t *state)
{
  if (state != 0)
  {
    *state = esp01s_state;
  }
}
