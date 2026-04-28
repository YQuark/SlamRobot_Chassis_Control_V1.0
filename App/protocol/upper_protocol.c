#include "upper_protocol.h"

#include <string.h>

static void UpperProtocol_WriteU32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xFFU);
  out[1] = (uint8_t)((value >> 8) & 0xFFU);
  out[2] = (uint8_t)((value >> 16) & 0xFFU);
  out[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void UpperProtocol_WriteI32(uint8_t *out, int32_t value)
{
  UpperProtocol_WriteU32(out, (uint32_t)value);
}

static void UpperProtocol_WriteI16(uint8_t *out, int16_t value)
{
  uint16_t raw = (uint16_t)value;
  out[0] = (uint8_t)(raw & 0xFFU);
  out[1] = (uint8_t)((raw >> 8) & 0xFFU);
}

static void UpperProtocol_WriteFloat(uint8_t *out, float value)
{
  uint32_t raw = 0U;
  (void)memcpy(&raw, &value, sizeof(raw));
  UpperProtocol_WriteU32(out, raw);
}

static uint32_t UpperProtocol_ReadU32(const uint8_t *in)
{
  return ((uint32_t)in[0]) |
         ((uint32_t)in[1] << 8) |
         ((uint32_t)in[2] << 16) |
         ((uint32_t)in[3] << 24);
}

static float UpperProtocol_ReadFloat(const uint8_t *in)
{
  uint32_t raw = UpperProtocol_ReadU32(in);
  float value = 0.0f;
  (void)memcpy(&value, &raw, sizeof(value));
  return value;
}

uint8_t UpperProtocol_Checksum8(const uint8_t *data, uint16_t length)
{
  uint8_t checksum = 0U;

  if (data == 0)
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < length; ++i)
  {
    checksum = (uint8_t)(checksum + data[i]);
  }

  return checksum;
}

uint16_t UpperProtocol_BuildFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint16_t out_len)
{
  uint8_t cmd_len = UPPER_PROTOCOL_CMD_LEN(payload_len);
  uint16_t frame_len = (uint16_t)cmd_len + 4U;

  if (out == 0 || out_len < frame_len || payload_len > UPPER_PROTOCOL_MAX_PAYLOAD)
  {
    return 0U;
  }
  if (payload_len > 0U && payload == 0)
  {
    return 0U;
  }

  out[0] = UPPER_PROTOCOL_HEAD_0;
  out[1] = UPPER_PROTOCOL_HEAD_1;
  out[2] = cmd_len;
  out[3] = cmd;
  if (payload_len > 0U)
  {
    (void)memcpy(&out[4], payload, payload_len);
  }
  out[frame_len - 1U] = UpperProtocol_Checksum8(&out[2], (uint16_t)cmd_len + 1U);
  return frame_len;
}

uint8_t UpperProtocol_ParseVelocityPayload(const uint8_t *payload, uint8_t payload_len, upper_velocity_payload_t *velocity)
{
  if (payload == 0 || velocity == 0 || payload_len != UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN)
  {
    return 0U;
  }

  velocity->linear_x = UpperProtocol_ReadFloat(&payload[0]);
  velocity->angular_z = UpperProtocol_ReadFloat(&payload[4]);
  velocity->enable = payload[8];
  velocity->mode = payload[9];
  return 1U;
}

uint8_t UpperProtocol_BuildStatusPayload(const upper_status_payload_t *status, uint8_t *out, uint8_t out_len)
{
  uint8_t offset = 0U;

  if (status == 0 || out == 0 || out_len < UPPER_PROTOCOL_STATUS_PAYLOAD_LEN)
  {
    return 0U;
  }

  UpperProtocol_WriteFloat(&out[offset], status->left_speed);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteFloat(&out[offset], status->right_speed);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteI32(&out[offset], status->left_encoder);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteI32(&out[offset], status->right_encoder);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteFloat(&out[offset], status->battery_voltage);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteFloat(&out[offset], status->left_current);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteFloat(&out[offset], status->right_current);
  offset = (uint8_t)(offset + 4U);

  for (uint8_t i = 0U; i < 3U; ++i)
  {
    UpperProtocol_WriteI16(&out[offset], status->imu_accel[i]);
    offset = (uint8_t)(offset + 2U);
  }
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    UpperProtocol_WriteI16(&out[offset], status->imu_gyro[i]);
    offset = (uint8_t)(offset + 2U);
  }

  UpperProtocol_WriteU32(&out[offset], status->error_flags);
  offset = (uint8_t)(offset + 4U);
  out[offset] = status->control_mode;

  return UPPER_PROTOCOL_STATUS_PAYLOAD_LEN;
}
