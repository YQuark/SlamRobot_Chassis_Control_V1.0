#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temperature_raw;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  float accel_g[3];
  float temperature_c;
  float gyro_dps[3];
  uint32_t last_update_ms;
  uint32_t last_probe_ms;
  uint32_t last_read_ms;
  volatile uint32_t last_i2c_error;
  uint32_t error_count;
  uint8_t detected_addr;
  uint8_t who_am_i;
  uint8_t last_error;
  uint8_t online;
  uint8_t enabled;
  volatile uint8_t transfer_state;
} imu_mpu6050_state_t;

void ImuMpu6050_Init(void);
void ImuMpu6050_Update(void);
void ImuMpu6050_GetState(imu_mpu6050_state_t *state);
uint8_t ImuMpu6050_SetEnabled(uint8_t enabled);
uint8_t ImuMpu6050_IsEnabled(void);
uint8_t ImuMpu6050_ProbeNow(void);
uint8_t ImuMpu6050_ConfigNow(void);
uint8_t ImuMpu6050_RequestRead(void);
uint8_t ImuMpu6050_WriteRegTest(uint8_t reg, uint8_t value, uint8_t *readback, uint32_t *hal_error);
uint8_t ImuMpu6050_TestAddress(uint8_t addr7, uint8_t *who_am_i, uint32_t *hal_error);
uint8_t ImuMpu6050_IsDeviceReady(uint8_t addr7);

enum
{
  IMU_MPU6050_ERROR_NONE = 0,
  IMU_MPU6050_ERROR_NO_DEVICE = 1,
  IMU_MPU6050_ERROR_WHOAMI_READ_FAIL = 2,
  IMU_MPU6050_ERROR_WHOAMI_MISMATCH = 3,
  IMU_MPU6050_ERROR_CONFIG_FAIL = 4,
  IMU_MPU6050_ERROR_READ_FAIL = 5,
  IMU_MPU6050_ERROR_READ_TIMEOUT = 6
};

enum
{
  IMU_MPU6050_TRANSFER_IDLE = 0,
  IMU_MPU6050_TRANSFER_READING = 1,
  IMU_MPU6050_TRANSFER_DONE = 2,
  IMU_MPU6050_TRANSFER_ERROR = 3,
  IMU_MPU6050_TRANSFER_TIMEOUT = 4
};

#ifdef __cplusplus
}
#endif

#endif
