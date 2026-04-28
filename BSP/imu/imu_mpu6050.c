#include "imu_mpu6050.h"

static imu_mpu6050_state_t imu_state;

void ImuMpu6050_Init(void)
{
  imu_state = (imu_mpu6050_state_t){0};
}

void ImuMpu6050_Update(void)
{
  /* TODO: validate MPU6050 address and initialization sequence on I2C1. */
}

void ImuMpu6050_GetState(imu_mpu6050_state_t *state)
{
  if (state != 0)
  {
    *state = imu_state;
  }
}
