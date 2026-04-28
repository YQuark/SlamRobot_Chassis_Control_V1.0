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
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  uint8_t online;
} imu_mpu6050_state_t;

void ImuMpu6050_Init(void);
void ImuMpu6050_Update(void);
void ImuMpu6050_GetState(imu_mpu6050_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
