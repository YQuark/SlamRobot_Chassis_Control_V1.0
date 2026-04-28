#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  MOTOR_SIDE_LEFT = 0,
  MOTOR_SIDE_RIGHT = 1
} motor_side_t;

typedef enum
{
  MOTOR_STOP_COAST = 0,
  MOTOR_STOP_BRAKE = 1
} motor_stop_mode_t;

void MotorDriver_Init(void);
void MotorDriver_SetPermille(motor_side_t side, int16_t permille);
void MotorDriver_Stop(motor_side_t side, motor_stop_mode_t mode);
void MotorDriver_StopAll(motor_stop_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
