#include "chassis_tasks.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_control.h"
#include "cmsis_os2.h"
#include "encoder_driver.h"
#include "imu_mpu6050.h"
#include "led_status.h"
#include "system_monitor.h"
#include "upper_uart.h"
#include "usart1_debug_console.h"

void ChassisTasks_InitHardware(void)
{
  EncoderDriver_Init();
  AdcMonitor_Init();
  ImuMpu6050_Init();
  LedStatus_Init();
  SystemMonitor_Init();
  ChassisControl_Init();
  UpperUart_Init();
  Usart1DebugConsole_Init();
}

void Task_ChassisControl(void *argument)
{
  (void)argument;
  for (;;)
  {
    ChassisControl_Step(osKernelGetTickCount());
    osDelay(CHASSIS_CONTROL_PERIOD_MS);
  }
}

void Task_EncoderUpdate(void *argument)
{
  (void)argument;
  for (;;)
  {
    EncoderDriver_Update(osKernelGetTickCount());
    osDelay(CHASSIS_ENCODER_PERIOD_MS);
  }
}

void Task_AdcMonitor(void *argument)
{
  (void)argument;
  for (;;)
  {
    AdcMonitor_Update();
    SystemMonitor_Update();
    osDelay(CHASSIS_ADC_PERIOD_MS);
  }
}

void Task_ImuUpdate(void *argument)
{
  (void)argument;
  for (;;)
  {
    ImuMpu6050_Update();
    osDelay(CHASSIS_IMU_PERIOD_MS);
  }
}

void Task_Led(void *argument)
{
  (void)argument;
  for (;;)
  {
    LedStatus_TaskStep(CHASSIS_LED_PERIOD_MS);
    osDelay(CHASSIS_LED_PERIOD_MS);
  }
}
