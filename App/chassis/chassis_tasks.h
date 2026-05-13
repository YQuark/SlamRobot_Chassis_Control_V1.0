#ifndef CHASSIS_TASKS_H
#define CHASSIS_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

void ChassisTasks_InitHardware(void);
void Task_ChassisControl(void *argument);
void Task_EncoderUpdate(void *argument);
void Task_AdcMonitor(void *argument);
void Task_ImuUpdate(void *argument);
void Task_Led(void *argument);
void Task_UpperUart(void *argument);
void Task_Ps2(void *argument);
void Task_Esp01s(void *argument);

#ifdef __cplusplus
}
#endif

#endif
