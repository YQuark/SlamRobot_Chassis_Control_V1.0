#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_CONTROL_PERIOD_MS        10U
#define CHASSIS_ENCODER_PERIOD_MS        10U
#define CHASSIS_ADC_PERIOD_MS            20U
#define CHASSIS_LED_PERIOD_MS            1U
#define UPPER_UART_TASK_PERIOD_MS        5U
#define UPPER_UART_STATUS_PERIOD_MS      50U

#define CHASSIS_CMD_TIMEOUT_MS           500U
#define CHASSIS_PWM_MAX_PERMILLE         900
#define CHASSIS_PWM_DEADBAND_PERMILLE    0

#define CHASSIS_WHEEL_RADIUS_M           0.0f /* TODO: confirm wheel radius before speed conversion. */
#define CHASSIS_WHEEL_BASE_M             0.0f /* TODO: confirm wheel track before angular command use. */

#define CHASSIS_ENCODER_BASE_PPR         11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT  4.0f
#define CHASSIS_MOTOR_GEAR_RATIO         56.0f

#define CHASSIS_LEFT_MOTOR_DIR           1
#define CHASSIS_RIGHT_MOTOR_DIR          1
#define CHASSIS_LEFT_ENCODER_DIR         1
#define CHASSIS_RIGHT_ENCODER_DIR        1

#define ADC_MONITOR_CHANNEL_COUNT        3U
#define ADC_MONITOR_VREF_V               3.3f
#define ADC_MONITOR_RESOLUTION_COUNTS    4095.0f
#define ADC_MONITOR_BATTERY_DIVIDER      1.0f  /* TODO: set resistor divider ratio. */
#define ADC_MONITOR_CURRENT_ZERO_V       0.0f  /* TODO: set current sensor zero point. */
#define ADC_MONITOR_CURRENT_V_PER_A      0.0f  /* TODO: set current sensor scale before reporting amps. */

#define BATTERY_LOW_WARN_V               10.5f
#define MOTOR_RATED_CURRENT_A            0.65f
#define MOTOR_STALL_CURRENT_A            2.4f

#ifdef __cplusplus
}
#endif

#endif
