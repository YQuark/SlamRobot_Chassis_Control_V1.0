#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_CONTROL_PERIOD_MS        10U
#define CHASSIS_ENCODER_PERIOD_MS        10U
#define CHASSIS_ADC_PERIOD_MS            20U
#define CHASSIS_IMU_PERIOD_MS            20U
#define CHASSIS_LED_PERIOD_MS            1U
#define UPPER_UART_TASK_PERIOD_MS        5U
#define UPPER_UART_STATUS_PERIOD_MS      50U

#define CHASSIS_CMD_TIMEOUT_MS           500U
#define CHASSIS_PWM_MAX_PERMILLE         900
#define CHASSIS_PWM_DEADBAND_PERMILLE    0

#define CHASSIS_MAX_LINEAR_MPS           0.5f /* Limit accepted linear_x command. */
#define CHASSIS_OPENLOOP_FULL_MPS        0.5f /* Open-loop command mapped to CHASSIS_PWM_MAX_PERMILLE. */
#define CHASSIS_ANGULAR_EPSILON_RPS      0.0001f

#define CHASSIS_WHEEL_RADIUS_M           0.035f
#define CHASSIS_WHEEL_BASE_M             0.0f /* TODO: set after the mechanical frame is finalized. */
#define CHASSIS_MIN_ENCODER_DT_MS        1U
#define CHASSIS_MAX_ENCODER_DT_MS        100U

#define CHASSIS_ENCODER_BASE_PPR         11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT  4.0f
#define CHASSIS_MOTOR_GEAR_RATIO         56.0f

#define CHASSIS_LEFT_MOTOR_DIR           -1
#define CHASSIS_RIGHT_MOTOR_DIR          1
#define CHASSIS_LEFT_ENCODER_DIR         1
#define CHASSIS_RIGHT_ENCODER_DIR        -1

#define ADC_MONITOR_CHANNEL_COUNT        3U
#define ADC_MONITOR_VREF_V               3.3f
#define ADC_MONITOR_RESOLUTION_COUNTS    4095.0f

/* VBAT_MAIN -> R19 47k -> divider node -> R21 10k -> GND.
 * ADC input has R20 220R in series, C32 100nF to GND, and BAT54S clamps.
 */
#define ADC_MONITOR_BATTERY_R_UPPER_OHM  47000.0f
#define ADC_MONITOR_BATTERY_R_LOWER_OHM  10000.0f
#define ADC_MONITOR_BATTERY_DIVIDER      ((ADC_MONITOR_BATTERY_R_UPPER_OHM + ADC_MONITOR_BATTERY_R_LOWER_OHM) / ADC_MONITOR_BATTERY_R_LOWER_OHM)

/* TB67H450FNG low-side current sense conditions:
 * EP/GND/RS node -> 0.1 ohm shunt -> system GND, no amplifier, no bias.
 * Current is treated as magnitude only; motor direction is not encoded here.
 * Formula: I = abs((Vadc - MOTOR_CURRENT_ZERO_V) / MOTOR_CURRENT_VOLTS_PER_AMP).
 * Shunt power: P = I^2 * R. 0.65 A -> 0.042 W, 2.4 A -> 0.576 W.
 * Confirm the physical shunt power rating has margin for stall transients.
 */
#define MOTOR_CURRENT_SHUNT_OHM          0.1f
#define MOTOR_CURRENT_ZERO_V             0.0f
#define MOTOR_CURRENT_VOLTS_PER_AMP      0.1f
#define MOTOR_CURRENT_FILTER_ALPHA       0.25f
#define MOTOR_CURRENT_LIMIT_A            0.8f
#define MOTOR_OVERCURRENT_DEBOUNCE_COUNT 5U

#define ADC_MONITOR_CALIBRATION_ENABLED  1U

#define CHASSIS_PID_ENABLED              1U
#define CHASSIS_PID_CORRECTION_LIMIT     300.0f

#define CHASSIS_PID_KP_L                 900.0f /* Initial low-speed tuning value. */
#define CHASSIS_PID_KI_L                 180.0f
#define CHASSIS_PID_KD_L                 0.0f
#define CHASSIS_PID_INTEGRAL_LIMIT_L     0.6f
#define CHASSIS_PID_KP_R                 900.0f /* Initial low-speed tuning value. */
#define CHASSIS_PID_KI_R                 180.0f
#define CHASSIS_PID_KD_R                 0.0f
#define CHASSIS_PID_INTEGRAL_LIMIT_R     0.6f

#define BATTERY_LOW_WARN_V               10.5f
#define BATTERY_LOW_MONITOR_ENABLED      0U   /* TODO: add low-voltage warning, power limit, and stop protection. */
#define MOTOR_RATED_CURRENT_A            0.65f
#define MOTOR_STALL_CURRENT_A            2.4f

#ifdef __cplusplus
}
#endif

#endif
