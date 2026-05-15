# USART2 / USART3 Shared Chassis Protocol

## Frame Format

```text
0xA5 0x5A | length | cmd | payload | CRC8
```

- `length`: byte count of `cmd + payload`, minimum `1`.
- `CRC8`: CRC8-MAXIM checksum (polynomial 0x31, lookup-table), computed from `length` through the last payload byte.
- Multi-byte fields are little-endian.
- Floating-point fields are IEEE-754 32-bit `float`, little-endian.

## MCU Receive Commands

### `0x01 SET_VELOCITY`

Payload:

```text
linear_x(float) + angular_z(float) + enable(uint8) + mode(uint8)
```

Acceptance rules:

- `linear_x` and `angular_z` must be finite values; NaN and Inf are rejected.
- `linear_x` is clamped by `CHASSIS_MAX_LINEAR_MPS`.
- Current firmware initializes `CHASSIS_WHEEL_BASE_M` to the measured physical value `0.178f`, so `angular_z` is accepted for controlled differential testing.
- If wheel base is later set to an invalid value, any non-zero `angular_z` rejects the whole frame and clears that source's command. If no higher-priority valid source remains, the chassis stays stopped.
- During emergency stop or latched fault stop, `SET_VELOCITY` is ignored and is not stored as a recoverable command.
- `enable == 0` clears the command for that source and keeps that source from continuing to own control.

The communication layer never writes motor PWM directly. Accepted commands are submitted to `ControlManager`; PWM output is only performed by the chassis control task.

The same frame format is used by:

- `USART3`: Raspberry Pi / ROS2 upper-computer link.
- `USART2`: ESP01S Arduino bridge with the built-in Chinese web console.

Control ownership is resolved in firmware, not by the transport layer:

```text
ESTOP / fault stop
  > USART3 upper computer
  > PS2 handheld controller
  > ESP01S web controller
```

### `0x02 ESTOP`

Payload:

```text
enabled(uint8)
```

- Non-zero enables emergency stop.
- `0` clears emergency stop.
- Entering or clearing emergency stop always clears every stored source command.
- After clearing emergency stop, motion resumes only after a new accepted `SET_VELOCITY`.

## MCU Status Command

### `0x81 STATUS`

Payload:

```text
left_speed(float)
right_speed(float)
left_encoder(int32)
right_encoder(int32)
battery_voltage(float)
left_current(float)
right_current(float)
imu_accel[3](int16)
imu_gyro[3](int16)
error_flags(uint32)
control_mode(uint8)
```

Current status periods:

- USART3: `UPPER_UART_STATUS_PERIOD_MS`.
- USART2 / ESP01S: `ESP01S_STATUS_PERIOD_MS`.

`imu_accel` and `imu_gyro` are cached raw MPU6050 register values. USART1 debug logs provide scaled VOFA+ channels in `mg` and `mdps` plus IMU diagnostic fields; the USART3 frame shape remains unchanged.

`control_mode` values:

```text
0 NONE
1 UPPER
2 PS2
3 ESP01S
4 DEBUG
```

## Error Flags

`error_flags` is a bit mask. Current MCU definitions:

```text
bit0 LOW_BATTERY
bit1 LEFT_OVERCURRENT
bit2 RIGHT_OVERCURRENT
bit3 ESTOP
bit4 FAULT_STOP
bit5 ENCODER_INVALID
```

Overcurrent faults are latched after debounce. A latched fault stops PWM through the chassis control layer and requires explicit clearing plus a new accepted velocity command before motion can resume.
