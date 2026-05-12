# USART3 Upper Computer Protocol

## Frame Format

```text
0xA5 0x5A | length | cmd | payload | checksum8
```

- `length`: byte count of `cmd + payload`, minimum `1`.
- `checksum8`: low 8 bits of the byte sum from `length` through the last payload byte.
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
- If wheel base is not configured, any non-zero `angular_z` rejects the whole frame.
- Rejected `angular_z` frames clear the active command and keep the chassis stopped.
- During emergency stop or latched fault stop, `SET_VELOCITY` is ignored and is not stored as a recoverable command.
- `enable == 0` clears the active command and keeps the chassis stopped.

The communication layer never writes motor PWM directly. Accepted commands are submitted to `ControlManager`; PWM output is only performed by the chassis control task.

### `0x02 ESTOP`

Payload:

```text
enabled(uint8)
```

- Non-zero enables emergency stop.
- `0` clears emergency stop.
- Entering or clearing emergency stop always clears the active command.
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

Current status period: `UPPER_UART_STATUS_PERIOD_MS`.

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
