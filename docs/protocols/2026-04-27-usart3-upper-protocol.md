# USART3 上位机协议

## 帧格式

```text
0xA5 0x5A | length | cmd | payload | checksum8
```

- `length`：`cmd + payload` 的字节数，最小为 1。
- `checksum8`：从 `length` 到 payload 末尾的逐字节累加低 8 位。
- 多字节字段：小端序。
- 浮点字段：IEEE-754 `float`，4 字节小端序。

## MCU 接收命令

- `0x01 SET_VELOCITY`
  - payload：`linear_x(float) + angular_z(float) + enable(uint8) + mode(uint8)`。
  - MCU 只更新统一 `ControlManager` 命令，不直接输出 PWM。
- `0x02 ESTOP`
  - payload：`enabled(uint8)`。
  - 非 0 表示急停使能，0 表示清除急停状态。

## MCU 状态上报

- `0x81 STATUS`
  - payload：`left_speed(float), right_speed(float), left_encoder(int32), right_encoder(int32), battery_voltage(float), left_current(float), right_current(float), imu_accel[3](int16), imu_gyro[3](int16), error_flags(uint32), control_mode(uint8)`。
  - 当前周期：50 ms。
