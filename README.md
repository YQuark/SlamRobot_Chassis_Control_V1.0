# F407_V1.0 两轮差速底盘控制工程

本工程是面向 ROS2 上位机控制的两轮差速移动机器人底盘控制程序，基于 STM32F407、HAL 和 FreeRTOS 开发。工程已由 STM32CubeMX 生成基础外设初始化代码，并在此基础上新增 BSP 驱动层、App 控制层、USART3 上位机协议和基础任务框架。

当前硬件结构已经从旧工程的四轮底盘调整为左右双电机两轮差速结构，后续开发不应继续沿用四轮、麦轮、四路编码器或旧电机参数假设。

## 当前状态

已实现：

- CubeMX 外设初始化框架。
- 双电机 TB67H450FNG PWM 驱动封装。
- 左右编码器 TIM 计数读取与速度换算框架。
- ADC DMA 三通道采样框架。
- FreeRTOS 任务骨架。
- USART3 上位机二进制协议框架。
- USART1 文本调试控制台，可用于电机、编码器、ADC 和闭环速度初调。
- 速度闭环初调框架与电流软限流/过流停车保护。
- LED 软件呼吸灯。
- MPU6050、PS2、ESP01S 预留接口。

暂未完成：

- 电机 PID 闭环精调。
- MPU6050 完整初始化、校准与读取。
- PS2 手柄实际协议解析。
- ESP01S 实际通信协议解析。
- ROS2 上位机节点。
- 轮距、PID 参数等硬件与控制参数实测标定。

## 硬件概要

| 类别 | 当前配置 |
| --- | --- |
| MCU | STM32F407 系列 |
| 软件框架 | STM32 HAL + FreeRTOS CMSIS-RTOS v2 |
| 底盘 | 左右两轮差速 |
| 电机 | JGB37-520-12100-EN，12 V，减速比 56 |
| 电机驱动 | TB67H450FNG，每个电机两路 PWM/方向输入 |
| 编码器 | AB 相增量式霍尔编码器，基础 11 PPR |
| IMU | MPU6050，I2C1 |
| 上位机 | 树莓派 4B，Ubuntu 22.04 + ROS2 |
| 上位机串口 | USART3，115200 8N1 |

编码器若按 AB 相四倍频计算，输出轴理论计数为：

```text
11 * 4 * 56 = 2464 counts/rev
```

实际换算仍应以当前 TIM 编码器模式、方向和硬件实测为准。

## 外设分配

| 功能 | MCU 外设/引脚 | 说明 |
| --- | --- | --- |
| 左电机 PWM1 | TIM1 CH1 / PA8 | `PWM_L_1` |
| 左电机 PWM2 | TIM1 CH4 / PA11 | `PWM_L_2` |
| 右电机 PWM1 | TIM3 CH3 / PC8 | `PWM_R_1` |
| 右电机 PWM2 | TIM3 CH4 / PC9 | `PWM_R_2` |
| 左编码器 | TIM4 CH1/CH2 / PD12/PD13 | `L_A`, `L_B` |
| 右编码器 | TIM8 CH1/CH2 / PC6/PC7 | `R_A`, `R_B` |
| 电池电压 | ADC1 IN0 / PA0 | `ADC_VBAT` |
| 左电机电流 | ADC1 IN1 / PA1 | `ADC_CUR_L` |
| 右电机电流 | ADC1 IN2 / PA2 | `ADC_CUR_R` |
| 调试串口 | USART1 / PA9, PA10 | 日志与调试 |
| ESP01S | USART2 / PD5, PD6 | Wi-Fi 通信预留 |
| 树莓派上位机 | USART3 / PB10, PB11 | ROS2 底盘控制通道 |
| MPU6050 | I2C1 / PB6, PB7 | IMU 通信 |
| IMU 中断 | PB5 EXTI | `IMU_INT` |
| PS2 | PA4, PA5, PA6, PA7 | CLK, CS, CMD, DAT |
| LED | PC2 | 软件呼吸灯 |

## 目录结构

```text
Core/
  Inc/ Src/                 CubeMX 生成代码与 USER CODE 接入点

BSP/
  motor/                    TB67H450FNG PWM 输出封装
  encoder/                  TIM 编码器读取与速度计算
  adc/                      ADC DMA 采样与电压/电流换算接口
  imu/                      MPU6050 预留驱动
  ps2/                      PS2 控制输入预留
  esp01s/                   ESP01S 通信预留
  led/                      LED 状态显示

App/
  chassis/                  底盘配置、差速控制和任务入口
  control/                  统一控制源管理
  debug/                    USART1 调试控制台
  monitor/                  电源、电流、错误状态汇总
  protocol/                 USART3 上位机协议与 UART 任务

docs/
  protocols/                通信协议说明
  requirements/             需求说明
  plans/                    阶段计划
```

## FreeRTOS 任务

当前在 `Core/Src/freertos.c` 的 USER CODE 区创建以下任务：

| 任务 | 周期 | 职责 |
| --- | --- | --- |
| `Task_ChassisControl` | 10 ms | 底盘目标处理、安全停车、后续 PID 输出 |
| `Task_EncoderUpdate` | 10 ms | 读取左右编码器并更新速度 |
| `Task_AdcMonitor` | 20 ms | 更新 ADC 原始值、电压、电流和错误状态 |
| `Task_UpperUart` | 5 ms | USART3 协议解析，50 ms 上报状态帧 |
| `Task_Usart1DebugConsole` | 10 ms | USART1 文本调试命令与日志输出 |
| `Task_Led` | 1 ms | 软件 PWM 呼吸灯和故障闪烁 |

约束：

- 只有底盘控制层可以调用电机驱动输出。
- USART3、PS2、ESP01S 任务只能更新统一控制命令，不允许直接写 PWM。
- USART1 调试任务只能通过底盘控制层或统一控制命令进行测试，不直接写 PWM。
- 复杂计算不要放在中断里。

## USART1 调试控制台

USART1 当前用于本地初步硬件联调，串口参数为：

```text
115200 8N1
```

常用命令：

| 命令 | 说明 |
| --- | --- |
| `help` | 显示命令列表 |
| `status` | 输出一次编码器、ADC、IMU 和底盘状态 |
| `log 1` / `log 0` | 开启或关闭 500 ms 周期日志 |
| `motor L R` | 左右电机开环 permille 测试，范围 `-900..900` |
| `left P` / `right P` | 单侧电机开环测试 |
| `vel V [W]` | 闭环速度测试，`V` 为 mm/s，`W` 为 mrad/s |
| `stop` | 停止开环测试并清除调试速度命令 |
| `estop 0|1` | 清除或设置软件急停 |

方向校准说明：

- 修正前实测 `motor -600 600` 时车体向前运动。
- 软件中已将左电机方向取反、右编码器方向取反。
- 修正后调试命令中 `motor 300 300` 应表示车体前进，并且左右速度反馈应同为正值。

调试建议：

```text
status
log 1
motor 300 300
stop
vel 100 0
stop
```

注意：

- `motor` 是开环测试命令，主要用于硬件方向、电机和编码器验证。
- `vel` 走统一控制命令和速度闭环，用于低速 PID 初调。
- 当前轮距仍未标定，角速度命令依赖 `CHASSIS_WHEEL_BASE_M`，未确认前优先测试 `vel V 0`。
- 当前 MPU6050 读取尚未完成，`IMU online=0` 不代表最终硬件结论。

## 上位机 USART3 协议

USART3 使用二进制帧：

```text
0xA5 0x5A | length | cmd | payload | checksum8
```

- `length` 为 `cmd + payload` 字节数。
- `checksum8` 为从 `length` 到 payload 末尾的逐字节累加低 8 位。
- 多字节字段为小端序。
- `float` 为 IEEE-754 32 位小端序。

当前命令：

| 命令 | 值 | 方向 | Payload |
| --- | --- | --- | --- |
| `SET_VELOCITY` | `0x01` | 上位机到 MCU | `linear_x(float), angular_z(float), enable(uint8), mode(uint8)` |
| `ESTOP` | `0x02` | 上位机到 MCU | `enabled(uint8)` |
| `STATUS` | `0x81` | MCU 到上位机 | 左右速度、编码器、电池电压、电流、IMU 摘要、错误状态、控制模式 |

详细说明见：

- `docs/protocols/2026-04-27-usart3-upper-protocol.md`

## 构建

工程使用 CMake Presets，默认工具链文件为：

```text
cmake/gcc-arm-none-eabi.cmake
```

需要本机安装并可访问：

- CMake 3.22 或更高版本。
- Ninja。
- `arm-none-eabi-gcc` 工具链。

常用命令：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

或构建 Release：

```bash
cmake --preset Release
cmake --build --preset Release
```

如果在 WSL 中构建，需要确认 `cmake`、`ninja` 和 `arm-none-eabi-gcc` 已加入 PATH。

## 配置与标定

主要配置集中在：

```text
App/chassis/chassis_config.h
```

当前必须后续实测确认的参数包括：

- 轮半径 `CHASSIS_WHEEL_RADIUS_M`。
- 轮距 `CHASSIS_WHEEL_BASE_M`。
- 左右电机方向 `CHASSIS_LEFT_MOTOR_DIR`、`CHASSIS_RIGHT_MOTOR_DIR`，当前按实测前进方向初步配置。
- 左右编码器方向 `CHASSIS_LEFT_ENCODER_DIR`、`CHASSIS_RIGHT_ENCODER_DIR`，当前按实测前进反馈同号初步配置。
- 电池电压分压比 `ADC_MONITOR_BATTERY_DIVIDER`。
- 电流采样零点和比例 `MOTOR_CURRENT_ZERO_V`、`MOTOR_CURRENT_VOLTS_PER_AMP`。
- PID 参数和死区补偿参数。
- 电流软限流阈值 `MOTOR_CURRENT_LIMIT_A` 与堵转保护阈值 `MOTOR_STALL_CURRENT_A`。

在未确认硬件参数前，不要把临时测试值当成最终参数提交。

## 调试建议

建议按以下顺序联调：

1. 构建通过，确认 CubeMX 初始化无误。
2. 上电确认 PWM 默认输出为 0，电机不误动作。
3. 验证 LED 呼吸灯。
4. 读取 ADC 原始值，确认三通道顺序。
5. 手动转动左右轮，确认 TIM4/TIM8 编码器计数方向。
6. 单电机低占空比开环测试，确认 TB67H450FNG 输入方向。
7. 修正左右电机方向和编码器方向配置。
8. 使用 USART1 `vel V 0` 进行低速直线速度环初调。
9. 验证电流软限流和过流 fault-stop。
10. 接入 USART3，验证上位机发送速度命令和 MCU 状态帧。
11. 联调两轮差速前进、后退、原地转向和急停。

## 开发规范

- 保持两轮差速模型，不引入四轮底盘或麦轮控制假设。
- CubeMX 生成文件只在 USER CODE 区添加必要接入逻辑。
- 业务逻辑优先放在 `App/` 和 `BSP/`。
- 硬件相关参数集中配置，避免散落在业务代码中。
- 通信层、PS2、ESP01S 不直接控制电机。
- 日志、IDE 配置、agent 运行产物和构建产物不提交。

Git 提交标题使用标准大写前缀和中文说明，例如：

```text
Feat: 新增 USART3 上位机协议
Fix: 修复右编码器方向配置
Docs: 完善工程 README
```
