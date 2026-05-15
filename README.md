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
- LED 状态闪烁。
- MPU6050 基础初始化、原始数据读取与单位换算。
- PS2 手柄输入解析与本地遥控接入。
- ESP01S USART2 二进制协议接入、Arduino 固件与中文无线调试页面。

暂未完成：

- 电机 PID 闭环精调。
- MPU6050 零偏校准、姿态融合与控制接入。
- PS2 手柄实物方向、摇杆死区和按键手感标定。
- ESP01S 页面与实际网络环境联调。
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
| ESP01S | USART2 / PD5, PD6 | Wi-Fi 调试与遥控链路 |
| 树莓派上位机 | USART3 / PB10, PB11 | ROS2 底盘控制通道 |
| MPU6050 | I2C1 / PB6, PB7 | IMU 通信 |
| IMU 中断 | PB5 EXTI | `IMU_INT` |
| PS2 | PA4, PA5, PA6, PA7 | CLK, CS, CMD, DAT |
| LED | PC2 | 状态闪烁 |

## 目录结构

```text
Core/
  Inc/ Src/                 CubeMX 生成代码与 USER CODE 接入点

BSP/
  motor/                    TB67H450FNG PWM 输出封装
  encoder/                  TIM 编码器读取与速度计算
  adc/                      ADC DMA 采样与电压/电流换算接口
  imu/                      MPU6050 预留驱动
  ps2/                      PS2 手柄协议与控制映射
  esp01s/                   ESP01S USART2 通信链路
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

ESP01S/
  ESP01S/                   ESP-01S Arduino 固件与内置中文 Web 页面
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
| `Task_Ps2` | 20 ms | PS2 采样、遥控映射与统一命令提交 |
| `Task_Esp01s` | 5 ms | USART2 帧解析与 ESP01S 状态帧回传 |
| `Task_Led` | 50 ms | 状态闪烁和故障快闪 |

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
| `header` | 输出 VOFA+ CSV 字段名 |
| `i2cscan` | 只探测 IMU 常用地址 `0x68/0x69`，避免全总线扫描导致调试任务长时间阻塞 |
| `imutest` | 单次读取 `0x68/0x69` 的 `WHO_AM_I` 与 HAL 错误码，并打印当前 IMU 缓存状态 |
| `imu 1` / `imu 0` | 启用或关闭 IMU 周期读取；默认关闭，避免 I2C 异常影响底盘调试 |
| `imuprobe` | 只读探测 IMU 地址和 `WHO_AM_I`，不写寄存器，不启动周期读取 |
| `imuinit` | 只读初始化并进入 `online=1`，不写配置寄存器，避免当前模块写寄存器卡死 |
| `imuread` | 发起一次非阻塞 IMU 采样读取，通过后续 `status/log` 查看结果 |
| `imuwrite R V` | 单次写 IMU 寄存器并读回，`R/V` 支持 `0x` 十六进制 |
| `log 1` / `log 0` | 开启或关闭 500 ms VOFA+ CSV 周期日志 |
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

VOFA+ 使用方式：

- 协议选择 FireWater/CSV。
- `log 1` 会先输出字段名，然后周期输出逗号分隔数据。
- 为避免嵌入式浮点 printf 开销，日志使用定标整数：速度 `mm/s`、电压 `mV`、电流 `mA`、加速度 `mg`、角速度 `mdps`。

CSV 字段：

```text
t_ms,left_mms,right_mms,left_target_mms,right_target_mms,left_req_mms,right_req_mms,left_err_mms,right_err_mms,left_pwm,right_pwm,vbat_mv,left_current_ma,right_current_ma,acc_x_mg,acc_y_mg,acc_z_mg,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,imu_online,imu_addr,imu_who,imu_err,imu_errcnt,imu_transfer,imu_i2cerr,imu_read_ms,error_flags,control_source,ps2_online,ps2_drive,ps2_macro,ps2_macro_btn,ps2_lx,ps2_ly,ps2_rx,ps2_ry,ps2_ok,ps2_fail
```

注意：

- `motor` 是开环测试命令，主要用于硬件方向、电机和编码器验证。
- `vel` 走统一控制命令和速度闭环，用于低速 PID 初调。
- 当前轮距按 0.178 m 物理测量值初始化，角速度和快捷旋转仍需通过实测继续修正有效轮距。
- 当前 MPU6050 默认使用 I2C1，优先探测地址 `0x68`，失败后尝试 `0x69`。
- 上电默认不主动读取 IMU；`imu 1` 只打开周期读取允许标志，不会立刻访问 I2C。
- 周期读取只在 `imuinit` 成功并进入 `online=1` 后执行；当前 `imuinit` 不写 IMU 配置寄存器。
- `imu 0` 只暂停周期读取，不清除 `online` 缓存状态；再次 `imu 1` 会继续使用上次 `imuinit` 成功后的在线状态。
- IMU 采样读取使用 I2C1 中断非阻塞模式，`imu_transfer` 显示状态：`0` 空闲，`1` 读取中，`2` 完成待解析，`3` 错误，`4` 超时。
- `imuwrite` 是危险诊断命令；当前硬件实测写 `0x6B/0x1A/0x1B` 可能导致 I2C 卡死，正常调试不要使用。
- 若 `i2cscan` 探测不到 `0x68/0x69`，优先检查 PB6/PB7 是否接反、GND 是否共地、AD0 电平、电源电平和模块外部上拉。
- `WHO_AM_I=0x68` 按 MPU6050 处理，`WHO_AM_I=0x70` 按 MPU6500/兼容寄存器器件处理；若上线后数据仍为 0，需要继续核对模块丝印和具体芯片型号。
- 调试期 I2C1 使用 100 kHz 并开启 MCU 内部上拉；最终硬件仍建议确认外部上拉。

## USART2 / USART3 共用协议

USART2 的 ESP01S 链路与 USART3 的上位机链路共用二进制帧：

```text
0xA5 0x5A | length | cmd | payload | CRC8
```

- `length` 为 `cmd + payload` 字节数。
- `CRC8` 为 CRC8-MAXIM 校验值（多项式 0x31，查表法），校验域从 `length` 到 payload 末尾。
- 多字节字段为小端序。
- `float` 为 IEEE-754 32 位小端序。

当前命令：

| 命令 | 值 | 方向 | Payload |
| --- | --- | --- | --- |
| `SET_VELOCITY` | `0x01` | USART2/USART3 到 MCU | `linear_x(float), angular_z(float), enable(uint8), mode(uint8)` |
| `ESTOP` | `0x02` | USART2/USART3 到 MCU | `enabled(uint8)` |
| `STATUS` | `0x81` | MCU 到 USART2/USART3 | 左右速度、编码器、电池电压、电流、IMU 摘要、错误状态、控制模式 |

详细说明见：

- `docs/protocols/2026-04-27-usart3-upper-protocol.md`

## PS2 与 ESP01S

- PS2 采用 GPIO 软件时序读取，优先配置模拟摇杆模式。
- PS2 在线时读取摇杆和快捷键；只有摇杆明显输入或快捷动作运行时才占用 PS2 控制源，摇杆回中后释放 PS2 控制源。
- 摇杆映射为统一底盘命令：左摇杆纵向线性控制 `linear_x`，右摇杆横向线性控制 `angular_z`。
- `L1`/`R1` 触发原地左/右定时长旋转，`L2`/`R2` 触发较短定时长旋转；任意明显摇杆输入会立即打断快捷动作。当前快捷动作不是闭环角度控制，实际角度需要结合速度斜坡、有效轮距和地面情况实测校准。
- 控制源优先级为：急停/故障停机 > USART3 上位机 > PS2 > ESP01S；USART1 调试控制仍保留独立维护入口。
- `ESP01S/ESP01S/ESP01S.ino` 提供 AP+STA、中文网页遥控、中文状态面板、调试事件日志和 Wi-Fi 配网页面。
- ESP01S 网页只负责把人的操作转换成正式串口帧，不直接代表 MCU 内部控制状态；最终以 MCU 回传 `STATUS` 为准。
- 当前 `CHASSIS_WHEEL_BASE_M` 已按 0.178 m 物理轮距初始化，后续仍需通过原地旋转实测继续修正有效轮距。

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

### 版本显示

- CMake 默认将固件版本注入为 `dev`，USART1 调试控制台启动时会打印该版本文本。
- 正式发布时，GitHub Actions 会在构建阶段临时注入你手动输入的语义化版本号，例如 `0.1.1`。
- 这样可以区分本地调试包与正式 release 附件。

### GitHub Actions

- `Firmware Build` 工作流覆盖 `master` 推送和面向 `master` 的 Pull Request，复用现有 `Debug` / `Release` Preset 做编译验收。
- `Release Firmware` 工作流需要在 GitHub Actions 页面手动触发，并输入 `X.Y.Z` 版本号。
- 手动发布时会构建 `Debug` 固件、创建 `vX.Y.Z` tag、生成 GitHub Release，并将固定发布摘要与“相对上一版 tag 的完整提交 changelog”写入发布页。
- GitHub Release 仅附带带版本号的 Debug `.elf` 与 `.map` 产物。

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
3. 验证 LED 闪烁状态。
4. 读取 ADC 原始值，确认三通道顺序。
5. 手动转动左右轮，确认 TIM4/TIM8 编码器计数方向。
6. 单电机低占空比开环测试，确认 TB67H450FNG 输入方向。
7. 修正左右电机方向和编码器方向配置。
8. 验证 PS2 的连接、摇杆线性输入、空闲释放、快捷旋转打断和掉线释放。
9. 烧录 ESP01S Arduino 固件，访问中文调试页面并确认状态帧刷新。
10. 使用 USART1 `vel V 0` 进行低速直线速度环初调。
11. 验证电流软限流和过流 fault-stop。
12. 接入 USART3，验证上位机发送速度命令和 MCU 状态帧。
13. 联调两轮差速前进、后退、原地转向和急停。

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
