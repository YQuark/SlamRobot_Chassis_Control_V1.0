# 两轮差速底盘硬件框架执行计划

## Internal Grade

L：单工程内分层新增模块，顺序执行并在本地构建验证。

## Waves

1. 治理产物：写入需求、执行计划和运行收据。
2. BSP/App：新增硬件封装、控制骨架、协议结构和任务函数。
3. 接入：在根 CMake 和 FreeRTOS USER CODE 区接入新增代码。
4. 验证：运行 CMake 配置和 Debug 构建，检查关键路径。
5. 清理：记录证明、清理结果和剩余风险。

## Ownership

- CubeMX 生成文件：只修改 `Core/Src/freertos.c` 的 USER CODE 区。
- 构建入口：只修改根 `CMakeLists.txt` 用户源和 include 区。
- 新增业务代码：写入 `BSP/` 与 `App/`。

## Verification

- `cmake --preset Debug`
- `cmake --build --preset Debug`
- `git diff -- CMakeLists.txt Core/Src/freertos.c BSP App docs outputs`

## Completion Rules

只有在构建验证完成并写入 cleanup receipt 后，才能声明首轮框架实现完成。若构建失败，必须报告失败命令和原因。
