# Penix

Penix 是 STM32 + ThreadX 的嵌入式机器人架构。本仓库保留可构建的板级工程、配置生成器、BSP、设备、功能模块、通用库和应用入口；不包含诊断示例。

## 目录与职责

- `boards/`：IOC、CubeMX 生成代码、链接脚本与固定硬件资源。
- `configs/`：板型参数、设备组合、默认值与配置生成器。
- `bsp/`：外设接口和板族实现；`devices/`：设备驱动。
- `modules/`：AHRS、遥控器、裁判系统等功能；`libs/`：通用库。
- `app/app.cpp`：机器人应用启动入口。按项目需要在 `app_start()` 初始化服务和控制任务。

## 配置与构建

需要 Arm GNU Toolchain、CMake 和 Ninja。编辑 `configs/boards/<board>/params.json` 与 `robot.json`，然后重新 Configure 和 Build。IOC 决定引脚、时钟、DMA 和外设初始化；`board.json` 描述固定板级资源。不要编辑 `build/` 中的生成文件。

```powershell
cmake --preset h723-debug
cmake --build --preset h723-debug
```

F407 选择对应的 `f407-debug` preset 和 F4 BSP。`PNX_BOARD` 不会自动切换 BSP 代码；需要另一份 BSP 工作树时，设置 `PNX_BSP_SOURCE_DIR`。构建产物位于 `build/<preset>/pnx_embedded.elf`。

PnX Config Editor 可用于编辑 JSON 和执行 Configure；Cortex Kit 使用调试配置中的芯片名及 ELF 路径。切板或切换构建类型后需核对这两项。硬件烧录和运行由操作者显式执行。

## 实现约束

函数应承担明确的校验、状态变更、资源所有权、协议转换或复用职责。仅被调用一次、只检查参数后继续转调的内部函数，应优先合并到调用处。保留 ThreadX/HAL 要求的回调入口和有设备语义的公开操作；删除公开接口前核对所有调用方和两种板型的契约。
