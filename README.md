# Penix

PnX 是面向 STM32 与 ThreadX 的可复用下位机模板，包含配置生成器、BSP、设备、功能模块、通用库和应用入口。具体机器人控制策略由应用实现；当前 `app_start()` 为空，不包含独立诊断固件入口。

项目需求与验收见 [.agents/docs/SPEC.md](.agents/docs/SPEC.md)，架构与实现边界见 [.agents/docs/DESIGN.md](.agents/docs/DESIGN.md)，协作规则见 [AGENTS.md](AGENTS.md)。

设计以 MCU 闭环、硬件事实单一来源、板族共享语义、明确资源与时间所有权为原则。消息通道保存最新状态，不保证事件或样本无损；本地命令接受、外设提交和设备反馈也具有不同含义。当前实现与目标设计的区别见 [架构设计](.agents/docs/DESIGN.md)。

## 目录与职责

- `configs/`：全部 PnX 配置源，包括 IOC、固定绑定、参数、设备组合、preset、工具链与产品链接脚本。使用说明见 [配置入口](configs/README.md)。
- `boards/`：CubeMX 生成代码与原生工程，供固件构建消费；配置解析不依赖该目录。
- `bsp/<板型>/<外设>/`：两板各自的外设接口和实现；`devices/`：设备驱动。
- `modules/`：AHRS、遥控器、裁判系统等功能；`libs/`：通用库。
- `app/app.cpp`：机器人应用启动入口。按项目需要在 `app_start()` 初始化服务和控制任务。

## 配置与构建

需要 Arm GNU Toolchain、CMake 3.23 或更高版本和 Ninja。选择 `configs/boards/h723_dmmc02/` 或 `configs/boards/f407_cboard/`，按需编辑 `params.json` 与 `robot.json`，再 Configure 和 Build。参数只保留相对默认值的差异，优先级与各文件职责见 [配置入口](configs/README.md)。不要编辑 `build/` 中的生成文件。

H723 与 F407 均有仓库内 BSP，Debug/Release 已通过编译链接；两板应用启动钩子仍待接入，不能将链接成功视为机器人应用可运行。限制见 [SPEC 的未决事项](.agents/docs/SPEC.md#未决事项)。

```powershell
cmake --preset h723-debug
cmake --build --preset h723-debug
```

只检查配置无需生成板级代码或安装交叉编译器：

```powershell
cmake -DPNX_BOARD=h723_dmmc02 -DOUT_DIR=build/config-preview/h723 -P configs/cmake/configure.cmake
```

IOC 权威源已移入 `configs/boards/<板型>/`；上述预览同时导出显式指向根目录 `boards/<板型>/` 的 CubeMX 脚本，重新生成前请按配置指南核对输出目录。

F407 使用 `f407-debug` preset。`PNX_BOARD` 自动选择 `bsp/h723_dmmc02/` 或 `bsp/f407_cboard/`；开发外部 BSP 时，可用 `PNX_BSP_SOURCE_DIR` 指向含 target.cmake 的板型目录。构建产物位于 `build/<preset>/pnx_embedded.elf`。

配置工具应基于现有生成器校验，并使调试芯片与 ELF 路径匹配所选板型和构建类型。当前仓库提供编辑器上下文导出脚本，外部 PnX Config Editor、Cortex Kit 的安装与会话未核验；完整工作流目标见 DESIGN。硬件烧录和运行由操作者显式执行。

## 实现约束

函数应承担明确的校验、状态变更、资源所有权、协议转换或复用职责。仅转发参数与返回值的封装应审查其必要性；协议解析、资源清理和上下文边界所需的小函数应保留，不机械按调用次数合并。保留 ThreadX/HAL 要求的回调入口和有设备语义的公开操作；删除公开接口前核对所有调用方和两种板型的契约。
