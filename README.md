# Penix

PnX 是面向 STM32 与 ThreadX 的可复用下位机模板，包含配置生成器、BSP、设备、功能模块、通用库和应用入口。具体机器人控制策略由应用实现；当前 `app_start()` 初始化基础日志，尚未启动机器人控制任务。

项目需求与验收见 [.agents/docs/SPEC.md](.agents/docs/SPEC.md)，架构与实现边界见 [.agents/docs/DESIGN.md](.agents/docs/DESIGN.md)，协作规则见 [AGENTS.md](AGENTS.md)。

## 目录与职责

- `configs/`：全部 PnX 配置源，包括 IOC、固定绑定、参数、设备组合、preset、工具链与产品链接脚本。使用说明见 [配置入口](configs/README.md)。
- `boards/`：CubeMX 生成代码与原生工程，供固件构建消费；配置解析不依赖该目录。
- `bsp/<板型>/<外设>/`：板级外设差异；`bsp/shared/`：两板共用的 USB、DWT 实现；`devices/`：设备操作、协议、单位和反馈，不创建线程。
- `modules/`：AHRS、遥控器、裁判系统等功能；`libs/`：通用库。
- `app/app.cpp`：机器人应用启动入口。按项目需要在 `app_start()` 装配模块和控制任务；配置启用不等于运行启动。

## 配置与构建

需要 Arm GNU Toolchain、CMake 3.23 或更高版本和 Ninja。选择 `configs/boards/h723_dmmc02/` 或 `configs/boards/f407_cboard/`，按需编辑 `params.json` 与 `robot.json`，再 Configure 和 Build。参数只保留相对默认值的差异，优先级与各文件职责见 [配置入口](configs/README.md)。不要编辑 `build/` 中的生成文件。

H723 与 F407 均有仓库内 BSP 和 Debug/Release preset。支持边界及未验证事项见 [SPEC](.agents/docs/SPEC.md#未决事项)，实际构建证据见任务计划。

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

## 开发与验证

职责、资源与数据交接见 [DESIGN](.agents/docs/DESIGN.md)，编码、类型、命名与验收统一见 [SPEC](.agents/docs/SPEC.md)。

```powershell
python -m unittest discover -s .agents/tests -v
git diff --check
```

行为回归需要 Python、CMake 和主机 g++。主机测试、ARM 编译链接与实机运行分别验证，不能互相替代。

## 基础日志

`logging::write()` 写入固定 32 字节记录，不分配内存、不格式化字符串、不等待互斥锁。容量和最低等级由 configs 的 `logging` 参数生成。日志位于 `.noinit` RAM，软件或看门狗复位后可保留，断电后不可保留；调试器可直接查看 `pnx_log_records`、`pnx_log_next_sequence` 和 `pnx_log_boot_count`。日志用于补充故障现场，调用方仍须处理函数返回状态。

```cpp
(void)logging::write(logging::level::error, "imu config",
                     types::status::not_configured);
```
