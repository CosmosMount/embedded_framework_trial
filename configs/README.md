# PnX 配置入口

项目配置以本目录为唯一来源。`boards/` 是固件构建消费的 CubeMX 生成工程；解析配置、预览生成结果和导出编辑器上下文不需要该目录存在。

## 先选要修改的内容

| 需求 | 修改位置 | 说明 |
| --- | --- | --- |
| 选择板型、Debug/Release | `CMakePresets.json` | 根目录同名文件只负责 include，日常命令仍在仓库根目录运行 |
| 引脚、时钟、DMA、NVIC、外设模式 | `boards/<板型>/<板型>.ioc` | 使用 CubeMX 编辑；需重新生成外设代码 |
| 固定接线、设备占用的硬件资源、DMA 内存事实 | `boards/<板型>/board.json` | 不存应用策略；资源须确实存在于 IOC |
| 板族、CPU/FPU 编译选项、调试目标 | `boards/<板型>/board.cmake` | 构建身份；新增板型由加载器自动发现 |
| 链接内存布局 | `boards/<板型>/memory.ld` | PnX 的实际链接脚本，CubeMX 自带脚本不会覆盖它 |
| 改变策略参数、应用别名 | `boards/<板型>/params.json` | 只写相对默认值的差异 |
| 启用设备、指定电机组合 | `boards/<板型>/robot.json` | 保留消费者启用与设备配置，不能改 HAL 外设模式 |
| 通用默认值、板型默认值与建议值 | `defaults.json` | 建议值只用于引导，不会自动覆盖应用选择 |

本文的路径均相对于 `configs/`，命令相对于仓库根目录。板型标识为 `h723_dmmc02`、`f407_cboard`；不保留旧板名别名。

## 参数如何生效

`defaults.common` → `defaults.boards.<板型>` → `params.json`，后者优先，嵌套对象递归合并。`defaults.recommendations` 不参与这一合并。省略参数表示继承，不能把删除覆盖项理解为禁用功能；设备启用应按 `robot.json` 的实际字段配置。

目前 H723 的 params 只保留 UART 别名与 CAN ID 类型；F407 还覆盖 USBX 启用。AHRS、温控等与默认值一致的覆盖已经移除，有效生成结果保持不变。确需调整时，从 defaults 找到同名字段写入该板 params，避免复制完整默认表。

`*_ticks` 使用 ThreadX tick，不等于毫秒；`*_ms` 为毫秒；温控目标使用摄氏度，duty 字段为占空比。具体物理量、协议 raw 值及合法范围以生成器检查和消费端接口为准，不把所有数值统一当作 SI 原始输入。线程优先级数值越小优先级越高；调整时检查整条任务链。

## 无需板级代码的配置检查

需要 CMake 3.23 或更高版本。以下命令不启动 CubeMX、不编译、不操作硬件：

```powershell
cmake -DPNX_BOARD=h723_dmmc02 -DOUT_DIR=build/config-preview/h723 -P configs/cmake/configure.cmake
cmake -DPNX_BOARD=f407_cboard -DOUT_DIR=build/config-preview/f407 -P configs/cmake/configure.cmake
cmake -DPNX_BOARD=h723_dmmc02 -DPNX_EDITOR_OUTPUT=build/config-preview/editor.json -P configs/cmake/export_editor_context.cmake
```

每次预览生成 `config.hpp`、`robot_config.hpp`、`bsp_bindings.hpp`、`bsp_bindings.cpp`，另输出 `cubemx.script`。编辑器 JSON 的 IOC、board、params、robot 路径全部指向本目录。生成器是配置有效性的最终裁定者，编辑器不自行维护另一套规则。遇到错误修正源 JSON/IOC 后重新运行，不修补生成文件。

## 从 IOC 重新生成

IOC 权威源位于 `configs/boards/<板型>/`。预览命令输出的 `cubemx.script` 使用绝对 IOC 路径及显式 `project path`，将工程输出定向到仓库根的 `boards/<板型>/`。先查看该脚本，再按本机 CubeMX 的脚本模式执行；本任务未调用 CubeMX 验证重生成。命令语义见 [ST 官方 CLI 文档](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.0/en/docs/markup/CubeMX_CLI.html)。

使用 GUI 时同样显式设置生成目录，不能直接接受迁移 IOC 后的默认目录。CubeMX 若在输出目录生成 IOC 副本，它是派生产物；硬件修改须回到 configs 中的权威源。生成的板工程自带 CMake/preset/链接文件属于 CubeMX 产物，PnX 根构建不读取它们的产品配置，只接入 `cmake/stm32cubemx` 的源码目标。

`memory.ld` 起始于本次 CubeMX 生成的内存布局；H723 额外定义现有 BSP 要求的、位于 RAM_D1 的对齐 DMA 段及起止符号。以后修改 IOC 内存/堆栈设置时，需要明确审查并同步本文件；不在 Configure 时静默覆盖产品链接策略。

## 构建固件

```powershell
cmake --preset h723-debug
cmake --build --preset h723-debug
```

要求 `boards/<板型>/` 下已生成的 CubeMX 工程、Arm GNU Toolchain 和 Ninja。实际产品工具链为 `cmake/gcc-arm-none-eabi.cmake`，由当前配置选择 CPU/FPU 和 memory.ld，不加载 boards 下的工具链。F407 使用 `f407-debug`，自动选择 `bsp/f407_cboard/`；H723 对应 `bsp/h723_dmmc02/`。`PNX_BSP_SOURCE_DIR` 默认留空，仅开发外部 BSP 时覆盖。旧缓存曾保存平铺 BSP 路径，迁移时使用 `cmake --fresh --preset h723-debug` 清除该路径。Release 使用同板型的 release preset。

固件输出为 `build/<preset>/pnx_embedded.elf`。根工程的高级覆盖参数 `PNX_PARAMS_OVERRIDE`、`PNX_ROBOT_CONFIG_OVERRIDE` 用于显式验证，常规配置仍修改本目录；不同板型须使用不同构建目录。旧缓存可能保存旧工具链路径，迁移后使用新的构建目录，勿沿用旧选择器。

最低版本升级源于 [CMake 3.23 的 preset include 支持](https://cmake.org/cmake/help/v3.23/manual/cmake-presets.7.html)。配置生成、固件链接、启动钩子和硬件运行是不同验收层级，当前限制见 [SPEC](../.agents/docs/SPEC.md)。

## 维护脚本

- `cmake/load_board.cmake`：发现并校验配置身份，集中解析本目录路径；不读取 boards 产物。
- `cmake/configure.cmake`：独立预览入口及 CubeMX 输出脚本。
- `cmake/import_ioc.cmake`：只解析硬件事实。
- `cmake/generate_config.cmake`：合并参数、校验资源/消费者、生成绑定。
- `cmake/export_editor_context.cmake`：导出同一配置源的文件与可用资源。
- `cmake/usbx_integration.cmake`：将产品 USBX 选择接入生成目标。
