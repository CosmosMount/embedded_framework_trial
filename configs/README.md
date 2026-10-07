# PnX 配置入口

项目配置以本目录为唯一来源。`boards/` 是固件构建消费的 CubeMX 生成工程；解析配置、预览生成结果和导出编辑器上下文不需要该目录存在。

修改或启用外设前，按 [SPEC 的外设资料准入要求](../.agents/docs/SPEC.md#外设资料依据与准入) 核对型号、板卡/固件版本及配置依据；逐项来源与当前缺口见 [外设资料索引](../.agents/docs/PERIPHERALS.md)。生成通过只说明当前检查已通过，不代表器件时钟、波特率、帧格式或接线符合手册。

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

`defaults.common` → `defaults.boards.<板型>` → `params.json`，后者优先，嵌套对象递归合并。`defaults.recommendations` 不参与这一合并。省略参数表示继承，不能把删除覆盖项理解为禁用功能。设备组合按 `robot.json` 配置，遥控来源和 USBX 等功能按对应 params 字段选择；这些选择决定编译与绑定，应用仍须调用初始化入口。

只写与默认值不同的字段。需要调整时，从 defaults 找到同名字段写入板型 params，避免复制完整默认表。

`*_ticks` 使用 ThreadX tick，不等于毫秒；`*_ms` 为毫秒；温控目标使用摄氏度，duty 字段为占空比。具体物理量、协议 raw 值及合法范围以生成器检查和消费端接口为准，不把所有数值统一当作 SI 原始输入。线程优先级数值越小优先级越高；调整时检查整条任务链。

`logging.capacity` 是保留 RAM 中的日志记录数，范围 1～255，每条固定占用 32 字节；另有少量状态字段。`logging.level` 可设为 `error`、`warning`、`info` 或 `debug`，低于指定严重程度的记录在写入前过滤。增加容量会直接占用对应板型 memory.ld 中的 `.noinit` RAM；日志不会写内部 Flash，也不提供断电保存。

## 无需板级代码的配置检查

需要 CMake 3.23 或更高版本。以下命令不启动 CubeMX、不编译、不操作硬件：

```powershell
cmake -DPNX_BOARD=h723_dmmc02 -DOUT_DIR=build/config-preview/h723 -P configs/cmake/configure.cmake
cmake -DPNX_BOARD=f407_cboard -DOUT_DIR=build/config-preview/f407 -P configs/cmake/configure.cmake
cmake -DPNX_BOARD=h723_dmmc02 -DPNX_EDITOR_OUTPUT=build/config-preview/editor.json -P configs/cmake/export_editor_context.cmake
```

每次预览生成 `config.hpp`、`robot_config.hpp`、`bsp_bindings.cpp`，另输出 `cubemx.script`。编辑器 JSON 的 IOC、board、params、robot 路径全部指向本目录。生成器是配置有效性的最终裁定者，编辑器不自行维护另一套规则。遇到错误修正源 JSON/IOC 后重新运行，不修补生成文件。

## 从 IOC 重新生成

IOC 权威源位于 `configs/boards/<板型>/`。预览命令输出的 `cubemx.script` 使用绝对 IOC 路径及显式 `project path`，将工程输出定向到仓库根的 `boards/<板型>/`。先查看该脚本，再按本机 CubeMX 的脚本模式执行；预览成功不代表 CubeMX 重生成已通过验证。命令语义见 [ST 官方 CLI 文档](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.0/en/docs/markup/CubeMX_CLI.html)。

使用 GUI 时同样显式设置生成目录，不能直接接受迁移 IOC 后的默认目录。CubeMX 若在输出目录生成 IOC 副本，它是派生产物；硬件修改须回到 configs 中的权威源。生成的板工程自带 CMake/preset/链接文件属于 CubeMX 产物，PnX 根构建不读取它们的产品配置，只接入 `cmake/stm32cubemx` 的源码目标。

`memory.ld` 管理产品链接布局；H723 额外定义现有 BSP 要求的、位于 RAM_D1 的对齐 DMA 段及起止符号。以后修改 IOC 内存/堆栈设置时，需要明确审查并同步本文件；不在 Configure 时静默覆盖产品链接策略。

## 构建固件

```powershell
cmake --preset h723-debug
cmake --build --preset h723-debug
```

要求 `boards/<板型>/` 下已生成的 CubeMX 工程、Arm GNU Toolchain 和 Ninja。实际产品工具链为 `cmake/gcc-arm-none-eabi.cmake`，由当前配置选择 CPU/FPU 和 memory.ld，不加载 boards 下的工具链。F407 使用 `f407-debug`，自动选择 `bsp/f407_cboard/`；H723 对应 `bsp/h723_dmmc02/`。`PNX_BSP_SOURCE_DIR` 默认留空，仅开发外部 BSP 时覆盖。Release 使用同板型的 release preset。

固件输出为 `build/<preset>/pnx_embedded.elf`。根工程的高级覆盖参数 `PNX_PARAMS_OVERRIDE`、`PNX_ROBOT_CONFIG_OVERRIDE` 用于显式验证，常规配置仍修改本目录；不同板型须使用不同构建目录。修改板型或工具链后使用新的构建目录，避免复用不匹配的缓存。

最低版本升级源于 [CMake 3.23 的 preset include 支持](https://cmake.org/cmake/help/v3.23/manual/cmake-presets.7.html)。配置生成、固件链接、启动钩子和硬件运行是不同验收层级，当前限制见 [SPEC](../.agents/docs/SPEC.md)。

## CAN FD 初始化

H7 CAN FD 的发送延迟补偿通过板型 params 中 `can.<端口>.tdc_offset`、`tdc_filter` 成对配置，均为 `[0, 127]` 的硬件参数；只有 IOC 启用 CAN FD BRS 的端口允许配置。两项存在时，BSP 在启动前配置并启用 TDC，失败返回错误；两项缺省时不启用。当前 FDCAN1 配置为 `13/13`，Classic 端口不配置 TDC。接收 FIFO 依据 IOC/CubeMX 生成的实际分配选择。

发送长度同时受协议和 CubeMX 配置的 `TxElmtSize` 限制：Classic 最多 8 字节；FD 的 DLC 补齐长度不得超过消息 RAM 元素容量。BSP 将 DLC 补齐部分置零，超出容量时拒绝提交，不自动扩大硬件配置。

## 独立采样与温控

在板型 params 中通过 `bmi088.temp_control_period_ticks` 设置温控周期，通过 `ahrs.imu_thread_priority`、`ahrs.temp_thread_priority` 分别设置优先级。采样由 DRDY 驱动，SPI 锁等待使用 `bmi088.spi_lock_wait_ticks`。默认值位于 defaults 的 common 节；覆盖时不要在 params 再嵌套 common。时间单位为 ThreadX tick，修改周期前核对板上 tick 频率。独立调度及保护边界见 [SPEC](../.agents/docs/SPEC.md#分层单位与调度)。

## 维护脚本

- `cmake/load_board.cmake`：发现并校验配置身份，集中解析本目录路径；不读取 boards 产物。
- `cmake/configure.cmake`：独立预览入口及 CubeMX 输出脚本。
- `cmake/import_ioc.cmake`：只解析硬件事实。
- `cmake/generate_config.cmake`：合并参数、校验资源/消费者、生成绑定。
- `cmake/export_editor_context.cmake`：导出同一配置源的文件与可用资源。
- `cmake/usbx_integration.cmake`：将产品 USBX 选择接入生成目标。

遥控/裁判模块直接使用 BSP UART，默认逻辑端口的物理映射由 `bindings.remoter_uart`、`bindings.vt03_uart`、`bindings.referee_uart` 定义。PS2 的轮询周期、SPI 超时、GPIO 时序和死区集中在 `ps2` 参数段。生成器拒绝未实现的电机 `control_mode: multi`。

## 遥控输入共存

在板型 `params.json` 设置 `"remoter": {"source": "vt03_dr16"}`，同时启用两路接收。`bindings.vt03_uart` 连接 VT03，`bindings.remoter_uart` 连接 DR16，两者须有 RX DMA 且使用不同串口。端口可分别更换，无需修改接收器 C++；引脚、波特率、校验位和 DMA 仍由同目录 IOC 配置并重新生成。当前两板示例中 VT03 与裁判指向同一端口，启用前必须按真实接线分配独立串口；不使用裁判时可显式设置 `bindings.referee_uart: "none"`。生成器拒绝已启用接收器之间的冲突。

VT03 有效时优先使用它的整套输入，失联后使用有效的 DR16；VT03 恢复后自动切回。`remoter.rx_timeout_ticks` 控制 DR16 空闲接收恢复；它与 `remoter.offline_timeout_ticks` 均要求整数 1～2147483647。后者是最后有效帧的最大年龄，超过才失效，单位为 ThreadX tick。两路都失效时输出离线并归零。频道重复发布不会刷新有效帧时间，切换不混合摇杆与键鼠；具体验收规则见 [SPEC](../.agents/docs/SPEC.md)。

单源 `dr16`、`vt03`、`ps2` 与 `none` 仍可选。PS2 保留独立轮询线程；DR16/VT03 双串口模式只使用一个遥控线程：回调复制最新帧及接收时间，线程每 tick 取帧、解码、选择并发布，不在中断中进行浮点解码或发布消息。帧邮箱采用 latest 语义，同一周期多帧可覆盖，不承诺保留所有短按事件。DR16 原有空闲接收重启由该线程执行，HAL 停启的异常耗时需纳入板上周期验证。同步 PS2 的模块入口统一为 gamepad；串口 PS2 已删除，不再提供串口接收模式。

## LED 与 UI

LED 参数位于 `params.led`，默认值见 `defaults.json`：

| 字段 | 单位与范围 |
| --- | --- |
| `spi_timeout_ms` | 每次 SPI 操作超时，毫秒 |
| `brightness` | 颜色通道值，0～255 |
| `blink_half_period_calls` | `blink()` 的半周期调用次数，1～32767；不是 RTOS tick |

LED 操作返回实际状态，失败不会推进 `toggle()`/`blink()` 的逻辑状态；GPIO 指示灯直接使用对应 BSP。

UI 的单次串口提交超时由 params.ui.uart_timeout_ms 配置（毫秒）。update() 每次最多提交一个字符帧或一组普通图元；发送失败保留待更新/删除状态，后续调用可重发。ok 仅表示本地串口接受，不代表裁判客户端已显示；UI 字符串借用调用方存储，必须保持有效。
