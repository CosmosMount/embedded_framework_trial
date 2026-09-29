# Configs 集中化计划 001

状态：已完成本次配置范围；F4 固件与应用启动存在下述独立限制。

## 范围、基线与收益

任务起点 HEAD=41aa5f3，已有 README/.gitignore/项目文档修改、961 项旧板目录删除及新生成板目录。根 Configure 原因旧板名与缺失板清单失败；配置生成器/编辑器寻找 boards 内 PnX 清单，参数覆盖存在重复。

用户要求配置集中于 configs、不依赖 boards，并提升可读性。本次将 IOC、板清单、固定绑定、产品链接布局、参数、设备组合、preset 和工具链集中管理；boards 保留生成工程。根 preset 仅 include 配置目录。硬件参数、业务值和设备选择保持不变；未改业务源码及 CubeMX 生成代码，未恢复旧板目录。

结构收益：根构建、工具链、预览和编辑器共用一个配置加载器；配置预览只需 configs 和 CMake。两板 params 从合计 100 行精简为 36 行，收益依据是有效输出相同而非行数本身。新配置指南说明所有者、覆盖规则、单位和真实入口。

## 已完成工作

- 统一 h723_dmmc02 / f407_cboard 标识，迁移 IOC 与 robot，保留硬件和业务值。
- 新增 configs 内板级事实与编译身份，板级事实来自旧清单并经新 IOC 的真实生成器校验；产品 memory.ld 基于用户重生成布局，H7 补全 BSP 要求的 DMA 契约。
- 移除旧 cmake/gcc-arm-none-eabi.cmake 与旧配置目录，根 preset 只做 include，不提供旧板名兼容层。
- 添加独立配置预览与 CubeMX 输出目录脚本；未执行 CubeMX 重生成。
- 统一配置身份/路径校验，根构建仅校验生成工程与 BSP，链接脚本修改会触发重新链接。
- 新增真实配置流水线隔离/拒绝回归，更新 configs/README、根 README、SPEC、DESIGN。

## 实际验证

工作目录 D:/Workspace/projects/penix；CMake 4.2.1、Arm GNU 13.3.1、Python 3.14。对应本次最终配置源码。

| 检查 | 结果 |
| --- | --- |
| 两板迁移前/后四个 C++ 生成文件逐字节比较 | PASS；默认覆盖精简未改变有效配置 |
| python -m unittest discover -s .agents/tests -v | PASS，4 项；复制 configs 到临时目录、没有 boards/BSP 时两板生成与编辑器导出成功；未知板型、错误身份、无效硬件绑定拒绝 |
| cmake --list-presets | PASS，保留四个用户 preset 名称 |
| cmake --fresh --preset h723-debug；cmake --build --preset h723-debug --parallel 4 | PASS，标准 build/h723-debug/pnx_embedded.elf 已链接；后续普通构建通过 |
| ELF DMA 符号与 board.json 对照 | PASS，0x24000000..0x24000520，1312 字节，起止均 32 字节对齐且位于配置允许的 RAM 范围 |
| cmake --preset f407-debug -B build/config-migration/f407-debug | 预期拒绝 H7 BSP（板族校验 PASS）；F4 完整固件 BLOCKED，未提供匹配 BSP |
| JSON 解析、文档链接/代码块、git diff --check、失效板名搜索 | PASS |
| CubeMX 实际重生成、F4 固件、其他设备组合、硬件时序/运行 | NOT_RUN / BLOCKED，不从配置与链接通过推断 |

## 已解决失败与剩余限制

首次 H7 链接失败缺 _sdma_buffer/_edma_buffer。核对 bsp_dma.cpp 与既有 DMA 合约后，在产品链接脚本定义 RAM_D1 专用段、KEEP 和对齐断言；链接及地址检查通过。未改用关闭 DMA 的绕行。

首次隔离测试中 Windows 临时目录重定向导致路径比较失败；两端统一 resolve 后通过，隔离断言未放宽。已有标准构建缓存仍引用旧工具链，普通 Configure 失败；用本机支持的 --fresh 刷新 CMake 缓存后通过，无旧选择器回退层。

F4 BSP 未提供；两板 App_ThreadX_Init 尚未调用 app_start，属于后续启动接入。CubeMX 脚本仅依据官方 CLI 生成，实际运行尚未验证。产品链接配置以用户新布局为基线，其他组合与启动初始化须单独验证。CMake 最低版本升为 3.23，以支持 preset include。

迁移对比与试构建临时目录 build/config-migration 在验收后清理，保留上述结论、正式测试和标准 H723 ELF；不新增备份、逐轮日志或额外报告。
