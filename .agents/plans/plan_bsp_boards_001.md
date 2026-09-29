# 双板 BSP 整合计划

状态：已完成并分步提交。用户授权提交现有修改，并整合历史 F407 BSP，按板型/外设组织。

## 基线与方案

1d150f7、29ddb42、0420dd7 分别保存配置/CubeMX、代码修正、规范文档。原 H723 Debug 和 8 项回归通过，F407 缺 BSP 无法构建。历史参考 4856e83 的 F407（源于 c147b1d），不恢复旧配置和回调模型。

bsp 下分 h723_dmmc02 与 f407_cboard，各自按外设细分；公共头接口一致、私有绑定限 BSP/生成源可见。配置仍在 configs，PNX_BOARD 自动选择板目录；显式 BSP 覆盖保留板族检查。bxCAN 适配当前接口，其他外设复用当前资源模型。退休原平铺路径及要求切换分支的说明。

## 验证结果

- PASS：两板 Debug/Release 四个 preset Configure 和链接，Arm GNU 13.3.1。H723 Debug FLASH 79268 B、DTCMRAM 52440 B、DMA 1312 B，与迁移前一致；F407 Debug FLASH 81104 B、RAM 62528 B。仅陈述链接统计，不推断实时性能。
- PASS：python -m unittest discover -s .agents/tests -v，9 项通过；补充扩展帧用例后 test_core.py 五项再次通过。F407 测试编译生产 bxCAN 源码，真实生成配置，HAL/RTOS 边界替换；覆盖标准/扩展过滤、发送长度/ID、邮箱忙、HAL 错误、初始化失败重试、接收回调/拒绝远程帧、IRQ 批次上限、未知句柄、注销及显式恢复。
- PASS：四套 compile_commands 的 BSP 源码/头路径隔离；两板公共头一致。临时根 Configure 验证错板族覆盖和 F407 启用 H7 诊断均拒绝。配置隔离测试仍通过。
- 已解决：F4 SPI cache 清理后移除了仍供长度校验使用的 limits 头，Release 首次失败；恢复必需头后 Debug/Release 均通过。中文 PowerShell 管道编码使文档替换未生效，已显式 UTF-8 写入并复核，不保留乱码草稿。
- 限制：F407 Release 原 USBX 中间件出现数组边界告警，未修改第三方。当前 F407 IOC 无 CAN SCE IRQ，错误中断完整硬件链未验证。两板 app_start 钩子、硬件收发、失败恢复时序均 NOT_RUN；未操作硬件或修改 IOC。
- 范围：F407 ADC/H7 FDCAN 诊断明确不支持，构建拒绝；旧 flash/indicator/diagnostics 没有当前有效消费者，不恢复。没有独立临时日志或源码副本，测试临时目录自动清理。

实现与测试提交：8205e26；本文件及最终设计、配置说明在随后文档提交中保存。git diff --check 和当前文档本地链接/UTF-8 检查均 PASS。
