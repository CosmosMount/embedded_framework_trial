# 外设资料索引

本文件维护资料与配置/实现的对应关系；准入规则见 [SPEC](SPEC.md#外设资料依据与准入)，测试入口见 [README](../../README.md#开发与验证)。资料支撑不等于实机验收。最近核对日期：2026-10-07。

状态：**已核对条目**仅指表内列出的内容；**部分**表示资料存在但适用版本、接线或实现尚未全部验证；**缺失**表示没有找到能支撑该能力的资料。当前不能宣称所有外设已经完成资料准入。

## 来源与获取

资料以厂商、文档名称、版本/发布日期及章节定位，不依赖个人电脑目录，也不记录文件哈希。下表中的 PDF 名称是资料标识，不是本地文件路径。PDF 页码从 1 开始，除特别注明均为文件页码。

公开来源优先使用表内直链。DJI/RoboMaster 资料可从 [官方规则/资料中心](https://bbs.robomaster.com/wiki/20204847) 按文档名称获取；达妙资料可从 [厂商资料仓库](https://github.com/dmBots/damiao-document) 或 [厂商官网](https://www.dmbot.cn/) 查找。入口可访问不代表其中所有历史版本均可下载；没有已确认公开地址的文档保留名称、版本和核对结论，需向厂商取得匹配版本，不以个人路径或其他型号资料替代。

| 标识 | 资料与适用范围 | 核对边界 |
| --- | --- | --- |
| MCU-H7 | ST [RM0468](https://www.st.com/resource/en/reference_manual/dm00603761.pdf)；[STM32H723xE/G DS13313 Rev 5](https://www.st.com/resource/en/datasheet/stm32h723vg.pdf)；[官方文档入口（含 ES0491 勘误、PM0253）](https://www.st.com/en/microcontrollers-microprocessors/stm32h723-733/documentation.html) | 对应 H723，不能套用 H743 内存和 DMA 拓扑。已定位官方来源；RM/勘误尚未逐条核对，DS 的 Memories、Pinouts、Electrical characteristics 是后续核对入口，不表示全部通过 |
| MCU-F4 | ST [RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)；[STM32F405/407 数据手册](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf) | GPIO/EXTI、RCC、DMA、TIM、SPI、USART、bxCAN、ADC、USB、I2C、CRC 章节；需选 F407 适用段。RM 发布版本、芯片修订对应勘误尚待固定；不能将网页可访问当作已读完参考手册 |
| BOARD-H7 | 达妙 CtrBoard-H7_V1.0-管脚标注图240127.pdf，第 2 页 | 已视觉核对 SPI2 PC0/PC3 片选、PE12 INT3、PB1 TIM3_CH4、PA7 SPI6 LED、PA9/PA10 USART1、PD2 UART5 及三个 CAN 接口。管脚图与当前绑定对应；仍需确认实物 DMMC02 板卡修订相同，图中没有完整加热电路/收发器型号 |
| BOARD-F4 | DJI [RoboMaster Development Board Type C User Manual](https://rm-static.djicdn.com/tem/35228/RoboMaster%20Development%20Board%20Type%20C%20User%20Manual.pdf)；[官方例程](https://github.com/RoboMaster/Development-Board-C-Examples) | 已找到官方入口，尚未固定该 PDF 版本并逐线核对实物修订、IMU/加热和 RGB LED 接线。不得用例程默认值代替器件额定限制 |
| BMI088 | Bosch [BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)，BST-BMI088-DS000-19，Rev 1.9 / 2024-01 | 已读寄存器 0x10（p.39）、INT3 配置/映射（p.41）、SPI 接口/时序表 12（p.44–48）；量程、电源与复位还须按第 3、5 章逐项验收 |
| DMIMU | 达妙 DM-IMU-L1 V1.2 / V1.3；[厂商资料目录](https://github.com/dmBots/DM-IMU)，[V1.3 来源目录](https://gitee.com/kit-miao/dm-imu/tree/master/%E8%AF%B4%E6%98%8E%E4%B9%A6) | 历史核对：V1.2 正文 p.10 位表优先于 p.18 错误示例；V1.3 / 2026-09-22 p.11 温度定义适用固件 2.0.3.0。见源码来源注释；本轮未重新取得 V1.3 文件，实物固件仍须匹配 |

HAL/API 行为还应核对当前仓内 `boards/<板型>/Drivers/STM32*xx_HAL_Driver/`，ThreadX/USBX 行为对应 `boards/<板型>/Middlewares/ST/`。这些源码可支撑软件接口，但不能取代电气资料。

## MCU 外设与板载资源

配置依据统一映射到 `configs/boards/<板型>/{<板型>.ioc,board.json,params.json,memory.ld}`；实际初始化在 `boards/<板型>/Core/Src/`。表内列出当前 IOC 启用实例，也登记生成层资源，不能只检查有 BSP 类的外设。

| 外设/资源 | 当前范围与实现 | 对应资料及必须核对的条目 | 状态 |
| --- | --- | --- | --- |
| RCC / SYS / NVIC / HAL 时基 | 两板时钟、IRQ；H7 TIM23 / F4 TIM14 HAL 时基；ThreadX SysTick/PendSV | MCU-H7/F4 的 RCC、TIM、异常优先级；板图晶振；生成 `main.c`、`stm32*hal_timebase_tim.c`、ThreadX 端口 | 部分；既有空闲 tick 实测仅适用于原记录状态，不代表全 IRQ 审核 |
| GPIO / EXTI | 两板片选、DRDY、板上 IO；`bsp/shared/{gpio,exti}` | MCU GPIO/EXTI 章节、BOARD-H7/F4、BMI088 p.41；输出极性、上拉及有效边沿 | 部分；F4 DRDY 边沿见下文 |
| SPI | H7 SPI2/BMI088、SPI6/LED；F4 SPI1/BMI088；`bsp/shared/spi` | MCU SPI 章节、BMI088 第 6 章、LED 精确型号时序；实际 kernel clock/分频、CPOL/CPHA、字长/位序 | BMI088 上限已核对；F4 超限；LED 资料不足 |
| UART/USART | H7 UART5/7、USART1/10；F4 USART1/2/3/6；`bsp/shared/usart` | MCU USART/DMA 章节，DR16、VT03、裁判协议；接线/反相、8E1/8N1、波特率、DMA 模式与回调 | 部分；协议参数已有依据，绑定不匹配见下文 |
| CAN / FDCAN | H7 FDCAN1/2/3；F4 CAN1/2；两板 `bsp/<板型>/can` | MCU CAN/FDCAN 章节、各电机/IMU 协议；仲裁/数据时序、实际帧格式、BRS、DLC、过滤器、消息 RAM、收发器 | 部分；H7 默认电机 FD 配置不一致，TDC 13/13 缺板级推导/测量来源；F4 SCE IRQ 未启用是既有未决项 |
| DMA / cache / 内存 | 两板 DMA；H7 DCache、专用 DMA 段；`bsp/shared/bsp` | MCU 总线矩阵、DMA、内存映射及 Cortex-M cache；DMA1/2/BDMA 分别可达区域和所有权 | 部分；现有地址/主机测试及历史 USART1 DMA 实测不覆盖每类 DMA 外设 |
| TIM / PWM | H7 TIM3/6/12；F4 TIM1/2/4/5/7/8/10，另 HAL 时基；`bsp/shared/pwm` | MCU 对应定时器章节；BOARD 图与负载电路；PSC/ARR/CCR、计数器位宽、通道、有效电平、共用周期 | 部分；加热功率参数缺板级实验依据，不能以 PWM 可运行证明温控安全 |
| ADC | H7 ADC1；`bsp/h723_dmmc02/adc`；F4 BSP 未支持 | MCU-H7 ADC 章节与电气采样条件；外接测量/分压电路 | 部分；PA5 图示可定位，模拟输入源阻抗、比例、校准和 DMA 实测待补 |
| USB | H7 USB_OTG_HS（内置 FS PHY）；F4 USB_OTG_FS；`bsp/shared/usb`、USBX | MCU USB 章节、BOARD USB 接线、仓内 USBX CDC/控制器 API；USB CDC 类规范尚未固定版本 | 部分；当前 H7 默认不启用 USBX、F4 启用，未进行本轮枚举/收发验收 |
| DWT | 两板 `bsp/shared/dwt` | Cortex-M7 PM0253 / Cortex-M4 PM0214 的 DWT/CYCCNT、调试与计数条件；CMSIS 当前头文件 | 部分；尚需固定并核对编程手册版本及计数回绕/调试暂停行为 |
| I2C | F4 I2C3，生成层初始化，未见自有 I2C BSP/设备消费者 | MCU-F4 I2C 章节；实际外接器件及板级上拉/负载 | 部分；未接入产品能力，不能从初始化存在推导设备支持 |
| CRC | 两板硬件 CRC 生成初始化；协议当前使用 `libs/crc` 软件 CRC | MCU CRC 章节；DR16/VT03/裁判各自协议及官方示例 | 部分；硬件 CRC 默认配置不能证明等同软件 CRC，保留既有有效生成资源 |
| 板上其他资源 | H7 图中的 QSPI Flash、LCD、RS485、摄像头、蜂鸣器等 | BOARD-H7 可定位管脚；需各器件型号/原理图/协议 | 图示不等于已配置或已实现；未启用设备不新增占位驱动，启用前补资料 |

## 外接设备与协议

| 设备（完整来源） | 资料定位与已核对内容 | 配置 / 实现 | 状态与缺口 |
| --- | --- | --- | --- |
| BMI088 | BMI088、BOARD-H7/F4；SPI dummy byte、INT3、量程/温度、SPI 时钟 | 两板 `board.json`、`defaults.json` bmi088/ahrs；`devices/imu/bmi088`、`modules/ahrs` | 部分；硬编码预偏置缺标定来源；温控 duty/PID 不能由芯片手册证明 |
| DM-IMU-L1 | DMIMU 条目 | robot/params 的 dmimu；`devices/imu/dmimu`、`modules/ahrs` | 历史协议核对，当前默认禁用；实际 firmware/mode/output 设置待匹配 |
| M2006 + C610 | DJI RM M2006 P36直流无刷减速电机使用说明.pdf V1.0 / 2019.03；C610 无刷电机调速器使用说明.pdf p.7–9 CAN | robot 的 dji_m2006；`devices/motors/dji` | 已核对 C610 命令 ±10000 ↔ ±10 A，模型减速比/反馈轴端需继续核验；C610 文件版本待确认，可从 [官方手册](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf) 核对 |
| M3508 + C620 | DJI RoboMaster M3508直流无刷减速电机使用说明V1.0（中英日）.pdf V1.0 / 2017.08；RoboMaster  C620 无刷电机调速器使用说明.pdf p.9–11 CAN | robot 的 dji_m3508；`devices/motors/dji` | 已核对 C620 命令 ±16384 ↔ ±20 A；Nm 换算仍缺标定，不能由额定点猜测 |
| GM6020 | DJI RoboMaster GM6020直流无刷电机使用说明20231013.pdf p.3、7–9，电流环适用 firmware >=1.0.11.2 / Assistant >=2.7 | robot 的 dji_gm6020；`devices/motors/dji` | 已核对 1 Mbps、标准 DATA/DLC8、电压/电流命令区别；实际电流环选项待匹配，H7 FD 默认错误见下文 |
| XROLL | 未找到完整厂商型号及对应电调/协议手册 | robot 的 dji_xroll；`devices/motors/dji/src/djimotors.cpp` | **缺失**；不能用 M3508 手册证明其减速比和反馈换算 |
| DM4310 | 达妙 DM-J4310-2EC V1.2 减速电机使用说明书 V1.4 2026-09-14.pdf p.6、10–12；另存 V1.1 硬件对应 V1.3 手册，不能混用 | robot 的 dm_dm4310；defaults.motors；`devices/motors/dm` | 部分；已核对标准 CAN/MIT 布局和可配置 P_MAX/V_MAX/T_MAX；当前默认 ±12.56637061 / ±15 / ±10 缺设备参数读取记录，不能径自替换成其他示例数值 |
| DM8009P | 达妙 DM-J8009P-2EC减速电机说明书V1.0.pdf p.4–8，控制模式/通信 | robot 的 dm_dm8009p；defaults.motors；`devices/motors/dm` | 部分；手册已找到，实物参数、固件及三个量程须匹配 |
| LK8016 / LK9025 | 瓴控 LK-TECH电机CAN协议说明V2.3.pdf p.13、19；瓴控电机CAN协议说明 V2.36 (2).pdf p.8–9；MG/MF 用户手册及产品图（该版本公开下载地址待补，向瓴控取得） | robot 的 lk_lk8016/lk_lk9025、protocol_options；`devices/motors/lk` | 已核对 V2.3 命令/反馈比例不同及 V2.36 MG/MF 比例不同；8016 是否 MG、9025 是否 MF、减速器版本及编码器位数须依据铭牌匹配，不能仅由数字后缀确定 |
| XV2 X42 / Y42 | 尚未取得精确产品/固件协议 | robot 的 xv2_x42/xv2_y42；`devices/motors/xv2` | **缺失**；帧尾、CAN 分片、单位比例、归零/标定命令必须补原始资料，现有回归只证明软件行为 |
| DR16 | DJI RoboMaster 机器人专用遥控器（接收机）用户手册.pdf p.5–8，18 B/14 ms、100 kbit/s、8E1、DBUS 反相 | params remoter/bindings；`devices/remotes/src/dr16.cpp`、gamepad | 已核对 UART 格式；两板 UART 采用 9 位含偶校验表示 8 数据位，接线须含反相环节；当前 source=none |
| VT03 / VT13 | RoboMaster RoboMaster裁判系统相机图传模块VT03&VT13使用说明书-CN&EN.pdf p.7–10，921600/8N1、帧字段 | params remoter/bindings；`devices/remotes/src/vt03.cpp` | 部分；F4 示例端口是 115200。CRC 冲突沿用既有用户裁定：采用 [DJI 官方示例](https://rm-static.djicdn.com/tem/17348/Example_Code_for_Data_Frame_and_Validation.c) 的反射 0x8408 路径，该例外仅限 VT03 CRC；不重新采用手册 CCITT-FALSE 叙述 |
| PS2 GPIO/SPI | 未找到实际手柄/接收模块型号及可核验协议/电气时序 | params.ps2；`devices/remotes/src/ps2.cpp` | **缺失**；LSB 顺序、模拟模式、ACK/字节间隔及 GPIO 时序需来源，不能用通用 SPI 手册替代 |
| WS2812 类 LED | BOARD-H7 仅标注“2812 LED”，未固定 WS2812/B/修订型号及 Worldsemi 手册 | robot.led、SPI6；`devices/led` | **部分**；源码 0x60/0x78、100 B 复位与 6 MHz SPI 可计算波形，但缺型号容差资料；当前禁用，不宣称时序通过 |
| F4 RGB GPIO LED | BOARD-F4 手册/原理图；PH10/11/12 绑定 | F4 board.json；`bsp/shared/gpio` | 部分；有效极性、电路待逐线核对，不套用 WS2812 协议 |
| 裁判系统 + UI | RoboMaster RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.pdf p.4 串口、p.25 起 0x0301 交互/UI；裁判系统使用/界面手册用于应用语义 | params.bindings.referee_uart；`modules/referee`、`devices/ui`、`libs/crc` | 部分；已核对常规链路 115200 与图传链路 921600 区别；尚未逐命令审计 2026 全部字段、长度和发送频率 |

## 当前问题与验收缺口

1. **P1，已确认：F407 BMI088 SPI 超规格。** IOC 的 SPI1 为 APB2 84 MHz / 8 = 10.5 MHz；生成 `boards/f407_cboard/Core/Src/spi.c` 同为 /8。BMI088 Rev 1.9 p.46 表 12 最大 10 MHz。生成器只检查绑定存在，没有拒绝超限。修正应在权威 IOC 选合法时钟/分频、重生成并核对两者一致，再验证采样；本次未改硬件参数。
2. **P1，已确认：H723 默认 GM6020 发成 FD/BRS。** `robot.json` 的 motor3=dji_gm6020 绑定 fdcan1/can_type=fd，IOC FDCAN1=FD_BRS；`bsp/h723_dmmc02/can/src/bsp_can.cpp::transmit` 按总线类型选择 FDCAN_FD_CAN，短至 8 B 也不会变成 Classic。GM6020 p.7–9 的标准 DATA 帧协议不支持把它当 FD/BRS 设备。motor2=DM4310 也在该总线上，已取得的手册只提供标准 CAN 协议，尚无 FD 支撑。需先明确实际设备/接线，再决定总线配置或逐帧能力契约；不能仅把 robot 的字符串改成 classic，生成器要求它与 IOC 一致。
3. **P1，已确认：H723 裁判常规链路波特率不匹配。** 默认 referee_uart=usart1，IOC/生成代码为 921600；`modules/referee/include/referee_protocol.hpp` 明确声明接电源管理模块 User 串口的常规链路，2026 V1.2.0 p.4 要求 115200。不能拿图传链路 921600 为当前常规链路背书。F4 USART6=115200 则满足该项；尚未核对真实接线。
4. **P2，已复现校验缺口：F407 VT03 示例绑定无法直接启用。** vt03_uart=usart6，但其 IOC/生成代码为 115200，VT03 要求 921600。临时 params 设 source=vt03、referee_uart=none 后仍成功生成，说明排除已知端口冲突后，生成器没有检查设备波特率。默认 source=none，因此不是当前已启动遥控故障；两板默认 app 也仅初始化日志。应在资料驱动的启用校验中发现该错误，而非等运行丢帧。
5. **P2，来源缺口：BMI088 板间共用标定常量。** `bmi088.hpp` 的 accel_preoffset/gyro_preoffset 没有传感器身份、测量条件或记录；校准失败会采用 gyro_preoffset，加速度始终加固定偏置。两板共享实现不能证明偏置可共享。需取得标定依据后决定配置/标定策略，不擅自归零或伪称现值错误。
6. **待实测：F407 DRDY 边沿与有效电平不同。** `configure()` 向 INT3_IO_CONF 写 0x01（高有效 push-pull，手册 p.41），F4 PC5 EXTI 配置下降沿，H7 PE12 配置上升沿。F4 捕获的是释放边沿而非有效边沿；是否造成漏采/延迟需器件中断清除语义与波形证明，不能仅凭此认定没有中断。

未列为故障：`gyro_odr_2000_bw_230 | gyro_lpm1_suspend` 跨寄存器借用常量含义不清，但写入的 0x80 对应 GYRO_BANDWIDTH 只读、无功能 bit 7（p.39），不能声称它使陀螺仪休眠。其可读性可单独整理，不能作为虚构根因。

本轮只作源码/配置/资料审查和既有软件验证；未完成所有寄存器、所有协议字段、所有板级电气参数的逐项验收。以上问题尚未修复，资料缺口尚未全部补齐；当前正式测试入口和验收边界见 SPEC；本次主机回归通过不能关闭上述硬件问题。
