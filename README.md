# OLED.2
校赛小车，36kb内存，22秒一圈
电磁循迹小车固件（STM32F103C8T6 核心板）
按 Netlist_Schematic1_2026-11-09.tel 的网表把 OLED.ioc 的外设、引脚配好，并写好全部应用代码： 电磁方向环 + 编码器速度环串级控制、OLED 显示车轮转速、旋转编码器/蓝牙两种手动模式、按航向积分统计圈数。

1. 快速开始
用 STM32CubeIDE 打开本工程 → 编译 → 下载。
上电时把车放在赛道中心线上（上电约 0.8 s 后自动做一次电磁零点校准，车必须静止）。
手机/电脑连蓝牙模块，发 a → 进入自动循迹，1 s 后起步（给时间把车放好）。
发 p 打印一行数据，发 o 打开连续数据流（调参时用）。
想手动开：转一下旋钮（自动接管），或发 f / b / + / -。
2. 引脚对照（来自网表）
U1/U2 是核心板的两个 20P 排针，所以网表里的节点号就是核心板引脚号。

网表	核心板	MCU 引脚	本工程功能
PWMA	U1.5	PA8	TIM1_CH1，电机 A 调速（H4.7）
PWMB	U1.6	PA9	TIM1_CH2，电机 B 调速（H4.1）
AIN1	U1.1	PB12	电机 A 方向（H4.5）
AIN2	U1.2	PB13	电机 A 方向（H4.6）
BIN1	U1.3	PB14	电机 B 方向（H4.3）
BIN2	U1.4	PB15	电机 B 方向（H4.2）
E1A / E1B	U2.10 / U2.9	PA6 / PA7	TIM3 编码器接口（4 倍频）
E2A / E2B	U1.14 / U1.15	PB6 / PB7	TIM4 编码器接口（4 倍频）
SCL / SDA	U2.6 / U2.5	PB10 / PB11	I2C2 400 kHz，接 U5 OLED
TXD / RXD	U2.14 / U2.13	PA2 / PA3	USART2 9600 8N1，接 U8 蓝牙
O1	U2.16	PA0	ADC1_IN0，运放输出（U4.6）
O2	U2.15	PA1	ADC1_IN1，运放输出（U4.3）
O3	U2.12	PA4	ADC1_IN4，运放输出（U4.2）
O4	U2.11	PA5	ADC1_IN5，运放输出（U4.1）
A / B	U1.12 / U1.13	PB4 / PB5	旋转编码器，EXTI4 / EXTI9_5，板上已有 4.7k 上拉
U3（电磁感应线圈）+ U11（运放输入）→ U4（运放输出 O1..O4）→ MCU ADC；U5 = OLED；U6 = 编码器 3V3/GND；U8 = 蓝牙；U10 = 编码器 A/B/GND。

3. PWM 与占空比上限
TIM1：PSC = 0，ARR = 4199 → 72 MHz / 4200 = 17.14 kHz。
CCR 硬限幅 2400（PWM_MAX_CCR）→ 2400/4200 = 57.1 % ≈ 满电 12.6 V 时 7.2 V。 想严格按 7.4 V：把 PWM_MAX_CCR 改成 2466。
该上限在 Motor_SetSigned() 里强制生效，任何 PID 输出、前馈、手动油门都超不过。
4. 控制结构（100 Hz）
代码块


                ┌──────────── 自动模式 ────────────┐
 O1..O4 ─ADC─► 差比和误差 e(-1..+1) ─[方向PID]─► 转向速度差 steer (mm/s)
                                                        │
          目标车速 V_base ─────────────────────────────┤
                                                        ▼
                                   左目标 = V_base - steer
                                   右目标 = V_base + steer
                                                        │
 E1/E2 编码器 ─► 左右轮实际速度 (mm/s) ─► [左右速度PID] ─► CCR(±2400) ─► TB6612
方向环：e = (左通道和 - 右通道和) / (左通道和 + 右通道和)，与磁场强弱无关，只和车相对导线的偏置有关。e > 0 表示导线在车的左侧，于是左轮减速、右轮加速（差速转向，无舵机）。
速度环：每个轮子一个 PID，输入是编码器测得的 mm/s，输出是 PWM 计数，所以车会稳定在设定速度上。
手动模式：旋钮/蓝牙给出油门百分比 → 目标速度 → 同一个速度环；此时不转向（steer = 0）。
保护：自动模式下连续 0.5 s 没有电磁信号（EC_OnTrack() 为假）→ 刹车并显示 LOST。
5. 圈数统计
用差速运动学对航向积分：ω = (v右 - v左) / 轴距，累计 360° 记一圈（LAP）。OLED 第 1 行显示 L<圈数>，第 8 行 H 是当前累计航向角。

默认 LAP_STOP_AFTER = 0：只统计、不停车，方便前期调车。
想"跑完一圈自动停"：把 board.h 里 LAP_STOP_AFTER 改成 1U（LAP_TARGET_LAPS 是目标圈数）。
这是里程计估计，不是起跑线检测：编码器/打滑会让航向有累计误差，跑多圈时可能偏早或偏晚，可发 z 随时清零重算。
6. 蓝牙指令（9600 8N1，单字符即可，回车可有可无）
指令	作用
a	进入自动循迹（清零圈数与航向，1 s 后起步）
m	进入手动模式（用蓝牙油门）
f	前进，油门 = 当前档位
b	后退，油门 = 当前档位
s	停车（刹车）
+ / -	档位 ±10 %（前进/后退中会同步改速度）
0..9	档位 = 数字×10 %（保持当前方向）
r	重新校准电磁零点（车要静止放在赛道上）
p	打印一行数据
o	打开/关闭连续数据流（100 ms 一行）
z	圈数、航向清零
h	帮助
数据格式：

代码块


M=AUT VL=452 VR=449 TL=450 TR=451 E=12 D=1180/1175 S=320,120,0,0 LAP=0 HD=87
VL/VR 左右轮速(mm/s)，TL/TR 目标速度，E 误差×100，D 左右 PWM 计数，S 四路电磁通道相对值，HD 航向角(度)。

7. 旋转编码器（U10）
正转 → 油门增大，车前进；反转 → 油门变负，车后退；转过的角度越大速度越高（每格 5 %，KNOB_STEP_PCT）。
只要转动旋钮就会从自动模式抢回手动控制（防误触：接管后油门从 0 开始）。
死区 ±3 %（MAN_DEADBAND_PCT），避免旋钮回中不干净时蠕动。
8. 必须按你的车核对的几个参数（Core/Inc/board.h）
宏	默认	不对会怎样	怎么确认
ENC_COUNTS_PER_REV / ENC_CPR_LEFT / ENC_CPR_RIGHT	1014 / 1014 / 1015（实车标定）	速度显示和速度环的 mm/s 量纲都错，PID 增益要跟着重调	发 q → 车直线推 1 m → 再发 q，把打印的 CPRL/CPRR 填进来；两轮差值应 < 1 %
WHEEL_DIAM_MM	47.0	直径不用很准：CPR 标定会吸收轮胎压缩误差	量一下轮胎外径即可
WHEEL_BASE_MM	127.0	只影响圈数/航向统计准度	左右轮中心距
ENC_LEFT_SIGN / ENC_RIGHT_SIGN	-1 / +1（随 BOARD_LR_SWAP 一起换）	速度环变正反馈：一推就自己加速、s 停不下来	前进时 OLED L/R 必须都为正；现场可用蓝牙 y（翻左）/u（翻右）
MOTOR_A_INVERT / MOTOR_B_INVERT	1 / 1（实车标定）	同上正反馈，表现为"发 f 车后退"	手动给 20 % 油门看轮子转向；固件有反馈反向保护，会刹车并报 FAULT: speed feedback reversed
BOARD_LR_SWAP	1（实车标定：电机与编码器一起交叉）	方向环越修越偏，循迹一出线就往固定一侧跑掉	抬车只转左轮，OLED 的 L 必须变；若 R 变就改这个
EC_W_L1/W_R1/W_L2/W_R2	O1/O2 为一对，O3/O4 关闭	只用两路也能跑，四路更稳	看 OLED 第 7 行 S 四个值：哪个通道在导线附近变大就是哪一路；把 O3/O4 按实际接的左右填权重
OLED_I2C_ADDR	0x78	屏幕不亮	有的模块是 0x7A（SA0 拉高）
BT_BAUDRATE	9600	蓝牙乱码	HC-05/06、AT-09 出厂多为 9600
网表里 O1..O4 只标了序号，没有标明左右/前后，所以代码默认只用 O1、O2 当左右一对（差比和），另外两路权重设 0。先看 OLED 第 7 行的 S 四个数，把车头左右摆动，明显变大的是左右水平线圈；再把对应权重填进 EC_W_*（左加右减）。

9. PID 调参顺序
先把方向环关掉：DIR_KP/DIR_KI/DIR_KD 全设 0，蓝牙 f 或旋钮让车直行，确认速度环正常。 速度环：SPEED_KP 从 4 开始，SPEED_KI 从 12 开始（单位见下），看 OLED T 与 L/R 是否跟得上；抖动就把 SPEED_KP 降或把 SPEED_LPF 调小（0.2 左右更平滑）。
速度环单位：误差 mm/s → 输出 PWM 计数。SPEED_KP=4 表示每差 10 mm/s 给 40 计数（≈1 % 占空比）。
再调方向环：DIR_KP=300 起步（误差 0.1 → 30 mm/s 速度差），不够跟弯就加到 400~600；DIR_KI 从 80 起，用来消除稳态偏置（一直偏向一边）；DIR_KD 一般 0。
整体车速：AUTO_SPEED_MMS（默认 450 mm/s）逐步加，跑到刚好能过弯不掉线为止。
发 o 打开数据流，用串口工具记录 E/HD 曲线调参最直观。
10. OLED 界面
代码块


AUT L0 TRK        ← 模式(AUT/KNO/BTH) + 圈数 + 电磁状态(TRK/LOST/---)
 45 CM/S          ← 车速（2 倍字号，cm/s）
T 450 E  12       ← 目标速度 / 误差×100
L 452 R 449       ← 左右轮实测速度 mm/s
D 1180/1175       ← 左右 PWM 计数（4200 = 100 %）
S 320 120   0   0 ← 四路电磁通道相对零点值
K  35 B   0 H  87 ← 旋钮油门% / 蓝牙油门% / 航向角
11. 关于 .ioc
OLED.ioc 已按上面配置改好：ADC1（4 通道扫描 + 连续 + DMA1_Channel1 循环）、I2C2 400 kHz、USART2 9600、TIM1 PWM（ARR 4199）、TIM3/TIM4 编码器（TI12、滤波 6）、PB12~PB15 输出、PB4/PB5 双边沿 EXTI，NVIC 打开 DMA1_Channel1 / EXTI4 / EXTI9_5 / USART2。

注意：这次没有用 CubeMX 重新生成代码（本机 CubeMX 无界面模式被系统注册表权限挡住，无法完成首次运行的许可确认）， 所以 Core/Src 下的初始化代码是按 CubeMX 生成风格手写的，和 ioc 内容一一对应。 在 CubeIDE 里点 "Generate Code" 也是安全的：所有用户代码都在 USER CODE 段内或独立文件里，重新生成不会丢失。 重新生成后如果 MX_ADC1_Init() 之类被改动，请对照 Core/Src/adc.c（ADC/DMA）、i2c.c、usart.c、tim.c、gpio.c。

网表里的一处小差异
你说 H5 接"电机驱动模块 output"，但网表里 H5 的 7 脚是 E2B / E2A / E1B / E1A / ADC / GND / +5V，也就是两路编码器反馈 + 电源，没有电机功率线。 代码按"H5 提供两路编码器 A/B"来用（TIM3/TIM4），这与你"编码器测速"的要求一致；H5.5 的 ADC 网络（接 PB0）本工程没有使用。

12. 文件清单（新增部分）
代码块


Core/Inc/board.h        所有引脚映射、机械参数、PID 参数、限幅（调参只看这一个文件）
Core/Inc/pid.h          位置式 PID（输出限幅 + 积分抗饱和 + 微分作用于测量值）
Core/Inc/motor.h        TB6612 驱动：方向引脚 + PWM，硬限幅 ±2400
Core/Inc/encoder.h      TIM3/TIM4 正交解码 → 轮速 mm/s
Core/Inc/ec_sensor.h    ADC+DMA 电磁采集、零点校准、差比和误差
Core/Inc/knob.h         PB4/PB5 旋钮正交解码 → 油门百分比
Core/Inc/bt.h           USART2 蓝牙指令解析（收发都是中断，不阻塞）
Core/Inc/oled.h         SSD1306 128×64 显存 + 5×7 字库 + 异步刷屏
Core/Inc/control.h      模式机、串级 PID、圈数统计、HUD
Core/Src/adc.c i2c.c usart.c gpio.c tim.c    CubeMX 风格外设初始化
Core/Src/motor.c encoder.c ec_sensor.c knob.c bt.c oled.c oled_font.c pid.c control.c
Core/Src/main.c         10 ms 控制 / 50 ms 刷屏调度
tools/oled_font.py      由 5×7 ASCII 点阵生成 oled_font.c 的脚本
编译占用：Flash ≈ 38 KB / 64 KB，RAM ≈ 4.2 KB / 20 KB（栈 0x800）。

13. 烧录后"什么都不动"怎么查
三个互不相干的系统（OLED、蓝牙、旋钮）同时全灭，基本只有两种可能：MCU 没在跑这份代码，或者板子上 5 V / 3.3 V 电源树没电（除 MCU 外所有模块都挂在这条链上：12 V 电池 → U12 降压 5 V → U9 LDO 3.3 V）。

新固件已经让板子能"自证"了，按下面顺序走，5 分钟内能定位。

第 1 步：确认烧进去的是这份代码
CubeIDE 里先 Clean 再 Build，构建日志里必须出现 control.c、oled.c、bt.c、ec_sensor.c（旧工程只有 main.c/gpio.c/tim.c，不会编译这些）。
生成的 Debug/OLED.elf 应该在 38 KB 左右；如果只有几 KB，说明新文件根本没编进去（右键工程 → Refresh，或检查 Debug/Core/Src/subdir.mk）。
下载时看控制台是否 Download verified successfully，下载完按一下板子上的复位键（或 Run → Resume）。
第 2 步：BOOT0 必须在 0
核心板上的 BOOT0 跳线（或 B0 引脚）如果 = 1，芯片一上电就进系统存储区，应用程序永远不会执行，但烧录照样成功 —— 这是"烧了没反应"最常见的原因。

第 3 步：接上电池，量电压
只插 ST-Link 时，主控板自己的 5 V/3.3 V 是没电的：OLED、蓝牙模块、编码器、电机驱动全部由电池经 U12/U9 供电。用万用表量：

测试点	正常值
U7（XT60）输入、U13 开关之后	≈ 电池电压（12.6 V）
U8.1 或 H5.7（+5V 网络）	5 V
U5.2（3V3 网络）	3.3 V
任何一级不对，先修电源，后面都不用看。

第 4 步：读上电自检报告（蓝牙 9600 8N1）
上电 1.5 s 后串口应收到：

代码块


==== OLED-CAR v2 ====
CLK=72000000Hz HSE-PLL ERR=0
OLED=0x78 ok=1 frames=3
I2C: 3C
ADC=2100,2050,1800,1900 BASE=2000,2000,2000,2000
ENC=0,0 CNT=0,0 KNOB=0 MODE=KNO
cmd h=help
按内容判断：

现象	结论
HSI-FALLBACK-HSE-DEAD	核心板晶振没起振（缺晶振/坏晶振），程序已自动降 64 MHz 继续跑；建议换板
ERR= 不是 0	某外设初始化失败，把数值告我
OLED=... ok=0 + I2C: NONE	I2C 上没有任何设备应答：屏没供电／线接反／断线
I2C: 3C 但屏幕全黑	屏可能是 128×32、SH1106 或复位脚没接
一个字都收不到	蓝牙模块没电或波特率不是 9600，或者 MCU 没跑这份代码 → 走第 5 步
第 5 步：用调试器一眼判定（最直接）
CubeIDE 里 Debug 启动 → 暂停（Suspend），看停在哪：

停在 main() 的 while (1) 里 → MCU 运行正常，问题在电源 / 接线 / 模块。
停在 HardFault_Handler 或 Default_Handler → 是真的崩了，把调用栈（Debug 窗口）发我。
连不上、或一直复位 → 供电 / BOOT0 / SWD 线问题。
第 6 步：单个功能各自的检查点
蓝牙没反应：默认 9600；命令单字符即可，\r\n 可选。网表里 U8.3 标 RXD、U8.4 标 TXD，如果模块丝印说的是"模块自己的 TX/RX"，就要交叉接（模块 TXD → PA3，模块 RXD → PA2）。可先用 USB-TTL 直连 PA2/PA3 收自检报告。
旋钮没反应：U10 的 A/B 需要 U6 供 3.3 V；A/B 上拉是板上 R8/R9。转旋钮时 OLED 最后一行 K 应变化，蓝牙发 i 也能看到 KNOB=。
OLED 不亮：本固件会自动试 0x78 和 0x7A 两个地址；都试不到就跳过刷屏（不会卡住控制环）。
第 7 步：H4.4 接的是 STBY，不是 GND（PCB 设计错误，已确认）
网表里 H4.4 属于 GND 网络，但驱动模块这位是 STBY（桥使能），必须为高电平。 STBY = 0 V 时 TB6612 的 A/B 输出是高阻，现象是：

方向脚电平正确（实测量到 AIN1 = 3.28 V）
PWM 波形正常（TIM1 CR1=0001 CCER=0011 BDTR=A000，MOE=1）
轮子一动不动，x 的每个相位都不动，V=0
软件无法解决（板上没有 GPIO 连到 STBY），必须改硬件：

方案	做法	说明
A（最快）	把 H4 排线里第 4 根线从插头退出／剪断，再用跳线把模块 STBY 接到 +5 V（H5.7 或 U8.1）	改完 STBY 对地应约 4.85 V
B（可软件控制）	模块 STBY 接到 H2.1（网络 B01 = PB1），并把 board.h 的 DRIVER_STBY_GPIO 改为 1U	固件 Motor_Init() 会把 PB1 拉高，以后还能软件待机
⚠️ 绝不能在 H4.4 还与地相连时直接给它加 +5 V——那等于把 5 V 短接到地，会打崩 5 V 电源。 一定要先断开第 4 根线，再给 STBY 加高电平。

下一版 PCB 建议：H4.4 改为 +5 V（或引到空闲 GPIO），并在 STBY 上加 10k 上拉到 5 V，避免悬空时不使能。

这次顺手加的健壮性改动
改动	作用
HSE 起振失败 → 自动切 HSI/PLL（64 MHz）	晶振坏的板子不再变"砖"，自检能报告
Error_Handler() 不再 __disable_irq(); while(1)	外设初始化失败不再让整块板子假死，只记 ERR 计数
OLED 地址自动识别 0x78 / 0x7A	排线是另一种模块时也能亮
蓝牙发送改成环形队列 + 中断	90 字符一行要 94 ms @9600，不再阻塞 100 Hz 控制环
屏幕不应答时 10 Hz 重试	屏坏/没插时不会拖慢控制环
栈 0x400 → 0x800	snprintf + 嵌套调用更安全
上电 1.5 s 自检报告	一眼看出 MCU/时钟/I2C/ADC/编码器状态
14. 实车标定结果（已验证：稳定循迹，JIT = 0）
这一节是"这台车调通时"的实际取值，换车/换线后按同样顺序重标即可。

14.1 最终标定值（Core/Inc/board.h）
宏	取值	怎么得到的
BOARD_LR_SWAP	1	抬车只转左轮，OLED 的 R 变化 → 电机与编码器整体左右交叉
MOTOR_A_INVERT / MOTOR_B_INVERT	1 / 1	给正指令车却后退 → 极性反，正反馈
ENC_LEFT_SIGN / ENC_RIGHT_SIGN	-1 / +1	随 BOARD_LR_SWAP 一起换（符号跟着定时器通道走）
ENC_COUNTS_PER_REV / CPR_L / CPR_R	1014 / 1014 / 1015	蓝牙 q：直线推 1 m，两轮计数 6869 / 6872
WHEEL_DIAM_MM / WHEEL_BASE_MM	47.0 / 127.0	卷尺实测
PWM_MAX_CCR	2730（65% → 电机 7.41 V）	电机额定 7.4 V、电池 3S、驱动压降约 1.2 V
AUTO_SPEED_MMS	800	+ 逐步加，找到 S 弯还能过的最高速
DIR_KP / DIR_KI	400 / 80（摇摆时先降到 260 / 60）	实跑调
EC_USE_BASELINE	0	运放模块输出的是直流包络，电平本身就是测量值
EC_W_L1 / W_R1 / W_L2 / W_R2	1 / 0 / 0 / 1	只有 O1、O4（垂直于赛道的两路线圈）有用；O2/O3 平行于赛道几乎无信号
编译优化	-O2	.cproject 已改；Flash 51.9 → 37.2 KB，运行快 3~5 倍
14.2 排查过程中确认的坑（按踩到的顺序）
现象	真正原因	解决
烧录后全无反应	Debug 会停在 main() 第一行（Temporary breakpoint）	按 F8 或改用 Run
同上	CubeMX 无界面模式被注册表权限挡住	代码按 CubeMX 风格手写，USER CODE 段可安全重新生成
电机不动但方向脚/占空比都对	H4.4 在网表里接 GND，而驱动模块这位是 STBY，桥被禁用	STBY 跳 5 V（或跳 PB1 + DRIVER_STBY_GPIO 1）；改板时 H4.4 应接 +5 V
发 f 车后退、s 停不下来、一推就自己加速	电机指令极性与编码器符号相反 → 速度环正反馈	MOTOR_x_INVERT = 1，并加反馈反向看门狗
循迹只往一侧跑偏	电机/编码器左右交叉 → 方向环越修越偏	BOARD_LR_SWAP = 1
OLED 花屏、frames=0	用了 HAL_I2C_Master_Transmit_IT 却没开 I2C2_EV/ER_IRQn，传输卡死	补中断服务函数 + NVIC；改用页寻址模式兼容 SH1106
E 恒为 0、不循迹	运放输出是直流电平，却被当成交流信号减了基线（增量只剩 7 counts）	EC_USE_BASELINE 0，只在正中标定误差偏置 EOFF
跑着跑着左右摇摆、出赛道	刷屏一次画 5000+ 像素（-O0 下几 ms）把 100 Hz 拍顶成 15~20 ms（JIT 每圈 +571）	刷屏改成每拍只画一行 + 编译 -O2 → JIT = 0
速度上不去、S 弯跟不上	57% 占空比只给电机 6.5 V（额定 7.4 V 却欠压跑）	PWM_MAX_CCR 2730 = 7.41 V
14.3 下一步（还没做）
找最高可用速度：跑起来按 +（每次 50 mm/s），到 S 弯开始跟不上再退 50，把该值写回 AUTO_SPEED_MMS
跑完一圈自动停：LAP_STOP_AFTER 改 1U（靠航向积分，满 360° 刹车）
验证圈数可信度：跑一圈看 HD 是否约 ±360、LAP 是否 +1
