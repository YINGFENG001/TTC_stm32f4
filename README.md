# TTC_stm32f4

STM32F407 末端设备控制工程，当前包含双路步进电机、1 路串口舵机夹爪和 1 路 EVS08 电动真空吸盘。

## 硬件接口

### 调试串口

- `USART1`
- `PB7`：`TX`
- `PB6`：`RX`
- 用于串口命令输入和状态打印

### 步进电机

- `mtor1`
  - `ENA`：`PE0`
  - `DIR`：`PE1`
  - `PUL`：`PI5` / `TIM8_CH1`
- `mtor2`
  - `ENA`：`PE4`
  - `DIR`：`PI8`
  - `PUL`：`PI6` / `TIM8_CH2`

### 夹爪串口

- `UART5`
- `PC12`：STM32 `TX`
- `PD2`：STM32 `RX`
- `GND`：与夹爪驱动板共地

当前实测可用接线：

- `PC12/UART5_TX -> 驱动板 TXD`
- `PD2/UART5_RX -> 驱动板 RXD`
- `GND <-> GND`

### EVS08 真空吸盘 RS485

- `UART4`
- `PC10`：STM32 `TX`
- `PC11`：STM32 `RX`
- `PH9`：`RE/DE` 方向控制
- 串口参数：`115200 8N1`
- 默认 Modbus 地址：`9`

方向控制：

- `PH9 = RESET`：发送模式
- `PH9 = SET`：接收模式

485 线序说明：

- STM32 的 `TX/RX` 是 UART 逻辑信号，不直接对应 RS485 的 `A/B`
- 板载 485 收发器负责把 UART 转成差分总线
- 接线时使用开发板标出的 `485 A / B`
- 通常 `A` 对设备 `A / D+`，`B` 对设备 `B / D-`

## 软件命令

### 步进电机

```text
mtor1/2 move [rev]
mtor1/2 stop
mtor1/2 rpm [value]
mtor1/2 set [accel] [decel] [gear] [micro]
mtor1/2 status
```

单位：

- `rev`：`0.1圈`，普通非 0 值表示定长运动，`+0` / `-0` 表示持续旋转直到 `stop`
- `accel`：`rpm/s`，有效范围由当前 `gear` 和电机轴 `40~1000rpm/s` 动态决定
- `decel`：`rpm/s`，有效范围由当前 `gear` 和电机轴 `40~1000rpm/s` 动态决定
- `rpm`：输出轴 `rpm`，上限由当前 `gear` 和电机轴最高 `1500rpm` 动态决定
- `gear`：减速比，格式如 `20:1`
- `micro`：驱动器细分，当前默认 `4`
- 默认值：`1:1` 默认 `rpm=200`、`accel/decel=400`；其他减速比默认值按电机轴等效限制折算，例如 `20:1` 默认 `rpm=30`、`accel/decel=30`

当前方向约定：

- `move 10`：顺时针定长运动
- `move -10`：逆时针定长运动
- `move +0`：顺时针持续旋转，直到 `stop`
- `move -0`：逆时针持续旋转，直到 `stop`
- `move` 和连续运行中的 `rpm` 回显会打印 `dir=CW/CCW`；正数为 `CW`，负数为 `CCW`
- `move +0/-0` 连续运行期间允许执行 `rpm [value]` 动态调速，调速斜坡使用当前 `set` 保存的 `accel/decel`
- 连续运行期间 `rpm 0` 表示按 `decel` 减速到停止，`rpm N` 表示目标方向为 `CW`，`rpm -N` 表示按 `decel` 减速后反向到 `CCW` 并按 `accel` 升到 `N rpm`
- 保存的 `rpm` 永远为正数；定长 `move [rev]` 的方向只由 `rev` 正负决定
- 定长 `move [rev]` 运行期间暂不支持动态调速，此时执行 `rpm [value]` 会返回 `busy`

当前机械参数：

- `mtor1`：`200步/圈`，`4细分`，`1:1直驱`
- `mtor2`：`200步/圈`，`4细分`，`20:1减速机`，默认输出轴 `rpm` 范围 `1~75`，默认 `30`

### 夹爪

```text
clamp ping [id]
clamp status [id]
clamp readreg [addr]
clamp open
clamp close
clamp move [openPos/Pct]
clamp grip [load] [openPos/Pct]
clamp release
clamp set [speed] [gripStep] [releaseDelta]
```

单位：

- `openPos/Pct`：支持绝对位置 `700~2048` 或实测物理开度百分比 `0.0%~100.0%`；`700` = `100%` 最大张开，实测开口约 `72mm`，`2048` = `0%` 闭合，例如 `clamp move 100%`、`clamp move 100.0%` 与 `clamp move 700` 等同，也支持 `clamp move 66.0%`
- `load`：`0.1%`，范围 `100~750`
- `speed`：`1~3000`，默认 `1000`
- `gripStep`：`5~100`，默认 `30`
- `releaseDelta`：`20~400`，默认 `100`
- `current`：`6.5mA`

当前夹爪标定：

- `open = 700`
- `close = 2048`
- `100%` 最大张开对应实测开口约 `72mm`，百分比表示相对该最大物理开口的开度。
- 百分比不是按 `pos` 线性比例计算，而是按实测物理开度标定表插值：
  `700=100.0%`、`800=88.9%`、`900=79.2%`、`1000=68.1%`、`1100=58.3%`、`1200=48.6%`、`1300=40.3%`、`1400=33.3%`、`1500=25.0%`、`1600=19.4%`、`1700=13.9%`、`1800=8.3%`、`1900=4.2%`、`2048=0.0%`。
- 初始化时会逐项校验舵机保护参数：地址 `34/0x22` 保护扭矩 `20%`，地址 `35/0x23` 保护时间 `200*10ms=2s`，地址 `36/0x24` 过载扭矩 `80%`；读出值不一致时才写入，减少 EEPROM 重复写入。

保护逻辑：

- `open` / `close` / `move` 到位后会自动取消扭矩使能，避免夹爪继续出力顶住机构。
- `open` / `close` / `move` 运行中如果负载达到 `70%` 并连续命中 3 次，会判定堵转并自动取消扭矩使能，防止损坏。
- `grip` 夹取成功后保持扭矩使能，用于维持夹持力。
- `status` 和夹爪动作返回中的位置统一显示为 `openPos/Pct = 700 (100.0%)`，其中百分比为实测物理开度插值结果。
- `grip [load] [openPos/Pct]` 中 `openPos/Pct` 为可选参数；不填时从当前位置开始精细闭合。
- `grip` 使用两阶段夹取：可选先运动到 `openPos/Pct`，再用 `10 step` 精细闭合到目标负载的 `60%`，随后进入 PI 保力。
- 设备快速检测周期固定为 `200ms`；PI 保力和吸盘掉落检测使用该周期。
- 4 个 LED 对应 `mtor1/mtor2/clamp/vacum` 健康状态：正常常亮，通信断联常灭，设备错误约 `1Hz` 闪烁；夹爪和吸盘每约 `5s` 做一次低频 `status` 查询。
- PI 保力非稳定调整日志最短 `2000ms` 打印一次，稳定保持日志最短 `20000ms` 打印一次；稳定判断为负载绝对误差 `20`。
- PI 保力中如果负载连续 5 次低于目标负载的 `40%`，且实际读取位置累计闭合推进超过 `200 position`，会判定物品掉落并关闭扭矩。

### EVS08 真空吸盘

```text
vacum set [min_vac] [max_vac] [timeout]
vacum grip
vacum release
vacum stop
vacum status
```

单位：

- `min_vac`：最小保持真空度，范围 `0~100%`
- `max_vac`：最大真空度，范围 `0~100%`
- `timeout`：抓取超时，范围 `1~255`，单位 `100ms`

示例：

```text
vacum set 30 70 10
vacum grip
vacum status
vacum release
```

其中 `vacum set 30 70 10` 表示最小真空度 `30%`、最大真空度 `70%`、抓取超时 `1000ms`。

当前实现中，EVS08 两个通道使用相同参数同步动作，不区分单独通道控制。

### 自动监控

- 吸盘掉落检测：`vac1` 和 `vac2` 两路真空度都连续 5 个快速检测周期低于等于 `5%` 时，上报 `@event level=fault dev=vacum event=drop` 并停止吸盘；单路降低但另一路仍能夹稳时不报掉落。
- 夹爪和吸盘后台状态读取第一次通信失败只打印 `@event level=warn ... action=retry`，连续第二次失败才上报 `@event level=fault` 并退出保持。
- 当前快速故障检测固定使用 `200ms`，不支持命令行修改。
- 已取消周期性 `@status` 自动打印，仅保留 `@event level=info|warn|fault` 后台异步事件上报。

## 主要特性

- `TIM8` 双通道独立控制两台步进电机
- 名称式串口命令分发
- 步进电机支持运行中途 `stop`
- 步进电机支持按电机单独方向反相配置
- 夹爪支持 `open / close / move / grip / release`
- 夹爪状态支持位置、速度、负载、电压、温度、电流读取
- EVS08 真空吸盘支持 `set / grip / release / stop / status`
- EVS08 状态支持通道状态、故障码、忙状态、物体检测、真空度、温度、总线电压读取

## 编译说明

开发环境：

- Keil MDK
- ARM Compiler 5

当前工程已在 `Misc Controls` 中加入：

```text
--no-multibyte-chars
```

用于避免中文字符串导致的多字节字符编译报错。

## 说明文档

- [Doc/末端外设控制说明.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/末端外设控制说明.md)
- [Doc/STM32内部功能与程序流程说明.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/STM32内部功能与程序流程说明.md)
- [Doc/串口命令说明.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/串口命令说明.md)
- [Doc/后续实现.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/后续实现.md)
