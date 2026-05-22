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
mtor1/2 accel [value]
mtor1/2 decel [value]
mtor1/2 rpm [value]
mtor1/2 status
```

单位：

- `rev`：`0.1圈`，普通非 0 值表示定长运动，`+0` / `-0` 表示持续旋转直到 `stop`
- `accel`：`rpm/s`
- `decel`：`rpm/s`
- `rpm`：`rpm`

当前方向约定：

- `move 10`：顺时针定长运动
- `move -10`：逆时针定长运动
- `move +0`：顺时针持续旋转，直到 `stop`
- `move -0`：逆时针持续旋转，直到 `stop`

### 夹爪

```text
clamp ping [id]
clamp status [id]
clamp readreg [addr]
clamp open
clamp close
clamp move [openPercentage: 0~100(%)]
clamp grip [load] [openPercentage]
clamp release
clamp set [speed] [gripStep] [releaseDelta]
```

单位：

- `openPercentage`：`0~100`，`0` 闭合，`100` 张开
- `load`：`0.1%`
- `speed`：`1~3000`，默认 `1000`
- `gripStep`：`5~100`，默认 `30`
- `releaseDelta`：`20~400`，默认 `100`
- `current`：`6.5mA`

当前夹爪标定：

- `open = 500`
- `close = 2048`

保护逻辑：

- `open` / `close` / `move` 到位后会自动取消扭矩使能，避免夹爪继续出力顶住机构。
- `open` / `close` / `move` 运行中如果负载达到 `70%` 并连续命中 3 次，会判定堵转并自动取消扭矩使能，防止损坏。
- `grip` 夹取成功后保持扭矩使能，用于维持夹持力。
- `grip [load] [openPercentage]` 中 `openPercentage` 为可选参数；不填时从当前位置开始精细闭合。
- `grip` 使用两阶段夹取：可选快速闭合到 `openPercentage`，再用 `10 step` 精细闭合到目标负载的 `60%`，随后进入 PI 保力。
- 设备快速检测周期固定为 `200ms`；PI 保力、吸盘掉落、电机无进展检测都使用该周期。
- PI 保力非稳定调整日志最短 `2000ms` 打印一次，稳定保持日志最短 `20000ms` 打印一次；稳定判断为绝对误差 `20`。
- PI 保力中如果负载连续 5 次低于目标负载的 `40%`，且为了恢复负载累计闭合推进超过 `80 position`，会判定物品掉落并关闭扭矩。

### EVS08 真空吸盘

```text
vacum set [min_vac] [max_vac] [timeout]
vacum grip
vacum release
vacum stop
vacum status
monitor [intervalMs]
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

### Monitor

```text
monitor
monitor [intervalMs]
```

- `monitor` 读取当前监控周期。
- 吸盘掉落检测：`vac1` 或 `vac2` 任一通道真空度连续 5 个快速检测周期低于等于 `5%` 时，上报 `@fault dev=vacum event=drop` 并停止吸盘。
- 电机无进展检测：电机运行中原始脉冲位置连续 5 个快速检测周期不变时，上报 `@fault dev=mtor* event=stall` 并停止电机。
- `monitor [intervalMs]` 保留为监控周期参数命令，当前快速故障检测固定使用 `200ms`，不跟随该参数。
- 已取消周期性 `@status` 自动打印，仅保留夹爪微调 `@info` 和异常停机 `@fault` 自动上报。

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

- [Doc/双电机步进控制修改说明.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/双电机步进控制修改说明.md)
- [Doc/双电机停止命令与方向修正方案.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/双电机停止命令与方向修正方案.md)
- [Doc/舵机夹爪控制修改说明.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/舵机夹爪控制修改说明.md)
- [Doc/EVS08电动吸盘RS485控制方案.md](D:/XGKJproject/stm32f4/DEVELOPING/Doc/EVS08电动吸盘RS485控制方案.md)
