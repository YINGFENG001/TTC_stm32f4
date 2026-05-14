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
mtor1 turn [rev]
mtor1 move [rev] [accel] [decel] [rpm]
mtor1 stop
mtor1 accel [value]
mtor1 decel [value]
mtor1 rpm [value]
mtor1 status

mtor2 turn [rev]
mtor2 move [rev] [accel] [decel] [rpm]
mtor2 stop
mtor2 accel [value]
mtor2 decel [value]
mtor2 rpm [value]
mtor2 status
```

单位：

- `rev`：`0.1圈`
- `accel`：`rpm/s`
- `decel`：`rpm/s`
- `rpm`：`rpm`

当前方向约定：

- `turn 10`：顺时针
- `turn -10`：逆时针

### 夹爪

```text
clamp ping [id]
clamp status [id]
clamp open [speed]
clamp close [speed]
clamp move [position] [speed]
clamp grip [load] [speed] [step]
clamp release [delta] [speed]
```

单位：

- `position`：舵机原始位置值
- `load`：`0.1%`
- `current`：`6.5mA`

当前夹爪标定：

- `open = 800`
- `close = 2048`

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
