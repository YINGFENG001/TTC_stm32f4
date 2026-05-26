# Device_Task 状态机监控与夹取保持说明

## 代码位置

主要实现文件：

```text
User/stepper/bsp_device_usart_ctl.c
```

主循环调用顺序：

```c
DealSerialData();
Device_Task();
Device_ReportDone();
```

`Device_Task()` 每次主循环执行一次，负责刷新步进电机状态，并调度以下监控任务：

```c
Clamp_HoldTask(now);
Vacum_MonitorTask(now);
```

## 周期参数

当前夹爪保力和吸盘掉落检测共用快速检测周期：

```c
#define DEVICE_FAST_CHECK_INTERVAL_MS  200U
#define VACUM_DROP_HIT_LIMIT           5U
```

当前快速故障检测固定使用 `200ms`，不提供命令行查询或修改入口。

## 夹爪夹取流程

夹爪命令：

```text
clamp grip [load] [openPos/Pct]
```

`openPos/Pct` 为可选位置参数：

- 不传时，从当前位置开始精细闭合。
- 传入不带 `%` 的整数时按绝对位置解析，范围 `700~2048`，`700=张开`，`2048=闭合`。
- 传入带 `%` 的整数时按开度百分比解析，范围 `0%~100%`，`100%` 等同 `700`，`0%` 等同 `2048`。
- `status` 和夹爪动作返回中的位置统一显示为 `openPos/Pct = 700 (100.0%)`。

夹取流程由 `Clamp_GripTwoStage()` 实现：

```c
#define CLAMP_GRIP_FINE_STEP           10U
#define CLAMP_GRIP_PRELOAD_PERCENT     60U
```

执行流程：

1. 如传入 `openPos/Pct`，夹爪先运动到换算后的绝对位置。
2. 位置运动阶段只按目标位置结束，不再使用目标负载 `30%` 提前结束。
3. 精细闭合阶段每次推进 `10 position`。
4. 负载达到目标负载的 `60%` 后，进入 PI 保力。

夹取成功后调用：

```c
Clamp_HoldBegin(load, &final_status);
```

## 夹爪 PI 保力

PI 保力由 `Clamp_HoldTask()` 执行，周期为 `DEVICE_FAST_CHECK_INTERVAL_MS`。

当前参数：

```c
#define CLAMP_HOLD_STABLE_ERROR              20
#define CLAMP_HOLD_ADJUST_PRINT_INTERVAL_MS  2000U
#define CLAMP_HOLD_STABLE_PRINT_INTERVAL_MS  20000U
#define CLAMP_HOLD_KP_NUM                    10
#define CLAMP_HOLD_KI_NUM                    2
#define CLAMP_HOLD_GAIN_DEN                  100
#define CLAMP_HOLD_INTEGRAL_LIMIT            1000
#define CLAMP_HOLD_DELTA_LIMIT               15
```

控制逻辑：

```c
error = target_load - abs(current_load);
integral += error;
delta_pos = (Kp * error + Ki * integral) / gain;
target_pos += delta_pos;
```

规则：

- `abs(error) <= 20` 判定为稳定。
- `delta_pos > 0` 表示继续闭合，提高负载。
- `delta_pos < 0` 表示张开一点，降低负载。
- 单次 PI 输出限幅为 `-15~15 position`。
- 积分限幅为 `-1000~1000`。
- 非稳定调整日志最短每 `2000ms` 打印一次。
- 稳定日志最短每 `20000ms` 打印一次。

日志格式：

```text
@info dev=clamp event=hold_pi load=-344 target=400 error=56 pos=1348
@info dev=clamp event=hold_stable load=-264 target=250 error=-14 pos=1349
```

日志中的 `load`、`error`、`pos` 为本次调整后读取到的状态。

## 夹爪掉落检测

夹爪掉落检测在 `Clamp_HoldTask()` 中执行。

当前参数：

```c
#define CLAMP_DROP_LOW_LOAD_PERCENT    40U
#define CLAMP_DROP_LOW_LOAD_HITS       5U
#define CLAMP_DROP_RECOVERY_POS_DELTA  80U
```

判定条件：

- 当前负载低于目标负载的 `40%`。
- 连续命中 `5` 个快速检测周期。
- 为恢复负载累计闭合推进超过 `80 position`。

满足条件后，关闭夹爪扭矩并上报：

```text
@fault dev=clamp event=drop load=-108 target=300 pos=1459 recover=90 action=torque_off
```

## 吸盘掉落检测

吸盘保持由以下入口启动：

```c
DeviceApi_VacumGrip();
Vacum_HoldBegin();
```

串口入口：

```text
vacum grip
```

掉落检测由 `Vacum_MonitorTask()` 执行，周期为 `DEVICE_FAST_CHECK_INTERVAL_MS`。

当前参数：

```c
#define VACUM_DROP_PERCENT_LIMIT  5U
#define VACUM_DROP_HIT_LIMIT      5U
```

判定条件：

- `vac1 <= 5%` 或 `vac2 <= 5%`。
- 连续命中 `5` 个快速检测周期。

满足条件后，执行 `Evs08_Stop()` 并上报：

```text
@fault dev=vacum event=drop vac1=4 vac2=63 obj1=0 obj2=1 action=stop
```

## 自动上报

当前自动上报分为两类：

```text
@info  夹爪 PI 调整或稳定保持
@fault 掉落、状态读取异常等故障
```

周期性 `@status` 自动打印已关闭。手动查看状态仍使用：

```text
status
clamp status
vacum status
mtor1 status
mtor2 status
```
