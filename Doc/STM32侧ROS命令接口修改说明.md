# STM32侧ROS命令接口修改说明

## User/ros/bsp_ros_protocol.h

- 新增 ROS 协议层头文件。
- 声明 `RosProtocol_TryDispatch(char *line)`，用于识别并处理 `#` 开头的 ROS 命令。
- 声明 `RosProtocol_ReportStepperDone()`，用于步进电机异步完成时输出 ROS `@done`。

## User/ros/bsp_ros_protocol.c

- 新增 ROS 命令解析入口，支持格式：
  `#<id> <device> <action> [args...]`
- 新增参数解析函数，支持有符号整数、无符号整数、`uint16_t`、`uint8_t` 参数校验。
- 新增统一协议输出函数：
  `@ack`、`@done`、`@state`、`@err`。
- 新增 `DeviceApiResult` 到 ROS 错误码字符串的映射。
- 新增步进电机 ROS 命令处理：
  `status`、`move`、`stop`、`accel`、`decel`、`rpm`。
  人工串口步进命令当前使用 `rpm` 和 `set [accel] [decel] [gear] [micro]` 管理参数。
  ROS 侧 `rpm` 与人工串口一致，支持有符号输入：`rpm N` 为 `CW`，`rpm -N` 为 `CCW`，`rpm 0` 为减速停机。
  ROS 侧 `move` 启动成功回显会附加 `dir=CW/CCW`，连续运行时还会附加 `continuous=1`。
- 新增夹爪命令处理：
  `status`、`move`、`open`、`close`、`grip`、`release`。
  - `move [openPos/Pct]` 支持夹爪绝对位置 `700~2048` 或实测物理开度百分比 `0%~100%`，其中 `100%` 等同 `700`，对应最大张开约 `72mm`。
  - `grip [load] [openPos/Pct]` 中 `openPos/Pct` 为可选参数。
  - 夹爪状态和动作返回中的位置统一显示为 `openPos/Pct = 700 (100.0%)`，百分比由实测标定表插值得出。
- 新增吸盘命令处理：
  `set`、`grip`、`release`、`stop`、`status`。
- 吸盘设备名同时兼容 `vacum` 和 `vacuum`。
- 新增全局状态命令 `#<id> status`，依次输出 `mtor1`、`mtor2`、`clamp`、`vacum` 状态，最后输出 `@done id=<id> dev=system cmd=status result=ok`。
- `move` 步进命令启动成功后调用 `DeviceApi_BindStepperRosCmd()`，绑定命令 id，等待后续完成上报。

## User/stepper/bsp_device_usart_ctl.h

- 新增 `DeviceApiResult`，统一设备 API 返回值。
- 新增 `DeviceStepperStatus`，提供步进电机使能、运行、当前位置、目标位置、错误码和运动参数。
- 新增 `DeviceClampStatus`，提供夹爪 id、位置、速度、负载、电压、温度、电流和状态。
- 新增 `DeviceVacumStatus`，提供吸盘状态寄存器、故障、忙状态、物体检测、真空度、温度和总线电压。
- 新增步进电机公共 API：
  `DeviceApi_StepperStatus()`、`DeviceApi_StepperMove()`、`DeviceApi_StepperRun()`、`DeviceApi_StepperStop()`、`DeviceApi_StepperSetAccel()`、`DeviceApi_StepperSetDecel()`、`DeviceApi_StepperSetRpm()`、`DeviceApi_StepperSetSignedRpm()`。
- 新增 `DeviceApi_BindStepperRosCmd()`，用于记录步进电机异步完成事件对应的 ROS 命令。
- 新增夹爪公共 API：
  `DeviceApi_ClampStatus()`、`DeviceApi_ClampMove()`、`DeviceApi_ClampOpen()`、`DeviceApi_ClampClose()`、`DeviceApi_ClampGrip()`、`DeviceApi_ClampRelease()`。
- 新增吸盘公共 API：
  `DeviceApi_VacumSet()`、`DeviceApi_VacumGrip()`、`DeviceApi_VacumRelease()`、`DeviceApi_VacumStop()`、`DeviceApi_VacumStatus()`。

## User/stepper/bsp_device_usart_ctl.c

- 引入 `bsp_ros_protocol.h`。
- 定义统一设备表 `devices[]`，其中 `EndDevice` 包含：
  `has_ros_cmd`、`ros_cmd_id`、`ros_cmd_name`。
- `Command_Dispatch()` 根据设备名分发到 `Stepper_Command()`、`Clamp_Command()`、`Vacum_Command()`。
- `Device_Task()` 刷新步进电机状态，并调度 `Clamp_HoldTask(now)`、`Vacum_MonitorTask(now)`。
- `Device_ReportDone()` 增加 ROS 完成上报分支：
  - 有 ROS 命令绑定时输出 `@done id=<id> dev=<device> cmd=<cmd> result=ok rev=<rev>`。
  - 无 ROS 命令绑定时保留原人工串口输出 `mtorX done rev=`。
  - 输出后清除 ROS 命令绑定字段。
- `DealSerialData()` 修改为：
  - 先调用 `RosProtocol_TryDispatch()`。
  - 非 ROS 命令返回 `FALSE` 后继续调用原 `Command_Dispatch()`。
  - 原人工串口命令保持兼容。

## 设备 API 实现位置

- 步进电机 `DeviceApi_Stepper*()` 当前实现在 `User/stepper/bsp_stepper_ctl.c`，复用机械换算、范围检查和 `stepper_move_T()`。
- 夹爪 `DeviceApi_Clamp*()` 当前实现在 `User/gripper/bsp_clamp_ctl.c`，复用 `Gripper_*()` 和 `BusServo_*()`。
- 吸盘 `DeviceApi_Vacum*()` 当前实现在 `User/vacum/bsp_vacum_ctl.c`，复用 `Evs08_*()`。
- 公共状态结构和 API 声明仍保留在 `User/stepper/bsp_device_usart_ctl.h`，供 ROS 协议层统一调用。

## User/gripper/bsp_bus_servo.c

- 本文件内局部补充 `TRUE/FALSE` 宏定义。
- 新增 `BusServo_RecvPacketChecked()`，在原应答包解析基础上增加 `fail_on_servo_error` 参数。
- 原 `BusServo_RecvPacket()` 保持写命令、ping 等旧行为，舵机 error 字节非 0 时仍返回 `BUS_SERVO_STATUS_ERROR`。
- `BusServo_ReadData()` 改为调用 `BusServo_RecvPacketChecked(..., FALSE)`。
- 状态读取时只要帧头、id、长度和校验正确，就返回 `BUS_SERVO_OK`，舵机 error 字节通过 `servo_error` 传出，不再导致 `clamp open/close` 误报 `status_error`。

## Project/Fire-F407.uvprojx

- 在 USER 分组中新增 `bsp_ros_protocol.c`。
- 新增文件路径：
  `..\User\ros\bsp_ros_protocol.c`
- 使 ROS 协议层参与 Keil 工程编译和链接。

## Doc/修改报告.md

- 新增按文件归类的简要修改报告。
- 记录新增 ROS 协议层、设备 API、串口分流、工程文件变更和构建验证情况。

## 维护要点

- 人工串口命令保持原入口和原输出；ROS 命令只通过 `#<id>` 前缀进入 `RosProtocol_TryDispatch()`。
- ROS 侧只解析 `@` 开头的协议行，普通调试日志不要作为 ROS 状态依据。
- 新增 ROS 命令时优先扩展 `User/ros/bsp_ros_protocol.c`，不要在人工命令处理函数里增加 ROS 输出分支。
- 新增设备动作时先补 `DeviceApi_*`，再由 ROS 协议层调用，避免复制底层执行逻辑。
- `DeviceApi_*` 是 ROS 和人工串口之间的共享边界，调整函数签名时需要同步检查 `User/ros/bsp_ros_protocol.c`、对应设备命令入口和公共头文件声明。
- `bsp_device_usart_ctl.c` 只保留统一设备表、串口分发、帮助信息、全局状态和周期任务调度；设备专有命令和状态机应继续放在各自的 `*_ctl.c` 文件中。
- 步进电机 `move` 是异步动作：启动成功先输出 `@ack`，完成后由 `Device_ReportDone()` 输出 `@done`。
- 夹爪和吸盘当前动作是同步调用：动作完成或底层调用返回后直接输出 `@done` 或 `@ack`。
- 全局 `#<id> status` 必须以 `@done id=<id> dev=system cmd=status result=ok` 结束，避免 ROS 侧一直等待。
- 协议字段保持无中文；夹爪位置字段当前使用 `openPos/Pct = pos (pct%)` 的双格式显示，`pct` 为实测物理开度百分比。
