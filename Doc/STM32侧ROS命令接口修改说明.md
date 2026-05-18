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
- 新增步进电机命令处理：
  `status`、`move`、`turn`、`stop`、`accel`、`decel`、`rpm`。
- 新增夹爪命令处理：
  `status`、`move`、`open`、`close`、`grip`、`release`。
- 新增吸盘命令处理：
  `set`、`grip`、`release`、`stop`、`status`。
- 吸盘设备名同时兼容 `vacum` 和 `vacuum`。
- 新增全局状态命令 `#<id> status`，依次输出 `mtor1`、`mtor2`、`clamp`、`vacum` 状态，最后输出 `@done id=<id> dev=system cmd=status result=ok`。
- `move/turn` 步进命令启动成功后调用 `DeviceApi_BindStepperRosCmd()`，绑定命令 id，等待后续完成上报。

## User/stepper/bsp_device_usart_ctl.h

- 新增 `DeviceApiResult`，统一设备 API 返回值。
- 新增 `DeviceStepperStatus`，提供步进电机使能、运行、当前位置、目标位置、错误码和运动参数。
- 新增 `DeviceClampStatus`，提供夹爪 id、位置、速度、负载、电压、温度、电流和状态。
- 新增 `DeviceVacumStatus`，提供吸盘状态寄存器、故障、忙状态、物体检测、真空度、温度和总线电压。
- 新增步进电机公共 API：
  `DeviceApi_StepperStatus()`、`DeviceApi_StepperMove()`、`DeviceApi_StepperTurn()`、`DeviceApi_StepperStop()`、`DeviceApi_StepperSetAccel()`、`DeviceApi_StepperSetDecel()`、`DeviceApi_StepperSetRpm()`。
- 新增 `DeviceApi_BindStepperRosCmd()`，用于记录步进电机异步完成事件对应的 ROS 命令。
- 新增夹爪公共 API：
  `DeviceApi_ClampStatus()`、`DeviceApi_ClampMove()`、`DeviceApi_ClampOpen()`、`DeviceApi_ClampClose()`、`DeviceApi_ClampGrip()`、`DeviceApi_ClampRelease()`。
- 新增吸盘公共 API：
  `DeviceApi_VacumSet()`、`DeviceApi_VacumGrip()`、`DeviceApi_VacumRelease()`、`DeviceApi_VacumStop()`、`DeviceApi_VacumStatus()`。

## User/stepper/bsp_device_usart_ctl.c

- 引入 `bsp_ros_protocol.h`。
- `EndDevice` 增加：
  `has_ros_cmd`、`ros_cmd_id`、`ros_cmd_name`。
- 初始化 `devices[]` 时补充 ROS 命令绑定字段默认值。
- `Device_ReportDone()` 增加 ROS 完成上报分支：
  - 有 ROS 命令绑定时输出 `@done id=<id> dev=<device> cmd=<cmd> result=ok rev=<rev>`。
  - 无 ROS 命令绑定时保留原人工串口输出 `mtorX done rev=`。
  - 输出后清除 ROS 命令绑定字段。
- 新增底层结果转换函数：
  - `StepperCmdResult` 转 `DeviceApiResult`。
  - `BusServoResult` 转 `DeviceApiResult`。
  - `GripperResult` 转 `DeviceApiResult`。
  - `ModbusResult` 转 `DeviceApiResult`。
- 新增夹爪和吸盘状态填充函数，将底层状态结构转换为 ROS 公共状态结构。
- 新增 `DeviceApi_StepperStart()`，复用现有步进机械换算、范围检查和 `stepper_move_T()`。
- 新增步进 `DeviceApi_*` 实现，供 ROS 协议层调用。
- 新增夹爪 `DeviceApi_*` 实现，复用现有 `Gripper_*()` 和 `BusServo_*()`。
- 新增吸盘 `DeviceApi_*` 实现，复用现有 `Evs08_*()`。
- `DealSerialData()` 修改为：
  - 先调用 `RosProtocol_TryDispatch()`。
  - 非 ROS 命令返回 `FALSE` 后继续调用原 `Command_Dispatch()`。
  - 原人工串口命令保持兼容。

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
- 步进电机 `move/turn` 是异步动作：启动成功先输出 `@ack`，完成后由 `Device_ReportDone()` 输出 `@done`。
- 夹爪和吸盘当前动作是同步调用：动作完成或底层调用返回后直接输出 `@done` 或 `@ack`。
- 全局 `#<id> status` 必须以 `@done id=<id> dev=system cmd=status result=ok` 结束，避免 ROS 侧一直等待。
- 协议字段保持无中文、无单位后缀、无空格值，方便 ROS 端稳定解析。
