/**
  ******************************************************************************
 * @file    bsp_device_usart_ctl.c
 * @brief   多设备名称式串口命令分发（按设备输出轴机械单位输入）
  ******************************************************************************
  */

#include "./stepper/bsp_device_usart_ctl.h"
#include "./gripper/bsp_bus_servo.h"
#include "./gripper/bsp_gripper.h"
#include "./vacum/bsp_evs08.h"
#include "./ros/bsp_ros_protocol.h"

typedef enum {
  DEVICE_MTOR1 = 0,
  DEVICE_MTOR2,
  DEVICE_CLAMP,
  DEVICE_VACUM,
  DEVICE_NUM
} DeviceId;

typedef enum {
  DEVICE_TYPE_STEPPER = 0,
  DEVICE_TYPE_SERVO,
  DEVICE_TYPE_VACUM
} DeviceType;

typedef enum {
  DEV_IDLE = 0,
  DEV_RUNNING,
  DEV_DONE,
  DEV_DISABLED,
  DEV_NOT_READY,
  DEV_ERROR
} DeviceState;

/* 电机本体和机械传动参数 */
typedef struct {
  uint16_t motor_steps_per_rev; /* 电机整步数/圈，例如200 */
  uint16_t micro_step;          /* 驱动器细分数，例如8细分 */
  uint16_t gear_num;            /* 传动比的分子，表示电机轴圈数 */
  uint16_t gear_den;            /* 传动比的分母，表示输出轴圈数 */
} StepperMechanicalConfig;

/* 单台电机对外暴露的参数范围限制 */
typedef struct {
  int32_t min_rev_0p1;          /* 最小位移输入值，单位0.1圈 */
  int32_t max_rev_0p1;          /* 最大位移输入值，单位0.1圈 */
  uint32_t min_rpm;             /* 最小速度输入值，单位rpm */
  uint32_t max_rpm;             /* 最大速度输入值，单位rpm */
  uint32_t min_accel_rpm_s;     /* 最小加速度输入值，单位rpm/s */
  uint32_t max_accel_rpm_s;     /* 最大加速度输入值，单位rpm/s */
  uint32_t min_decel_rpm_s;     /* 最小减速度输入值，单位rpm/s */
  uint32_t max_decel_rpm_s;     /* 最大减速度输入值，单位rpm/s */
  uint32_t max_motor_rpm;       /* 电机轴等效最高转速限制，单位rpm */
} StepperSafetyLimit;

/* 单台电机当前保存的控制参数 */
typedef struct {
  int32_t rev_0p1;          /* 当前目标位移，单位0.1圈 */
  uint32_t accel_rpm_s;     /* 当前加速度，单位rpm/s */
  uint32_t decel_rpm_s;     /* 当前减速度，单位rpm/s */
  uint32_t rpm;             /* 当前目标速度，单位rpm */
} StepperParam;

/* 单台电机的完整配置：机械参数 + 限制 + 默认控制参数 */
typedef struct {
  StepperMechanicalConfig mech; /* 机械和传动参数 */
  StepperSafetyLimit limit;     /* 参数范围限制 */
  StepperParam param;           /* 当前默认/运行参数 */
} StepperDeviceConfig;

typedef struct {
  uint16_t speed;
  uint16_t grip_step;
  uint16_t release_delta;
} ClampParam;

typedef struct {
  uint8_t active;
  uint16_t target_load;
  uint8_t low_load_hits;
  int16_t integral;
  uint16_t last_target_pos;
  uint16_t low_load_start_pos;
  int16_t last_load_abs;
  uint32_t last_adjust_print_tick;
  uint32_t last_stable_print_tick;
  BusServoStatus last_status;
} ClampHoldContext;

typedef struct {
  uint8_t active;
  uint8_t drop_hits;
  Evs08Status last_status;
} VacumHoldContext;

typedef struct {
  uint8_t active;
  int32_t last_position;
  uint8_t no_progress_hits;
} StepperMonitorContext;

/* 串口层维护的设备状态快照 */
typedef struct {
  DeviceId id;           /* 设备编号 */
  const char *name;      /* 串口命令名称 */
  DeviceType type;       /* 设备类型 */
  DeviceState state;     /* 当前状态 */
  uint8_t enabled;       /* 是否使能输出 */
  int32_t position;      /* 当前位置信息，单位0.1圈 */
  int32_t target;        /* 当前目标位置信息，单位0.1圈 */
  int32_t value;         /* 预留的通用数值状态 */
  uint32_t error_code;   /* 设备错误码 */
  uint8_t has_ros_cmd;
  uint32_t ros_cmd_id;
  const char *ros_cmd_name;
} EndDevice;

#define INPUT_REV_SCALE                10
#define MTOR_MAX_OUTPUT_REV_0P1        100000
#define DEVICE_MONITOR_INTERVAL_DEFAULT_MS 2000U
#define DEVICE_MONITOR_INTERVAL_MIN_MS 1000U
#define DEVICE_MONITOR_INTERVAL_MAX_MS 60000U
#define DEVICE_FAST_CHECK_INTERVAL_MS  200U
#define DEVICE_STALL_HIT_LIMIT         5U
#define CLAMP_DROP_LOW_LOAD_PERCENT    40U
#define CLAMP_DROP_LOW_LOAD_HITS       5U
#define CLAMP_DROP_RECOVERY_POS_DELTA  80U
#define CLAMP_GRIP_FINE_STEP           10U
#define CLAMP_GRIP_PRELOAD_PERCENT     60U
#define CLAMP_GRIP_FAST_STOP_PERCENT   30U
#define CLAMP_HOLD_ADJUST_PRINT_INTERVAL_MS 2000U
#define CLAMP_HOLD_STABLE_PRINT_INTERVAL_MS 20000U
#define CLAMP_HOLD_STABLE_ERROR        20
#define CLAMP_HOLD_KP_NUM              10
#define CLAMP_HOLD_KI_NUM              2
#define CLAMP_HOLD_GAIN_DEN            100
#define CLAMP_HOLD_INTEGRAL_LIMIT      1000
#define CLAMP_HOLD_DELTA_LIMIT         15
#define VACUM_DROP_PERCENT_LIMIT       5U
#define DEVICE_FAULT_STALL             1001U
#define DEVICE_FAULT_DROP              1002U

static EndDevice devices[DEVICE_NUM] = {
  {DEVICE_MTOR1, "mtor1", DEVICE_TYPE_STEPPER, DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0},
  {DEVICE_MTOR2, "mtor2", DEVICE_TYPE_STEPPER, DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0},
  {DEVICE_CLAMP, "clamp", DEVICE_TYPE_SERVO,   DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0},
  {DEVICE_VACUM, "vacum", DEVICE_TYPE_VACUM,   DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0}
};

static StepperDeviceConfig stepper_cfg[STEPPER_NUM] = {
  /* mtor1：200步/圈，8细分，1:1直驱 */
  {
    {200, 8, 1, 1},
    {-MTOR_MAX_OUTPUT_REV_0P1, MTOR_MAX_OUTPUT_REV_0P1, 1, 600, 60, 500, 60, 500, 600},
    {50, 100, 100, 100}
  },
  /* mtor2：200步/圈，8细分，1:1直驱 */
  {
    {200, 8, 1, 1},
    {-MTOR_MAX_OUTPUT_REV_0P1, MTOR_MAX_OUTPUT_REV_0P1, 1, 600, 60, 500, 60, 500, 600},
    {50, 100, 100, 100}
  }
};

static ClampParam clamp_param = {
  GRIPPER_SPEED_DEFAULT,
  GRIPPER_GRIP_STEP_DEFAULT,
  GRIPPER_RELEASE_DELTA_DEFAULT
};

static ClampHoldContext clamp_hold = {FALSE, 0, 0, 0, 0, 0, 0, 0, 0, {0}};
static VacumHoldContext vacum_hold = {FALSE, 0, {0}};
static StepperMonitorContext stepper_monitor[STEPPER_NUM] = {
  {FALSE, 0, 0},
  {FALSE, 0, 0}
};
static uint32_t device_monitor_interval_ms = DEVICE_MONITOR_INTERVAL_DEFAULT_MS;

static void Stepper_ApplyMechanicalConfig(void)
{
  static uint8_t applied = FALSE;
  uint8_t i;
  uint32_t pulses_per_motor_rev;

  if (applied == TRUE)
  {
    return;
  }

  for (i = 0; i < STEPPER_NUM; i++)
  {
    pulses_per_motor_rev = (uint32_t)stepper_cfg[i].mech.motor_steps_per_rev * stepper_cfg[i].mech.micro_step;
    Stepper_SetMotorPulsesPerRev(i, pulses_per_motor_rev);
  }

  applied = TRUE;
}

static const char *Stepper_ResultName(StepperCmdResult result)
{
  switch (result)
  {
    case STEPPER_CMD_OK:          return "ok";
    case STEPPER_CMD_BUSY:        return "busy";
    case STEPPER_CMD_DISABLED:    return "disabled";
    case STEPPER_CMD_ID_ERROR:    return "id_error";
    case STEPPER_CMD_PARAM_ERROR: return "param_error";
    default:                      return "unknown";
  }
}

static void Clamp_PrintStatusFields(const BusServoStatus *status_data);
static void Clamp_FillExtraStatus(uint8_t servo_id, BusServoStatus *status_data);
static uint16_t Clamp_OpenPercentToPosition(uint8_t open_percentage);
static void Vacum_Command(uint8_t device_id, int argc, char *argv[]);
static void Device_PrintFullStatus(void);
static void Clamp_HoldBegin(uint16_t target_load, const BusServoStatus *status_data);
static void Clamp_HoldEnd(void);
static GripperResult Clamp_GripTwoStage(uint8_t servo_id, uint16_t load_threshold,
                                        uint8_t has_open_percentage, uint8_t open_percentage,
                                        BusServoStatus *final_status, BusServoResult *servo_result);
static void Vacum_HoldBegin(void);
static void Vacum_HoldEnd(void);
static void Clamp_HoldTask(uint32_t now);
static void Vacum_MonitorTask(uint32_t now);
static void Device_MonitorTask(uint32_t now);

static int Device_FindByName(const char *name)
{
  uint8_t i;

  for (i = 0; i < DEVICE_NUM; i++)
  {
    if (strcmp(name, devices[i].name) == 0)
    {
      return i;
    }
  }
  return -1;
}

static void PrintFixed1Signed(int32_t value)
{
  int32_t abs_val;

  if (value < 0)
  {
    putchar('-');
    abs_val = -value;
  }
  else
  {
    abs_val = value;
  }
  printf("%ld.%01ld", (long)(abs_val / 10), (long)(abs_val % 10));
}

static uint32_t Stepper_GetMotorPulsesPerRev(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;

  mech = &stepper_cfg[motor_id].mech;
  return (uint32_t)mech->motor_steps_per_rev * mech->micro_step;
}

static int32_t Stepper_GetPulsesPerOutputRev(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;

  mech = &stepper_cfg[motor_id].mech;
  return (int32_t)((Stepper_GetMotorPulsesPerRev(motor_id) * mech->gear_num) / mech->gear_den);
}

static uint32_t Stepper_OutputRpmToMotorRpm(uint8_t motor_id, uint32_t rpm)
{
  const StepperMechanicalConfig *mech;

  mech = &stepper_cfg[motor_id].mech;
  return (uint32_t)((rpm * mech->gear_num) / mech->gear_den);
}

static uint32_t Stepper_OutputAccelToMotorAccel(uint8_t motor_id, uint32_t accel_rpm_s)
{
  const StepperMechanicalConfig *mech;

  mech = &stepper_cfg[motor_id].mech;
  return (uint32_t)((accel_rpm_s * mech->gear_num) / mech->gear_den);
}

static int32_t Stepper_StepToRev0p1(uint8_t motor_id, int32_t step)
{
  int32_t pulses_per_output_rev;

  pulses_per_output_rev = Stepper_GetPulsesPerOutputRev(motor_id);

  if (step >= 0)
  {
    return (step * INPUT_REV_SCALE + pulses_per_output_rev / 2) / pulses_per_output_rev;
  }
  return (step * INPUT_REV_SCALE - pulses_per_output_rev / 2) / pulses_per_output_rev;
}

/*
 * 输入层单位约定：
 * 1. rev_0p1      : 0.1 圈（按设备输出轴计算）
 * 2. rpm          : rpm（按设备输出轴计算）
 * 3. accel_rpm_s  : rpm/s（按设备输出轴计算）
 * 4. decel_rpm_s  : rpm/s（按设备输出轴计算）
 *
 * 当前硬件前提：
 * 1. 系统时钟 SystemCoreClock = 168MHz
 * 2. TIM8 采用输出比较 Toggle 模式输出步进脉冲
 * 3. TIM_PRESCALER = 31，定时器计数频率 = 168MHz / (31 + 1) = 5.25MHz
 * 4. TIM_PERIOD = 0xFFFF，当前使用16位计数器
 * 5. 两台电机均为 200步/圈、8细分、1:1直驱
 */
static int32_t Stepper_RevToStep(uint8_t motor_id, int32_t rev_0p1)
{
  int32_t pulses_per_output_rev;

  pulses_per_output_rev = Stepper_GetPulsesPerOutputRev(motor_id);

  if (rev_0p1 >= 0)
  {
    return (rev_0p1 * pulses_per_output_rev + INPUT_REV_SCALE / 2) / INPUT_REV_SCALE;
  }
  return (rev_0p1 * pulses_per_output_rev - INPUT_REV_SCALE / 2) / INPUT_REV_SCALE;
}

static uint32_t Stepper_RpmToSpeed(uint8_t motor_id, uint32_t rpm)
{
  float motor_rpm;

  motor_rpm = (float)Stepper_OutputRpmToMotorRpm(motor_id, rpm);
  return (uint32_t)((motor_rpm * 2.0f * 3.14159f * 10.0f / 60.0f) + 0.5f);
}

static uint32_t Stepper_RpmPerSecToAccel(uint8_t motor_id, uint32_t accel_rpm_s)
{
  float motor_accel_rpm_s;

  motor_accel_rpm_s = (float)Stepper_OutputAccelToMotorAccel(motor_id, accel_rpm_s);
  return (uint32_t)((motor_accel_rpm_s * 2.0f * 3.14159f * 10.0f / 60.0f) + 0.5f);
}

static uint32_t Stepper_CalcInitialStepDelay(uint8_t motor_id, uint32_t accel_internal)
{
  uint32_t pulses_per_motor_rev;
  float alpha;
  float a_sq;

  if (accel_internal == 0)
  {
    return 0;
  }

  pulses_per_motor_rev = Stepper_GetMotorPulsesPerRev(motor_id);
  alpha = (float)(2.0f * 3.14159f / pulses_per_motor_rev);
  a_sq = (float)(2.0f * 100000.0f * alpha);

  return (uint32_t)((T1_FREQ_148 * sqrt(a_sq / accel_internal)) / 10.0f);
}

static uint8_t Stepper_ParseMoveRev(const char *text, int32_t *rev_0p1,
                                    uint8_t *continuous, uint8_t *dir)
{
  if ((text == 0) || (rev_0p1 == 0) || (continuous == 0) || (dir == 0))
  {
    return FALSE;
  }

  if (strcmp(text, "+0") == 0)
  {
    *rev_0p1 = 0;
    *continuous = TRUE;
    *dir = CW;
    return TRUE;
  }

  if (strcmp(text, "-0") == 0)
  {
    *rev_0p1 = 0;
    *continuous = TRUE;
    *dir = CCW;
    return TRUE;
  }

  *rev_0p1 = atoi(text);
  if (*rev_0p1 == 0)
  {
    return FALSE;
  }

  *continuous = FALSE;
  *dir = (*rev_0p1 < 0) ? CCW : CW;
  return TRUE;
}

static int16_t Device_Abs16(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static uint16_t Clamp_LimitPosition(int32_t position)
{
  if (position < GRIPPER_POS_OPEN_MAX)
  {
    return GRIPPER_POS_OPEN_MAX;
  }
  if (position > GRIPPER_POS_CLOSE_MIN)
  {
    return GRIPPER_POS_CLOSE_MIN;
  }
  return (uint16_t)position;
}

static uint16_t Clamp_LoadPercent(uint16_t load, uint16_t percent)
{
  return (uint16_t)(((uint32_t)load * percent) / 100U);
}

static int16_t Clamp_LimitDelta(int32_t delta)
{
  if (delta > CLAMP_HOLD_DELTA_LIMIT)
  {
    return CLAMP_HOLD_DELTA_LIMIT;
  }
  if (delta < -CLAMP_HOLD_DELTA_LIMIT)
  {
    return -CLAMP_HOLD_DELTA_LIMIT;
  }
  return (int16_t)delta;
}

static int16_t Clamp_LimitIntegral(int32_t integral)
{
  if (integral > CLAMP_HOLD_INTEGRAL_LIMIT)
  {
    return CLAMP_HOLD_INTEGRAL_LIMIT;
  }
  if (integral < -CLAMP_HOLD_INTEGRAL_LIMIT)
  {
    return -CLAMP_HOLD_INTEGRAL_LIMIT;
  }
  return (int16_t)integral;
}

static GripperResult Clamp_ReadServoStatus(uint8_t servo_id, BusServoStatus *status_data,
                                           BusServoResult *servo_result)
{
  BusServoResult result;

  result = BusServo_ReadStatus(servo_id, status_data, 0);
  if (servo_result != 0)
  {
    *servo_result = result;
  }
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_STATUS_ERROR;
  }

  Clamp_FillExtraStatus(servo_id, status_data);
  return GRIPPER_OK;
}

static void Clamp_HoldBegin(uint16_t target_load, const BusServoStatus *status_data)
{
  clamp_hold.active = TRUE;
  clamp_hold.target_load = target_load;
  clamp_hold.low_load_hits = 0;
  clamp_hold.integral = 0;
  clamp_hold.last_target_pos = 0;
  clamp_hold.low_load_start_pos = 0;
  clamp_hold.last_adjust_print_tick = 0;
  clamp_hold.last_stable_print_tick = 0;
  if (status_data != 0)
  {
    clamp_hold.last_status = *status_data;
    clamp_hold.last_target_pos = Clamp_LimitPosition(status_data->position);
    clamp_hold.last_load_abs = Device_Abs16(status_data->load);
  }
  devices[DEVICE_CLAMP].state = DEV_RUNNING;
}

static void Clamp_HoldEnd(void)
{
  clamp_hold.active = FALSE;
  clamp_hold.target_load = 0;
  clamp_hold.low_load_hits = 0;
  clamp_hold.integral = 0;
  clamp_hold.last_target_pos = 0;
  clamp_hold.low_load_start_pos = 0;
  clamp_hold.last_load_abs = 0;
  clamp_hold.last_adjust_print_tick = 0;
  clamp_hold.last_stable_print_tick = 0;
}

static GripperResult Clamp_GripTwoStage(uint8_t servo_id, uint16_t load_threshold,
                                        uint8_t has_open_percentage, uint8_t open_percentage,
                                        BusServoStatus *final_status, BusServoResult *servo_result)
{
  BusServoResult result;
  GripperResult grip_result;
  BusServoStatus status_data;
  uint16_t target_pos;
  uint16_t load_abs;
  uint16_t preload_load;
  uint16_t fast_stop_load;
  uint16_t loops;
  uint16_t i;

  if ((load_threshold < GRIPPER_GRIP_LOAD_MIN) ||
      (load_threshold > GRIPPER_GRIP_LOAD_MAX) ||
      (open_percentage > 100))
  {
    return GRIPPER_RANGE_ERROR;
  }

  result = BusServo_SetTorqueEnable(servo_id, 1);
  if (servo_result != 0)
  {
    *servo_result = result;
  }
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_TORQUE_ON_ERROR;
  }

  grip_result = Clamp_ReadServoStatus(servo_id, &status_data, servo_result);
  if (grip_result != GRIPPER_OK)
  {
    return grip_result;
  }
  if (final_status != 0)
  {
    *final_status = status_data;
  }

  fast_stop_load = Clamp_LoadPercent(load_threshold, CLAMP_GRIP_FAST_STOP_PERCENT);
  preload_load = Clamp_LoadPercent(load_threshold, CLAMP_GRIP_PRELOAD_PERCENT);

  if (has_open_percentage == TRUE)
  {
    target_pos = Clamp_OpenPercentToPosition(open_percentage);
    result = BusServo_MoveRaw(servo_id, target_pos, clamp_param.speed);
    if (servo_result != 0)
    {
      *servo_result = result;
    }
    if (result != BUS_SERVO_OK)
    {
      return GRIPPER_MOVE_ERROR;
    }

    loops = (uint16_t)(GRIPPER_MOVE_TIMEOUT_MS / GRIPPER_POLL_INTERVAL_MS);
    if (loops == 0)
    {
      loops = 1;
    }

    for (i = 0; i < loops; i++)
    {
      delay_ms(GRIPPER_POLL_INTERVAL_MS);
      grip_result = Clamp_ReadServoStatus(servo_id, &status_data, servo_result);
      if (grip_result != GRIPPER_OK)
      {
        return grip_result;
      }
      if (final_status != 0)
      {
        *final_status = status_data;
      }

      load_abs = (uint16_t)Device_Abs16(status_data.load);
      if ((load_abs >= fast_stop_load) ||
          (Device_Abs16((int16_t)(target_pos - status_data.position)) <= GRIPPER_MOVE_TOLERANCE))
      {
        break;
      }
    }
  }

  target_pos = Clamp_LimitPosition(status_data.position);
  while (target_pos < GRIPPER_POS_CLOSE_MIN)
  {
    load_abs = (uint16_t)Device_Abs16(status_data.load);
    if (load_abs >= preload_load)
    {
      return GRIPPER_OK;
    }

    target_pos = Clamp_LimitPosition((int32_t)target_pos + CLAMP_GRIP_FINE_STEP);
    result = BusServo_MoveRaw(servo_id, target_pos, clamp_param.speed);
    if (servo_result != 0)
    {
      *servo_result = result;
    }
    if (result != BUS_SERVO_OK)
    {
      return GRIPPER_MOVE_ERROR;
    }

    delay_ms(GRIPPER_POLL_INTERVAL_MS);
    grip_result = Clamp_ReadServoStatus(servo_id, &status_data, servo_result);
    if (grip_result != GRIPPER_OK)
    {
      return grip_result;
    }
    if (final_status != 0)
    {
      *final_status = status_data;
    }
  }

  return (Device_Abs16(status_data.load) >= (int16_t)preload_load) ? GRIPPER_OK : GRIPPER_NO_OBJECT;
}

static void Vacum_HoldBegin(void)
{
  vacum_hold.active = TRUE;
  vacum_hold.drop_hits = 0;
  devices[DEVICE_VACUM].state = DEV_RUNNING;
}

static void Vacum_HoldEnd(void)
{
  vacum_hold.active = FALSE;
  vacum_hold.drop_hits = 0;
}

static void StepperMonitor_Reset(uint8_t motor_id)
{
  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  stepper_monitor[motor_id].active = FALSE;
  stepper_monitor[motor_id].last_position = stepPosition[motor_id];
  stepper_monitor[motor_id].no_progress_hits = 0;
}

static void Clamp_HoldTask(uint32_t now)
{
  static uint32_t last_control_tick = 0;
  uint8_t servo_error;
  BusServoResult result;
  BusServoStatus status_data;
  int16_t load_abs;
  int16_t error;
  int16_t delta_pos;
  int32_t pi_out;
  uint16_t low_load_limit;
  uint16_t recovery_delta;
  uint16_t fault_target_load;
  uint16_t next_pos;

  if (clamp_hold.active != TRUE)
  {
    return;
  }

  if ((now - last_control_tick) < DEVICE_FAST_CHECK_INTERVAL_MS)
  {
    return;
  }
  last_control_tick = now;

  servo_error = 0;
  result = BusServo_ReadStatus(GRIPPER_SERVO_ID_DEFAULT, &status_data, &servo_error);
  if (result != BUS_SERVO_OK)
  {
    devices[DEVICE_CLAMP].state = DEV_ERROR;
    devices[DEVICE_CLAMP].error_code = result;
    Clamp_HoldEnd();
    (void)BusServo_SetTorqueEnable(GRIPPER_SERVO_ID_DEFAULT, 0);
    printf("\n@fault dev=clamp event=status_error result=%s servo_err=0x%02X action=torque_off",
           BusServo_ResultName(result), servo_error);
    return;
  }

  Clamp_FillExtraStatus(GRIPPER_SERVO_ID_DEFAULT, &status_data);
  clamp_hold.last_status = status_data;
  devices[DEVICE_CLAMP].position = status_data.position;
  devices[DEVICE_CLAMP].target = status_data.position;
  devices[DEVICE_CLAMP].value = status_data.status;
  devices[DEVICE_CLAMP].error_code = 0;
  devices[DEVICE_CLAMP].state = DEV_RUNNING;

  load_abs = Device_Abs16(status_data.load);
  error = (int16_t)clamp_hold.target_load - load_abs;
  if (Device_Abs16(error) <= CLAMP_HOLD_STABLE_ERROR)
  {
    clamp_hold.integral = 0;
    clamp_hold.low_load_hits = 0;
    clamp_hold.low_load_start_pos = 0;
    clamp_hold.last_load_abs = load_abs;
    clamp_hold.last_target_pos = Clamp_LimitPosition(status_data.position);
    if ((clamp_hold.last_stable_print_tick == 0) ||
        ((now - clamp_hold.last_stable_print_tick) >= CLAMP_HOLD_STABLE_PRINT_INTERVAL_MS))
    {
      clamp_hold.last_stable_print_tick = now;
      printf("\n@info dev=clamp event=hold_stable load=%d target=%u error=%d pos=%d",
             status_data.load,
             clamp_hold.target_load,
             error,
             status_data.position);
    }
    return;
  }

  clamp_hold.integral = Clamp_LimitIntegral((int32_t)clamp_hold.integral + error);
  pi_out = ((int32_t)CLAMP_HOLD_KP_NUM * error) +
           ((int32_t)CLAMP_HOLD_KI_NUM * clamp_hold.integral);
  delta_pos = Clamp_LimitDelta(pi_out / CLAMP_HOLD_GAIN_DEN);
  if (delta_pos == 0)
  {
    delta_pos = (error > 0) ? 1 : -1;
  }

  if (clamp_hold.last_target_pos == 0)
  {
    clamp_hold.last_target_pos = Clamp_LimitPosition(status_data.position);
  }
  next_pos = Clamp_LimitPosition((int32_t)clamp_hold.last_target_pos + delta_pos);

  low_load_limit = Clamp_LoadPercent(clamp_hold.target_load, CLAMP_DROP_LOW_LOAD_PERCENT);
  if ((load_abs < low_load_limit) && (delta_pos > 0))
  {
    if (clamp_hold.low_load_hits == 0)
    {
      clamp_hold.low_load_start_pos = clamp_hold.last_target_pos;
    }
    clamp_hold.low_load_hits++;
    recovery_delta = (next_pos >= clamp_hold.low_load_start_pos) ?
                     (uint16_t)(next_pos - clamp_hold.low_load_start_pos) : 0;
    if ((clamp_hold.low_load_hits >= CLAMP_DROP_LOW_LOAD_HITS) &&
        (recovery_delta >= CLAMP_DROP_RECOVERY_POS_DELTA))
    {
      fault_target_load = clamp_hold.target_load;
      Clamp_HoldEnd();
      (void)BusServo_SetTorqueEnable(GRIPPER_SERVO_ID_DEFAULT, 0);
      devices[DEVICE_CLAMP].state = DEV_ERROR;
      devices[DEVICE_CLAMP].error_code = DEVICE_FAULT_DROP;
      printf("\n@fault dev=clamp event=drop load=%d target=%u pos=%d recover=%u action=torque_off",
             status_data.load,
             fault_target_load,
             status_data.position,
             recovery_delta);
      return;
    }
  }
  else
  {
    clamp_hold.low_load_hits = 0;
    clamp_hold.low_load_start_pos = 0;
  }

  result = BusServo_MoveRaw(GRIPPER_SERVO_ID_DEFAULT, next_pos, clamp_param.speed);
  if (result == BUS_SERVO_OK)
  {
    clamp_hold.last_target_pos = next_pos;
    clamp_hold.last_load_abs = load_abs;
    devices[DEVICE_CLAMP].target = next_pos;
    delay_ms(GRIPPER_POLL_INTERVAL_MS);
    if (Clamp_ReadServoStatus(GRIPPER_SERVO_ID_DEFAULT, &status_data, &result) == GRIPPER_OK)
    {
      load_abs = Device_Abs16(status_data.load);
      error = (int16_t)clamp_hold.target_load - load_abs;
      clamp_hold.last_status = status_data;
      clamp_hold.last_load_abs = load_abs;
      devices[DEVICE_CLAMP].position = status_data.position;
      devices[DEVICE_CLAMP].value = status_data.status;
      devices[DEVICE_CLAMP].error_code = 0;
    }
    if ((clamp_hold.last_adjust_print_tick == 0) ||
        ((now - clamp_hold.last_adjust_print_tick) >= CLAMP_HOLD_ADJUST_PRINT_INTERVAL_MS))
    {
      clamp_hold.last_adjust_print_tick = now;
      printf("\n@info dev=clamp event=hold_pi load=%d target=%u error=%d pos=%d",
             status_data.load,
             clamp_hold.target_load,
             error,
             status_data.position);
    }
  }
}

static void Vacum_MonitorTask(uint32_t now)
{
  static uint32_t last_tick = 0;
  ModbusResult result;
  Evs08Status status_data;

  if (vacum_hold.active != TRUE)
  {
    return;
  }

  if ((now - last_tick) < DEVICE_FAST_CHECK_INTERVAL_MS)
  {
    return;
  }
  last_tick = now;

  result = Evs08_ReadStatus(&status_data);
  if (result != MODBUS_OK)
  {
    devices[DEVICE_VACUM].state = DEV_ERROR;
    devices[DEVICE_VACUM].error_code = result;
    Vacum_HoldEnd();
    (void)Evs08_Stop();
    printf("\n@fault dev=vacum event=status_error result=%s action=stop",
           Evs08_ResultName(result));
    return;
  }

  vacum_hold.last_status = status_data;
  devices[DEVICE_VACUM].enabled = (status_data.ch1_enabled || status_data.ch2_enabled) ? TRUE : FALSE;
  devices[DEVICE_VACUM].value = status_data.ch1_status_reg;
  devices[DEVICE_VACUM].error_code = 0;
  devices[DEVICE_VACUM].state = DEV_RUNNING;

  if ((status_data.ch1_vac_percent <= VACUM_DROP_PERCENT_LIMIT) ||
      (status_data.ch2_vac_percent <= VACUM_DROP_PERCENT_LIMIT))
  {
    vacum_hold.drop_hits++;
    if (vacum_hold.drop_hits >= DEVICE_STALL_HIT_LIMIT)
    {
      Vacum_HoldEnd();
      (void)Evs08_Stop();
      devices[DEVICE_VACUM].state = DEV_ERROR;
      devices[DEVICE_VACUM].error_code = DEVICE_FAULT_DROP;
      printf("\n@fault dev=vacum event=drop vac1=%u vac2=%u obj1=%u obj2=%u action=stop",
             status_data.ch1_vac_percent,
             status_data.ch2_vac_percent,
             status_data.ch1_obj,
             status_data.ch2_obj);
    }
    return;
  }
  vacum_hold.drop_hits = 0;
}

static void Device_MonitorTask(uint32_t now)
{
  static uint32_t last_check_tick = 0;
  uint8_t i;

  if ((now - last_check_tick) >= DEVICE_FAST_CHECK_INTERVAL_MS)
  {
    last_check_tick = now;
    for (i = 0; i < STEPPER_NUM; i++)
    {
      if (motor_status[i].running == TRUE)
      {
        if (stepper_monitor[i].active != TRUE)
        {
          stepper_monitor[i].active = TRUE;
          stepper_monitor[i].last_position = stepPosition[i];
          stepper_monitor[i].no_progress_hits = 0;
        }
        else if (stepPosition[i] == stepper_monitor[i].last_position)
        {
          stepper_monitor[i].no_progress_hits++;
          if (stepper_monitor[i].no_progress_hits >= DEVICE_STALL_HIT_LIMIT)
          {
            (void)Stepper_Stop(i);
            devices[i].state = DEV_ERROR;
            devices[i].error_code = DEVICE_FAULT_STALL;
            printf("\n@fault dev=%s event=stall pos=", devices[i].name);
            PrintFixed1Signed(devices[i].position);
            printf(" target=");
            PrintFixed1Signed(devices[i].target);
            printf(" action=stop");
            StepperMonitor_Reset(i);
          }
        }
        else
        {
          stepper_monitor[i].last_position = stepPosition[i];
          stepper_monitor[i].no_progress_hits = 0;
        }
      }
      else
      {
        StepperMonitor_Reset(i);
      }
    }
  }
}

void Device_Task(void)
{
  uint8_t i;
  uint32_t now;

  now = HAL_GetTick();
  Stepper_ApplyMechanicalConfig();

  for (i = 0; i < STEPPER_NUM; i++)
  {
    devices[i].position = Stepper_StepToRev0p1(i, stepPosition[i]);
    devices[i].enabled = motor_status[i].out_ena;

    if (motor_status[i].out_ena != TRUE)
    {
      devices[i].state = DEV_DISABLED;
    }
    else if (motor_status[i].running == TRUE)
    {
      devices[i].state = DEV_RUNNING;
    }
    else if (devices[i].state == DEV_RUNNING)
    {
      devices[i].state = DEV_DONE;
    }
    else if ((devices[i].state != DEV_DONE) && (devices[i].state != DEV_ERROR))
    {
      devices[i].state = DEV_IDLE;
    }
  }

  Clamp_HoldTask(now);
  Vacum_MonitorTask(now);
  Device_MonitorTask(now);
}

void Device_ReportDone(void)
{
  uint8_t i;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    if (devices[i].state == DEV_DONE)
    {
      if (devices[i].has_ros_cmd == TRUE)
      {
        RosProtocol_ReportStepperDone(devices[i].ros_cmd_id,
                                      devices[i].name,
                                      devices[i].ros_cmd_name,
                                      devices[i].position);
        devices[i].has_ros_cmd = FALSE;
        devices[i].ros_cmd_id = 0;
        devices[i].ros_cmd_name = 0;
      }
      else
      {
        printf("\n%s done rev=", devices[i].name);
        PrintFixed1Signed(devices[i].position);
      }
      devices[i].state = DEV_IDLE;
    }
  }
}

static void Clamp_FillExtraStatus(uint8_t servo_id, BusServoStatus *status_data)
{
  uint8_t servo_error;
  uint8_t servo_state;
  int16_t servo_current;

  if (status_data == 0)
  {
    return;
  }

  servo_error = 0;
  servo_state = 0;
  servo_current = 0;

  if (BusServo_ReadServoState(servo_id, &servo_state, &servo_error) == BUS_SERVO_OK)
  {
    status_data->status = servo_state;
  }

  if (BusServo_ReadCurrent(servo_id, &servo_current, &servo_error) == BUS_SERVO_OK)
  {
    status_data->current = servo_current;
  }
}

static void Device_PrintStatus(uint8_t id)
{
  BusServoStatus clamp_status;
  BusServoResult clamp_result;
  uint8_t servo_error;

  Device_Task();

  if (id >= DEVICE_NUM)
  {
    return;
  }

  if (devices[id].type == DEVICE_TYPE_SERVO)
  {
    servo_error = 0;
    clamp_result = BusServo_ReadStatus(GRIPPER_SERVO_ID_DEFAULT, &clamp_status, &servo_error);
    if (clamp_result == BUS_SERVO_OK)
    {
      Clamp_FillExtraStatus(GRIPPER_SERVO_ID_DEFAULT, &clamp_status);
      devices[id].enabled = TRUE;
      devices[id].position = clamp_status.position;
      devices[id].target = clamp_status.position;
      devices[id].value = clamp_status.status;
      devices[id].error_code = 0;
      devices[id].state = DEV_IDLE;

      printf("\nclamp status id=%u result=ok", GRIPPER_SERVO_ID_DEFAULT);
      Clamp_PrintStatusFields(&clamp_status);
    }
    else
    {
      devices[id].state = DEV_ERROR;
      devices[id].error_code = clamp_result;
      printf("\nclamp status id=%u result=%s",
             GRIPPER_SERVO_ID_DEFAULT,
             BusServo_ResultName(clamp_result));
      if (servo_error != 0)
      {
        printf(" servo_err=0x%02X", servo_error);
      }
    }
    return;
  }

  printf("\n%s enabled=%d rev=", devices[id].name, devices[id].enabled);
  PrintFixed1Signed(devices[id].position);
  printf(" target=");
  PrintFixed1Signed(devices[id].target);
  printf(" err=%lu", (unsigned long)devices[id].error_code);
}

static void Stepper_PrintLimit(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;

  printf("\n%s limit rev=", devices[motor_id].name);
  PrintFixed1Signed(limit->min_rev_0p1);
  printf("~");
  PrintFixed1Signed(limit->max_rev_0p1);
  printf(" rpm=%lu~%lu accel=%lu~%lu decel=%lu~%lu gear=%u:%u micro=%u motor_rpm_max=%lu",
         (unsigned long)limit->min_rpm,
         (unsigned long)limit->max_rpm,
         (unsigned long)limit->min_accel_rpm_s,
         (unsigned long)limit->max_accel_rpm_s,
         (unsigned long)limit->min_decel_rpm_s,
         (unsigned long)limit->max_decel_rpm_s,
         mech->gear_num,
         mech->gear_den,
         mech->micro_step,
         (unsigned long)limit->max_motor_rpm);
}

static void ShowCommandHelp(void)
{
  printf("\n命令说明:");
  printf("\n  mtor1/2 move [rev: -10000~10000(0.1圈), +0/-0 continuous]");
  printf("\n  mtor1/2 stop");
  printf("\n  mtor1/2 accel [value: 60~500(rpm/s)]");
  printf("\n  mtor1/2 decel [value: 60~500(rpm/s)]");
  printf("\n  mtor1/2 rpm [value: 1~600]");
  printf("\n  mtor1/2 status");
  printf("\n  clamp ping [id: 0~253]");
  printf("\n  clamp status [id: 0~253]");
  printf("\n  clamp readreg [addr: 0~255]");
  printf("\n  clamp open");
  printf("\n  clamp close");
  printf("\n  clamp move [openPercentage: 0~100(%)]");
  printf("\n  clamp grip [load: 100~900(0.1%%)] [openPercentage: 0~100(%%), optional]");
  printf("\n  clamp release");
  printf("\n  clamp set [speed: 1~3000] [gripStep: 5~100] [releaseDelta: 20~400]");
  printf("\n  vacum set [min_vac:0~100(%%)] [max_vac:0~100(%%)] [timeout:1~255(100ms)]");
  printf("\n  vacum grip");
  printf("\n  vacum release");
  printf("\n  vacum stop");
  printf("\n  vacum status");
  printf("\n  monitor [intervalMs: 1000~60000]");
  printf("\n  status");
  printf("\n示例: mtor1 move 50 -> 5.0圈; mtor1 move +0 -> continuous CW until stop");
  printf("\n");
}

void ShowHelp(void)
{
  Stepper_ApplyMechanicalConfig();

  printf("\n================ 设备概览 ================");
  printf("\n设备:");
  printf("\n  mtor1 : 步进电机0 ENA PE0, DIR PE1, PUL PI5(TIM8_CH1)");
  printf("\n  mtor2 : 步进电机1 ENA PE4, DIR PI8, PUL PI6(TIM8_CH2)");
  printf("\n  clamp : 玄雅STS舵机夹爪  TX PC12, RX PD2(UART5), id=10");
  printf("\n  vacum : 钧舵EVS08真空吸盘 TX PC10, RX PC11(UART4, RS485), DE/RE PH9, id=9");
  printf("\n参数说明:");
  printf("\n  mtor rev   = -10000~10000(0.1圈)，默认 50 = 5.0圈，+0/-0 持续旋转直到 stop");
  printf("\n       accel = 60~500(rpm/s)，默认 100");
  printf("\n       decel = 60~500(rpm/s)，默认 100");
  printf("\n       rpm   = 1~600(rpm)，默认 100");
  printf("\n       stop  = 立即停止当前运动并保持当前位置");
  printf("\n  clamp position = 500~2048");
  printf("\n        move     = openPercentage 0~100, 0=close, 100=open");
  printf("\n        default  = speed 1000, gripStep 30, releaseDelta 100");
  printf("\n        load     = 0.1%%");
  printf("\n        current  = 6.5mA");
  printf("\n  fast check = 200ms, vacum/mtor fault hits = 5");
  printf("\n  vacum min/max_vac = 真空度(%%)");
  printf("\n        timeout     = 1~255(100ms)");
	printf("\n        grip        = 真空吸取");
	printf("\n        release     = 破真空停止");
  printf("\n        stop        = 直接停止");
  printf("\n输入 ? 查看详细命令说明，输入 status 查看全部设备状态\n");
}

static void Stepper_PrintParam(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  mech = &stepper_cfg[motor_id].mech;

  printf("\n%s param rev=", devices[motor_id].name);
  PrintFixed1Signed(stepper_cfg[motor_id].param.rev_0p1);
  printf(" accel=%lurpm/s decel=%lurpm/s rpm=%lurpm gear=%u:%u micro=%u motor_rev_pulse=%lu output_rev_pulse=%ld",
         (unsigned long)stepper_cfg[motor_id].param.accel_rpm_s,
         (unsigned long)stepper_cfg[motor_id].param.decel_rpm_s,
         (unsigned long)stepper_cfg[motor_id].param.rpm,
         mech->gear_num,
         mech->gear_den,
         mech->micro_step,
         (unsigned long)Stepper_GetMotorPulsesPerRev(motor_id),
         (long)Stepper_GetPulsesPerOutputRev(motor_id));
}

static void Stepper_PrintStartResult(uint8_t motor_id, StepperCmdResult result)
{
  printf("\n%s %s", devices[motor_id].name, Stepper_ResultName(result));
  if (result == STEPPER_CMD_OK)
  {
    printf(" rev=");
    PrintFixed1Signed(stepper_cfg[motor_id].param.rev_0p1);
    printf(" accel=%lurpm/s decel=%lurpm/s rpm=%lurpm",
           (unsigned long)stepper_cfg[motor_id].param.accel_rpm_s,
           (unsigned long)stepper_cfg[motor_id].param.decel_rpm_s,
           (unsigned long)stepper_cfg[motor_id].param.rpm);
  }
}

static void Stepper_PrintStopResult(uint8_t motor_id, StepperCmdResult result)
{
  printf("\n%s stop %s rev=", devices[motor_id].name, Stepper_ResultName(result));
  PrintFixed1Signed(devices[motor_id].position);
}

static uint8_t Stepper_CheckUserRange(uint8_t motor_id, const StepperParam *param)
{
  const StepperSafetyLimit *limit;

  limit = &stepper_cfg[motor_id].limit;

  if ((param->rev_0p1 < limit->min_rev_0p1) || (param->rev_0p1 > limit->max_rev_0p1))
  {
    return FALSE;
  }
  if ((param->accel_rpm_s < limit->min_accel_rpm_s) || (param->accel_rpm_s > limit->max_accel_rpm_s))
  {
    return FALSE;
  }
  if ((param->decel_rpm_s < limit->min_decel_rpm_s) || (param->decel_rpm_s > limit->max_decel_rpm_s))
  {
    return FALSE;
  }
  if ((param->rpm < limit->min_rpm) || (param->rpm > limit->max_rpm))
  {
    return FALSE;
  }
  return TRUE;
}

static uint8_t Stepper_CheckMotorEquivalentRange(uint8_t motor_id, const StepperParam *param)
{
  const StepperSafetyLimit *limit;
  uint32_t motor_rpm;
  uint32_t accel_internal;
  uint32_t decel_internal;
  uint32_t accel_delay;
  uint32_t decel_delay;

  limit = &stepper_cfg[motor_id].limit;
  motor_rpm = Stepper_OutputRpmToMotorRpm(motor_id, param->rpm);

  if (motor_rpm > limit->max_motor_rpm)
  {
    return FALSE;
  }

  accel_internal = Stepper_RpmPerSecToAccel(motor_id, param->accel_rpm_s);
  decel_internal = Stepper_RpmPerSecToAccel(motor_id, param->decel_rpm_s);
  accel_delay = Stepper_CalcInitialStepDelay(motor_id, accel_internal);
  decel_delay = Stepper_CalcInitialStepDelay(motor_id, decel_internal);

  if ((accel_delay == 0) || (decel_delay == 0))
  {
    return FALSE;
  }

  if (((accel_delay / 2) > TIM_PERIOD) || ((decel_delay / 2) > TIM_PERIOD))
  {
    return FALSE;
  }

  return TRUE;
}

static DeviceApiResult DeviceApi_FromStepperResult(StepperCmdResult result)
{
  switch (result)
  {
    case STEPPER_CMD_OK:          return DEVICE_API_OK;
    case STEPPER_CMD_BUSY:        return DEVICE_API_BUSY;
    case STEPPER_CMD_DISABLED:    return DEVICE_API_DISABLED;
    case STEPPER_CMD_ID_ERROR:    return DEVICE_API_ID_ERROR;
    case STEPPER_CMD_PARAM_ERROR: return DEVICE_API_PARAM_ERROR;
    default:                      return DEVICE_API_DEVICE_ERROR;
  }
}

static DeviceApiResult DeviceApi_FromBusServoResult(BusServoResult result)
{
  switch (result)
  {
    case BUS_SERVO_OK:             return DEVICE_API_OK;
    case BUS_SERVO_UART_ERROR:     return DEVICE_API_UART_ERROR;
    case BUS_SERVO_TIMEOUT:        return DEVICE_API_TIMEOUT;
    case BUS_SERVO_ID_ERROR:       return DEVICE_API_ID_ERROR;
    case BUS_SERVO_PARAM_ERROR:    return DEVICE_API_PARAM_ERROR;
    case BUS_SERVO_CHECKSUM_ERROR: return DEVICE_API_CRC_ERROR;
    default:                       return DEVICE_API_DEVICE_ERROR;
  }
}

static DeviceApiResult DeviceApi_FromGripperResult(GripperResult result, BusServoResult servo_result)
{
  switch (result)
  {
    case GRIPPER_OK:          return DEVICE_API_OK;
    case GRIPPER_RANGE_ERROR: return DEVICE_API_RANGE_ERROR;
    case GRIPPER_TIMEOUT:     return DEVICE_API_TIMEOUT;
    case GRIPPER_SERVO_ERROR:
    case GRIPPER_STATUS_ERROR:
    case GRIPPER_TORQUE_OFF_ERROR:
    case GRIPPER_TORQUE_ON_ERROR:
    case GRIPPER_MOVE_ERROR:  return DeviceApi_FromBusServoResult(servo_result);
    default:                  return DEVICE_API_DEVICE_ERROR;
  }
}

static DeviceApiResult DeviceApi_FromModbusResult(ModbusResult result)
{
  switch (result)
  {
    case MODBUS_OK:          return DEVICE_API_OK;
    case MODBUS_UART_ERROR:  return DEVICE_API_UART_ERROR;
    case MODBUS_TIMEOUT:     return DEVICE_API_TIMEOUT;
    case MODBUS_CRC_ERROR:   return DEVICE_API_CRC_ERROR;
    case MODBUS_ID_ERROR:    return DEVICE_API_ID_ERROR;
    case MODBUS_PARAM_ERROR: return DEVICE_API_PARAM_ERROR;
    default:                 return DEVICE_API_DEVICE_ERROR;
  }
}

static void DeviceApi_FillClampStatus(uint8_t servo_id, const BusServoStatus *src, DeviceClampStatus *out)
{
  if ((src == 0) || (out == 0))
  {
    return;
  }

  out->servo_id = servo_id;
  out->pos = src->position;
  out->speed = src->speed;
  out->load = src->load;
  out->voltage = src->voltage;
  out->temp = src->temperature;
  out->current = src->current;
  out->state = src->status;
}

static void DeviceApi_FillVacumStatus(const Evs08Status *src, DeviceVacumStatus *out)
{
  if ((src == 0) || (out == 0))
  {
    return;
  }

  out->state1 = src->ch1_status_reg;
  out->state2 = src->ch2_status_reg;
  out->fault = src->fault_reg;
  out->busy1 = src->ch1_busy;
  out->busy2 = src->ch2_busy;
  out->obj1 = src->ch1_obj;
  out->obj2 = src->ch2_obj;
  out->vac1 = src->ch1_vac_percent;
  out->vac2 = src->ch2_vac_percent;
  out->temp = src->temperature;
  out->bus_x10 = src->bus_voltage_x10;
}

static DeviceApiResult DeviceApi_StepperStart(uint8_t motor_id, const StepperParam *param)
{
  int32_t step_target;
  uint32_t accel_internal;
  uint32_t decel_internal;
  uint32_t speed_internal;
  StepperCmdResult result;

  if ((!STEPPER_ID_VALID(motor_id)) || (param == 0))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  Stepper_ApplyMechanicalConfig();

  if ((Stepper_CheckUserRange(motor_id, param) != TRUE) ||
      (Stepper_CheckMotorEquivalentRange(motor_id, param) != TRUE))
  {
    return DEVICE_API_RANGE_ERROR;
  }

  stepper_cfg[motor_id].param = *param;
  step_target = Stepper_RevToStep(motor_id, param->rev_0p1);
  accel_internal = Stepper_RpmPerSecToAccel(motor_id, param->accel_rpm_s);
  decel_internal = Stepper_RpmPerSecToAccel(motor_id, param->decel_rpm_s);
  speed_internal = Stepper_RpmToSpeed(motor_id, param->rpm);

  devices[motor_id].target = param->rev_0p1;
  result = stepper_move_T(motor_id, step_target, accel_internal, decel_internal, speed_internal);
  if (result == STEPPER_CMD_OK)
  {
    devices[motor_id].state = DEV_RUNNING;
  }

  return DeviceApi_FromStepperResult(result);
}

DeviceApiResult DeviceApi_StepperStatus(uint8_t motor_id, DeviceStepperStatus *out)
{
  if ((!STEPPER_ID_VALID(motor_id)) || (out == 0))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  Device_Task();
  out->enabled = devices[motor_id].enabled;
  out->running = (devices[motor_id].state == DEV_RUNNING) ? TRUE : FALSE;
  out->rev_0p1 = devices[motor_id].position;
  out->target_0p1 = devices[motor_id].target;
  out->err = devices[motor_id].error_code;
  out->accel_rpm_s = stepper_cfg[motor_id].param.accel_rpm_s;
  out->decel_rpm_s = stepper_cfg[motor_id].param.decel_rpm_s;
  out->rpm = stepper_cfg[motor_id].param.rpm;
  return DEVICE_API_OK;
}

DeviceApiResult DeviceApi_StepperMove(uint8_t motor_id, int32_t rev_0p1)
{
  StepperParam param;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  param.rev_0p1 = rev_0p1;
  param.accel_rpm_s = stepper_cfg[motor_id].param.accel_rpm_s;
  param.decel_rpm_s = stepper_cfg[motor_id].param.decel_rpm_s;
  param.rpm = stepper_cfg[motor_id].param.rpm;
  return DeviceApi_StepperStart(motor_id, &param);
}

DeviceApiResult DeviceApi_StepperRun(uint8_t motor_id, uint8_t dir)
{
  StepperParam param;
  uint32_t accel_internal;
  uint32_t speed_internal;
  StepperCmdResult result;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  param = stepper_cfg[motor_id].param;
  param.rev_0p1 = 1;
  if ((Stepper_CheckUserRange(motor_id, &param) != TRUE) ||
      (Stepper_CheckMotorEquivalentRange(motor_id, &param) != TRUE))
  {
    return DEVICE_API_RANGE_ERROR;
  }

  accel_internal = Stepper_RpmPerSecToAccel(motor_id, param.accel_rpm_s);
  speed_internal = Stepper_RpmToSpeed(motor_id, param.rpm);
  stepper_cfg[motor_id].param.rev_0p1 = 0;
  devices[motor_id].target = 0;
  result = Stepper_RunContinuous(motor_id, dir, accel_internal, speed_internal);
  if (result == STEPPER_CMD_OK)
  {
    devices[motor_id].state = DEV_RUNNING;
  }
  return DeviceApi_FromStepperResult(result);
}

DeviceApiResult DeviceApi_StepperStop(uint8_t motor_id, int32_t *rev_0p1)
{
  StepperCmdResult result;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  Device_Task();
  result = Stepper_Stop(motor_id);
  devices[motor_id].target = devices[motor_id].position;
  devices[motor_id].state = DEV_IDLE;
  devices[motor_id].has_ros_cmd = FALSE;
  devices[motor_id].ros_cmd_id = 0;
  devices[motor_id].ros_cmd_name = 0;
  if (rev_0p1 != 0)
  {
    *rev_0p1 = devices[motor_id].position;
  }
  return DeviceApi_FromStepperResult(result);
}

DeviceApiResult DeviceApi_StepperSetAccel(uint8_t motor_id, uint32_t accel)
{
  StepperParam param;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  param = stepper_cfg[motor_id].param;
  param.accel_rpm_s = accel;
  if ((Stepper_CheckUserRange(motor_id, &param) != TRUE) ||
      (Stepper_CheckMotorEquivalentRange(motor_id, &param) != TRUE))
  {
    return DEVICE_API_RANGE_ERROR;
  }
  stepper_cfg[motor_id].param = param;
  return DEVICE_API_OK;
}

DeviceApiResult DeviceApi_StepperSetDecel(uint8_t motor_id, uint32_t decel)
{
  StepperParam param;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  param = stepper_cfg[motor_id].param;
  param.decel_rpm_s = decel;
  if ((Stepper_CheckUserRange(motor_id, &param) != TRUE) ||
      (Stepper_CheckMotorEquivalentRange(motor_id, &param) != TRUE))
  {
    return DEVICE_API_RANGE_ERROR;
  }
  stepper_cfg[motor_id].param = param;
  return DEVICE_API_OK;
}

DeviceApiResult DeviceApi_StepperSetRpm(uint8_t motor_id, uint32_t rpm)
{
  StepperParam param;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  param = stepper_cfg[motor_id].param;
  param.rpm = rpm;
  if ((Stepper_CheckUserRange(motor_id, &param) != TRUE) ||
      (Stepper_CheckMotorEquivalentRange(motor_id, &param) != TRUE))
  {
    return DEVICE_API_RANGE_ERROR;
  }
  stepper_cfg[motor_id].param = param;
  return DEVICE_API_OK;
}

void DeviceApi_BindStepperRosCmd(uint8_t motor_id, uint32_t id, const char *cmd)
{
  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  devices[motor_id].has_ros_cmd = TRUE;
  devices[motor_id].ros_cmd_id = id;
  devices[motor_id].ros_cmd_name = cmd;
}

static void Stepper_Command(uint8_t device_id, int argc, char *argv[])
{
  uint8_t motor_id;
  uint32_t temp;
  int32_t temp_rev;
  uint8_t continuous;
  uint8_t dir;
  StepperCmdResult result;
  DeviceApiResult api_result;

  Stepper_ApplyMechanicalConfig();
  motor_id = device_id;

  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    return;
  }

  /* 查询类命令：打印该设备当前状态、默认参数和安全范围 */
  if (strcmp(argv[1], "status") == 0)
  {
    Device_PrintStatus(device_id);
    Stepper_PrintParam(motor_id);
    Stepper_PrintLimit(motor_id);
    return;
  }

  if (strcmp(argv[1], "stop") == 0)
  {
    if (argc != 2)
    {
      printf("\n%s stop param_error", devices[device_id].name);
      return;
    }

    Device_Task();
    result = Stepper_Stop(motor_id);
    devices[device_id].target = devices[device_id].position;
    devices[device_id].state = DEV_IDLE;
    Stepper_PrintStopResult(motor_id, result);
    return;
  }

  /* 运动类命令：更新目标参数，完成范围校验后再启动电机 */
  if (strcmp(argv[1], "move") == 0)
  {
    if ((argc != 3) || (Stepper_ParseMoveRev(argv[2], &temp_rev, &continuous, &dir) != TRUE))
    {
      printf("\n%s %s param_error", devices[device_id].name, argv[1]);
      return;
    }

    if (continuous == TRUE)
    {
      api_result = DeviceApi_StepperRun(motor_id, dir);
    }
    else
    {
      api_result = DeviceApi_StepperMove(motor_id, temp_rev);
    }

    if (api_result == DEVICE_API_RANGE_ERROR)
    {
      printf("\n%s param_range_error", devices[device_id].name);
      Stepper_PrintLimit(motor_id);
      return;
    }

    result = (api_result == DEVICE_API_OK) ? STEPPER_CMD_OK :
             (api_result == DEVICE_API_BUSY) ? STEPPER_CMD_BUSY :
             (api_result == DEVICE_API_DISABLED) ? STEPPER_CMD_DISABLED :
             (api_result == DEVICE_API_ID_ERROR) ? STEPPER_CMD_ID_ERROR :
             STEPPER_CMD_PARAM_ERROR;
    Stepper_PrintStartResult(motor_id, result);
    if ((api_result == DEVICE_API_OK) && (continuous == TRUE))
    {
      printf(" continuous dir=%s", (dir == CCW) ? "ccw" : "cw");
    }
    return;
  }

  /* 参数设置类命令：单独修改加速度、减速度或速度 */
  if ((strcmp(argv[1], "accel") == 0) ||
      (strcmp(argv[1], "decel") == 0) ||
      (strcmp(argv[1], "rpm") == 0))
  {
    if (argc != 3)
    {
      printf("\n%s %s param_error", devices[device_id].name, argv[1]);
      return;
    }

    temp = atoi(argv[2]);

    if (strcmp(argv[1], "accel") == 0)
    {
      stepper_cfg[motor_id].param.accel_rpm_s = temp;
      if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
          (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
      {
        printf("\n%s accel range_error", devices[device_id].name);
        Stepper_PrintLimit(motor_id);
        return;
      }
    }
    else if (strcmp(argv[1], "decel") == 0)
    {
      stepper_cfg[motor_id].param.decel_rpm_s = temp;
      if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
          (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
      {
        printf("\n%s decel range_error", devices[device_id].name);
        Stepper_PrintLimit(motor_id);
        return;
      }
    }
    else
    {
      stepper_cfg[motor_id].param.rpm = temp;
      if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
          (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
      {
        printf("\n%s rpm range_error", devices[device_id].name);
        Stepper_PrintLimit(motor_id);
        return;
      }
    }

    Stepper_PrintParam(motor_id);
    Stepper_PrintLimit(motor_id);
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
}

static uint8_t Clamp_ParseServoId(int argc, char *argv[], uint8_t default_id)
{
  int id;

  if (argc < 3)
  {
    return default_id;
  }

  id = atoi(argv[2]);
  if ((id < 0) || (id > 253))
  {
    return default_id;
  }

  return (uint8_t)id;
}

static void Clamp_PrintResult(const char *action, uint8_t servo_id,
                              BusServoResult result, uint8_t servo_error)
{
  printf("\nclamp %s id=%u result=%s",
         action,
         servo_id,
         BusServo_ResultName(result));
  if (servo_error != 0)
  {
    printf(" servo_err=0x%02X", servo_error);
  }
}

static uint8_t Clamp_SpeedValid(uint16_t speed)
{
  return ((speed >= GRIPPER_SPEED_MIN) && (speed <= GRIPPER_SPEED_MAX)) ? TRUE : FALSE;
}

static uint8_t Clamp_GripStepValid(uint16_t step)
{
  return ((step >= GRIPPER_GRIP_STEP_MIN) && (step <= GRIPPER_GRIP_STEP_MAX)) ? TRUE : FALSE;
}

static uint8_t Clamp_ReleaseDeltaValid(uint16_t delta)
{
  return ((delta >= GRIPPER_RELEASE_DELTA_MIN) && (delta <= GRIPPER_RELEASE_DELTA_MAX)) ? TRUE : FALSE;
}

static uint16_t Clamp_OpenPercentToPosition(uint8_t open_percentage)
{
  uint32_t span;

  span = (uint32_t)(GRIPPER_POS_CLOSE_MIN - GRIPPER_POS_OPEN_MAX);
  return (uint16_t)(GRIPPER_POS_CLOSE_MIN - ((span * open_percentage + 50) / 100));
}

static void Clamp_PrintParam(void)
{
  printf("\nclamp param speed=%u gripStep=%u releaseDelta=%u",
         clamp_param.speed,
         clamp_param.grip_step,
         clamp_param.release_delta);
}

static void Clamp_PrintStateBits(uint8_t state)
{
  if (state == 0)
  {
    return;
  }

  printf(" voltage_err=%u encoder_err=%u temp_err=%u current_err=%u load_err=%u",
         (state & 0x01) ? 1 : 0,
         (state & 0x02) ? 1 : 0,
         (state & 0x04) ? 1 : 0,
         (state & 0x08) ? 1 : 0,
         (state & 0x20) ? 1 : 0);
}

static void Clamp_PrintStatusFields(const BusServoStatus *status_data)
{
  int32_t current_ma_x10;
  int32_t current_ma_abs_x10;
  int32_t load_abs_x10;

  current_ma_x10 = (int32_t)status_data->current * 65;
  current_ma_abs_x10 = (current_ma_x10 < 0) ? -current_ma_x10 : current_ma_x10;
  load_abs_x10 = (status_data->load < 0) ? -(int32_t)status_data->load : (int32_t)status_data->load;
  printf(" pos=%d speed=%d load=%d(%s%ld.%ld%%) voltage=%u.%uV temp=%u current=%d(%s%ld.%ldmA) state=0x%02X",
         (int)status_data->position,
         (int)status_data->speed,
         (int)status_data->load,
         (status_data->load < 0) ? "-" : "",
         (long)(load_abs_x10 / 10),
         (long)(load_abs_x10 % 10),
         status_data->voltage / 10,
         status_data->voltage % 10,
         status_data->temperature,
         (int)status_data->current,
         (current_ma_x10 < 0) ? "-" : "",
         (long)(current_ma_abs_x10 / 10),
         (long)(current_ma_abs_x10 % 10),
         status_data->status);
  Clamp_PrintStateBits(status_data->status);
  printf("\n");
}

static void Clamp_PrintMoveResult(const char *action, uint8_t servo_id,
                                  uint16_t position, uint16_t speed,
                                  GripperResult result, BusServoResult servo_result,
                                  const BusServoStatus *status_data)
{
  printf("\nclamp %s id=%u target=%u speed=%u result=%s",
         action,
         servo_id,
         position,
         speed,
         Gripper_ResultName(result));
  if ((result == GRIPPER_SERVO_ERROR) ||
      (result == GRIPPER_TORQUE_OFF_ERROR) ||
      (result == GRIPPER_TORQUE_ON_ERROR) ||
      (result == GRIPPER_MOVE_ERROR) ||
      (result == GRIPPER_STATUS_ERROR))
  {
    printf(" servo_result=%s", BusServo_ResultName(servo_result));
  }
  if (status_data != 0)
  {
    Clamp_PrintStatusFields(status_data);
  }
  printf("\n");
}

DeviceApiResult DeviceApi_ClampStatus(uint8_t servo_id, DeviceClampStatus *out)
{
  uint8_t servo_error;
  BusServoResult result;
  BusServoStatus status_data;

  if (out == 0)
  {
    return DEVICE_API_PARAM_ERROR;
  }

  servo_error = 0;
  result = BusServo_ReadStatus(servo_id, &status_data, &servo_error);
  if (result != BUS_SERVO_OK)
  {
    devices[DEVICE_CLAMP].state = DEV_ERROR;
    devices[DEVICE_CLAMP].error_code = result;
    return DeviceApi_FromBusServoResult(result);
  }

  Clamp_FillExtraStatus(servo_id, &status_data);
  DeviceApi_FillClampStatus(servo_id, &status_data, out);
  devices[DEVICE_CLAMP].state = (clamp_hold.active == TRUE) ? DEV_RUNNING :
                                 ((status_data.moving != 0) ? DEV_RUNNING : DEV_IDLE);
  devices[DEVICE_CLAMP].position = status_data.position;
  devices[DEVICE_CLAMP].target = status_data.position;
  devices[DEVICE_CLAMP].value = status_data.status;
  devices[DEVICE_CLAMP].error_code = 0;
  return DEVICE_API_OK;
}

static DeviceApiResult DeviceApi_ClampMovePosition(uint16_t position, DeviceClampStatus *out)
{
  BusServoResult servo_result;
  BusServoStatus final_status;
  GripperResult gripper_result;
  DeviceApiResult ret;

  Clamp_HoldEnd();
  memset(&final_status, 0, sizeof(final_status));
  gripper_result = Gripper_MoveFeedback(GRIPPER_SERVO_ID_DEFAULT, position, clamp_param.speed,
                                        &final_status, &servo_result);
  ret = DeviceApi_FromGripperResult(gripper_result, servo_result);
  if (ret == DEVICE_API_OK)
  {
    DeviceApi_FillClampStatus(GRIPPER_SERVO_ID_DEFAULT, &final_status, out);
    devices[DEVICE_CLAMP].position = final_status.position;
    devices[DEVICE_CLAMP].target = position;
    devices[DEVICE_CLAMP].value = final_status.status;
    devices[DEVICE_CLAMP].error_code = 0;
    devices[DEVICE_CLAMP].state = DEV_IDLE;
  }
  else
  {
    devices[DEVICE_CLAMP].error_code = gripper_result;
    devices[DEVICE_CLAMP].state = DEV_ERROR;
  }
  return ret;
}

DeviceApiResult DeviceApi_ClampMove(uint8_t open_percentage, DeviceClampStatus *out)
{
  if (open_percentage > 100)
  {
    return DEVICE_API_RANGE_ERROR;
  }

  return DeviceApi_ClampMovePosition(Clamp_OpenPercentToPosition(open_percentage), out);
}

DeviceApiResult DeviceApi_ClampOpen(DeviceClampStatus *out)
{
  return DeviceApi_ClampMovePosition(GRIPPER_POS_OPEN_MAX, out);
}

DeviceApiResult DeviceApi_ClampClose(DeviceClampStatus *out)
{
  return DeviceApi_ClampMovePosition(GRIPPER_POS_CLOSE_MIN, out);
}

static DeviceApiResult DeviceApi_ClampGripInternal(uint16_t load, uint8_t has_open_percentage,
                                                   uint8_t open_percentage, DeviceClampStatus *out)
{
  BusServoResult servo_result;
  BusServoStatus final_status;
  GripperResult gripper_result;
  DeviceApiResult ret;

  memset(&final_status, 0, sizeof(final_status));
  gripper_result = Clamp_GripTwoStage(GRIPPER_SERVO_ID_DEFAULT, load,
                                      has_open_percentage, open_percentage,
                                      &final_status, &servo_result);
  ret = DeviceApi_FromGripperResult(gripper_result, servo_result);
  if (ret == DEVICE_API_OK)
  {
    DeviceApi_FillClampStatus(GRIPPER_SERVO_ID_DEFAULT, &final_status, out);
    devices[DEVICE_CLAMP].position = final_status.position;
    devices[DEVICE_CLAMP].target = final_status.position;
    devices[DEVICE_CLAMP].value = final_status.status;
    devices[DEVICE_CLAMP].error_code = 0;
    Clamp_HoldBegin(load, &final_status);
  }
  else
  {
    devices[DEVICE_CLAMP].error_code = gripper_result;
    devices[DEVICE_CLAMP].state = DEV_ERROR;
  }
  return ret;
}

DeviceApiResult DeviceApi_ClampRelease(DeviceClampStatus *out)
{
  int16_t cur_pos;
  uint16_t target_pos;
  BusServoResult servo_result;
  BusServoStatus final_status;
  GripperResult gripper_result;
  DeviceApiResult ret;

  cur_pos = 0;
  target_pos = 0;
  Clamp_HoldEnd();
  memset(&final_status, 0, sizeof(final_status));
  gripper_result = Gripper_Release(GRIPPER_SERVO_ID_DEFAULT, clamp_param.release_delta, clamp_param.speed,
                                   &cur_pos, &target_pos, &final_status, &servo_result);
  ret = DeviceApi_FromGripperResult(gripper_result, servo_result);
  if (ret == DEVICE_API_OK)
  {
    DeviceApi_FillClampStatus(GRIPPER_SERVO_ID_DEFAULT, &final_status, out);
    devices[DEVICE_CLAMP].position = final_status.position;
    devices[DEVICE_CLAMP].target = target_pos;
    devices[DEVICE_CLAMP].value = final_status.status;
    devices[DEVICE_CLAMP].error_code = 0;
    devices[DEVICE_CLAMP].state = DEV_IDLE;
  }
  else
  {
    devices[DEVICE_CLAMP].error_code = gripper_result;
    devices[DEVICE_CLAMP].state = DEV_ERROR;
  }
  return ret;
}

DeviceApiResult DeviceApi_ClampGrip(uint16_t load, DeviceClampStatus *out)
{
  return DeviceApi_ClampGripInternal(load, FALSE, 0, out);
}

DeviceApiResult DeviceApi_ClampGripAt(uint16_t load, uint8_t open_percentage, DeviceClampStatus *out)
{
  if (open_percentage > 100)
  {
    return DEVICE_API_RANGE_ERROR;
  }

  return DeviceApi_ClampGripInternal(load, TRUE, open_percentage, out);
}

DeviceApiResult DeviceApi_ClampSet(uint16_t speed, uint16_t grip_step, uint16_t release_delta)
{
  if ((Clamp_SpeedValid(speed) == FALSE) ||
      (Clamp_GripStepValid(grip_step) == FALSE) ||
      (Clamp_ReleaseDeltaValid(release_delta) == FALSE))
  {
    return DEVICE_API_RANGE_ERROR;
  }

  clamp_param.speed = speed;
  clamp_param.grip_step = grip_step;
  clamp_param.release_delta = release_delta;
  return DEVICE_API_OK;
}

static void Clamp_Command(uint8_t device_id, int argc, char *argv[])
{
  uint8_t servo_id;
  uint8_t servo_error;
  BusServoResult result;
  BusServoResult servo_result;
  BusServoStatus status_data;
  BusServoStatus final_status;
  GripperResult gripper_result;
  uint16_t position;
  uint16_t speed;
  uint16_t load_threshold;
  uint16_t step;
  uint16_t delta;
  uint16_t target_pos;
  int16_t cur_pos;
  int parsed;
  uint8_t reg_addr;
  uint8_t reg_value;
  uint8_t open_percentage;

  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    return;
  }

  memset(&final_status, 0, sizeof(final_status));
  servo_id = GRIPPER_SERVO_ID_DEFAULT;
  servo_error = 0;

  /* 基础通讯命令：优先支持 ping/status，便于现场快速检查接线和反馈。 */
  if (strcmp(argv[1], "ping") == 0)
  {
    servo_id = Clamp_ParseServoId(argc, argv, GRIPPER_SERVO_ID_DEFAULT);
    result = BusServo_Ping(servo_id, &servo_error);
    Clamp_PrintResult("ping", servo_id, result, servo_error);
    printf("\n");

    if (result == BUS_SERVO_OK)
    {
      devices[device_id].state = DEV_IDLE;
      devices[device_id].error_code = 0;
    }
    else
    {
      devices[device_id].state = DEV_ERROR;
      devices[device_id].error_code = result;
    }
    return;
  }

  if (strcmp(argv[1], "status") == 0)
  {
    servo_id = Clamp_ParseServoId(argc, argv, GRIPPER_SERVO_ID_DEFAULT);
    result = BusServo_ReadStatus(servo_id, &status_data, &servo_error);
    Clamp_PrintResult("status", servo_id, result, servo_error);

    if (result == BUS_SERVO_OK)
    {
      Clamp_FillExtraStatus(servo_id, &status_data);
      devices[device_id].state = (clamp_hold.active == TRUE) ? DEV_RUNNING :
                                 ((status_data.moving != 0) ? DEV_RUNNING : DEV_IDLE);
      devices[device_id].position = status_data.position;
      devices[device_id].value = status_data.status;
      devices[device_id].error_code = 0;

      Clamp_PrintStatusFields(&status_data);
      printf("\n");
    }
    else
    {
      devices[device_id].state = DEV_ERROR;
      devices[device_id].error_code = result;
      printf("\n");
    }
    return;
  }

  /* 位置类命令：open/close/move 都走反馈式运动，并在持续堵转时由底层卸力停机。 */
  if (strcmp(argv[1], "readreg") == 0)
  {
    if (argc != 3)
    {
      printf("\nclamp readreg param_error");
      return;
    }

    parsed = atoi(argv[2]);
    if ((parsed < 0) || (parsed > 255))
    {
      printf("\nclamp readreg range_error");
      return;
    }

    reg_addr = (uint8_t)parsed;
    reg_value = 0;
    result = BusServo_ReadData(servo_id, reg_addr, 1, &reg_value, &servo_error);
    printf("\nclamp readreg id=%u addr=%u(0x%02X) result=%s",
           servo_id,
           reg_addr,
           reg_addr,
           BusServo_ResultName(result));
    if (result == BUS_SERVO_OK)
    {
      printf(" value=%u(0x%02X)", reg_value, reg_value);
    }
    if (servo_error != 0)
    {
      printf(" servo_err=0x%02X", servo_error);
    }
    printf("\n");
    return;
  }

  if (strcmp(argv[1], "set") == 0)
  {
    if (argc != 5)
    {
      printf("\nclamp set param_error");
      return;
    }

    speed = (uint16_t)atoi(argv[2]);
    step = (uint16_t)atoi(argv[3]);
    delta = (uint16_t)atoi(argv[4]);
    if (DeviceApi_ClampSet(speed, step, delta) != DEVICE_API_OK)
    {
      printf("\nclamp set range_error");
      return;
    }

    Clamp_PrintParam();
    printf("\n");
    return;
  }

  if (strcmp(argv[1], "open") == 0)
  {
    if (argc != 2)
    {
      printf("\nclamp open param_error");
      return;
    }

    Clamp_HoldEnd();
    gripper_result = Gripper_MoveFeedback(servo_id, GRIPPER_POS_OPEN_MAX, clamp_param.speed, &final_status, &servo_result);
    Clamp_PrintMoveResult("open", servo_id, GRIPPER_POS_OPEN_MAX, clamp_param.speed, gripper_result, servo_result, &final_status);

    devices[device_id].target = GRIPPER_POS_OPEN_MAX;
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  if (strcmp(argv[1], "close") == 0)
  {
    if (argc != 2)
    {
      printf("\nclamp close param_error");
      return;
    }

    Clamp_HoldEnd();
    gripper_result = Gripper_MoveFeedback(servo_id, GRIPPER_POS_CLOSE_MIN, clamp_param.speed, &final_status, &servo_result);
    Clamp_PrintMoveResult("close", servo_id, GRIPPER_POS_CLOSE_MIN, clamp_param.speed, gripper_result, servo_result, &final_status);

    devices[device_id].target = GRIPPER_POS_CLOSE_MIN;
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  if (strcmp(argv[1], "move") == 0)
  {
    if (argc != 3)
    {
      printf("\nclamp move param_error");
      return;
    }

    parsed = atoi(argv[2]);
    if ((parsed < 0) || (parsed > 100))
    {
      printf("\nclamp move range_error");
      return;
    }

    open_percentage = (uint8_t)parsed;
    position = Clamp_OpenPercentToPosition(open_percentage);
    Clamp_HoldEnd();
    gripper_result = Gripper_MoveFeedback(servo_id, position, clamp_param.speed, &final_status, &servo_result);
    printf("\nclamp move openPercentage=%u", open_percentage);
    Clamp_PrintMoveResult("move", servo_id, position, clamp_param.speed, gripper_result, servo_result, &final_status);

    devices[device_id].target = position;
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  /* 夹持类命令：grip 通过逐步闭合和负载阈值判断建立夹持，不主动卸力。 */
  if (strcmp(argv[1], "grip") == 0)
  {
    if ((argc != 3) && (argc != 4))
    {
      printf("\nclamp grip param_error");
      return;
    }

    load_threshold = (uint16_t)atoi(argv[2]);
    open_percentage = 0;
    if (argc == 4)
    {
      parsed = atoi(argv[3]);
      if ((parsed < 0) || (parsed > 100))
      {
        printf("\nclamp grip range_error");
        return;
      }
      open_percentage = (uint8_t)parsed;
    }

    gripper_result = Clamp_GripTwoStage(servo_id, load_threshold,
                                        (argc == 4) ? TRUE : FALSE,
                                        open_percentage,
                                        &final_status, &servo_result);
    printf("\nclamp grip id=%u load=%u",
           servo_id,
           load_threshold);
    if (argc == 4)
    {
      printf(" openPercentage=%u", open_percentage);
    }
    printf(" speed=%u fineStep=%u preload=%u%% result=%s",
           clamp_param.speed,
           CLAMP_GRIP_FINE_STEP,
           CLAMP_GRIP_PRELOAD_PERCENT,
           Gripper_ResultName(gripper_result));
    if ((gripper_result == GRIPPER_MOVE_ERROR) || (gripper_result == GRIPPER_STATUS_ERROR))
    {
      printf(" servo_result=%s", BusServo_ResultName(servo_result));
    }
    if (gripper_result != GRIPPER_RANGE_ERROR)
    {
      Clamp_PrintStatusFields(&final_status);
    }

    if (gripper_result != GRIPPER_RANGE_ERROR)
    {
      devices[device_id].position = final_status.position;
      devices[device_id].target = final_status.position;
    }
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    if (gripper_result == GRIPPER_OK)
    {
      Clamp_HoldBegin(load_threshold, &final_status);
    }
    else
    {
      devices[device_id].state = DEV_ERROR;
    }
    return;
  }

  /* 松开类命令：release 先卸力、读位置，再重新上扭矩退开一点。 */
  if (strcmp(argv[1], "release") == 0)
  {
    if (argc != 2)
    {
      printf("\nclamp release param_error");
      return;
    }

    cur_pos = 0;
    target_pos = 0;

    Clamp_HoldEnd();
    gripper_result = Gripper_Release(servo_id, clamp_param.release_delta, clamp_param.speed, &cur_pos, &target_pos, &final_status, &servo_result);
    printf("\nclamp release id=%u cur=%d target=%u delta=%u speed=%u result=%s",
           servo_id,
           (int)cur_pos,
           target_pos,
           clamp_param.release_delta,
           clamp_param.speed,
           Gripper_ResultName(gripper_result));
    if ((gripper_result == GRIPPER_MOVE_ERROR) ||
        (gripper_result == GRIPPER_STATUS_ERROR) ||
        (gripper_result == GRIPPER_TORQUE_OFF_ERROR) ||
        (gripper_result == GRIPPER_TORQUE_ON_ERROR))
    {
      printf(" servo_result=%s", BusServo_ResultName(servo_result));
    }
    if ((gripper_result == GRIPPER_OK) ||
        (gripper_result == GRIPPER_STALL) ||
        (gripper_result == GRIPPER_TIMEOUT))
    {
      Clamp_PrintStatusFields(&final_status);
    }

    if (gripper_result != GRIPPER_RANGE_ERROR)
    {
      devices[device_id].position = final_status.position;
      devices[device_id].target = target_pos;
    }
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
}

static void Vacum_PrintStatusFields(const Evs08Status *status_data)
{
  printf(" state1=0x%04X state2=0x%04X fault=0x%04X busy1=%u busy2=%u obj1=%u obj2=%u vac1=%u%% vac2=%u%% temp=%u bus=%u.%uV",
         status_data->ch1_status_reg,
         status_data->ch2_status_reg,
         status_data->fault_reg,
         status_data->ch1_busy,
         status_data->ch2_busy,
         status_data->ch1_obj,
         status_data->ch2_obj,
         status_data->ch1_vac_percent,
         status_data->ch2_vac_percent,
         status_data->temperature,
         status_data->bus_voltage_x10 / 10,
         status_data->bus_voltage_x10 % 10);
}

static uint8_t Vacum_ParsePercent(char *text, uint8_t *value)
{
  int parsed;

  if ((text == 0) || (value == 0))
  {
    return FALSE;
  }

  parsed = atoi(text);
  if ((parsed < 0) || (parsed > 100))
  {
    return FALSE;
  }

  *value = (uint8_t)parsed;
  return TRUE;
}

static uint8_t Vacum_ParseTimeout(char *text, uint8_t *value)
{
  int parsed;

  if ((text == 0) || (value == 0))
  {
    return FALSE;
  }

  parsed = atoi(text);
  if ((parsed < 1) || (parsed > 255))
  {
    return FALSE;
  }

  *value = (uint8_t)parsed;
  return TRUE;
}

static void Vacum_PrintResult(const char *action, ModbusResult result)
{
  printf("\nvacum %s result=%s", action, Evs08_ResultName(result));
}

DeviceApiResult DeviceApi_VacumSet(uint8_t min_vac, uint8_t max_vac, uint8_t timeout)
{
  ModbusResult result;

  if ((min_vac > 100) || (max_vac > 100) || (min_vac > max_vac) ||
      (timeout == 0))
  {
    return DEVICE_API_RANGE_ERROR;
  }

  result = Evs08_SetParams(min_vac, max_vac, timeout);
  devices[DEVICE_VACUM].state = (result == MODBUS_OK) ? DEV_IDLE : DEV_ERROR;
  devices[DEVICE_VACUM].error_code = (result == MODBUS_OK) ? 0 : result;
  return DeviceApi_FromModbusResult(result);
}

DeviceApiResult DeviceApi_VacumGrip(void)
{
  ModbusResult result;

  result = Evs08_Grip();
  devices[DEVICE_VACUM].state = (result == MODBUS_OK) ? DEV_RUNNING : DEV_ERROR;
  devices[DEVICE_VACUM].error_code = (result == MODBUS_OK) ? 0 : result;
  if (result == MODBUS_OK)
  {
    Vacum_HoldBegin();
  }
  return DeviceApi_FromModbusResult(result);
}

DeviceApiResult DeviceApi_VacumRelease(void)
{
  ModbusResult result;

  result = Evs08_Release();
  devices[DEVICE_VACUM].state = (result == MODBUS_OK) ? DEV_IDLE : DEV_ERROR;
  devices[DEVICE_VACUM].error_code = (result == MODBUS_OK) ? 0 : result;
  if (result == MODBUS_OK)
  {
    Vacum_HoldEnd();
  }
  return DeviceApi_FromModbusResult(result);
}

DeviceApiResult DeviceApi_VacumStop(void)
{
  ModbusResult result;

  result = Evs08_Stop();
  devices[DEVICE_VACUM].state = (result == MODBUS_OK) ? DEV_IDLE : DEV_ERROR;
  devices[DEVICE_VACUM].error_code = (result == MODBUS_OK) ? 0 : result;
  if (result == MODBUS_OK)
  {
    Vacum_HoldEnd();
  }
  return DeviceApi_FromModbusResult(result);
}

DeviceApiResult DeviceApi_VacumStatus(DeviceVacumStatus *out)
{
  ModbusResult result;
  Evs08Status status_data;

  if (out == 0)
  {
    return DEVICE_API_PARAM_ERROR;
  }

  result = Evs08_ReadStatus(&status_data);
  if (result != MODBUS_OK)
  {
    devices[DEVICE_VACUM].state = DEV_ERROR;
    devices[DEVICE_VACUM].error_code = result;
    return DeviceApi_FromModbusResult(result);
  }

  DeviceApi_FillVacumStatus(&status_data, out);
  devices[DEVICE_VACUM].enabled = (status_data.ch1_enabled || status_data.ch2_enabled) ? TRUE : FALSE;
  devices[DEVICE_VACUM].value = status_data.ch1_status_reg;
  devices[DEVICE_VACUM].error_code = 0;
  devices[DEVICE_VACUM].state = (vacum_hold.active == TRUE) ? DEV_RUNNING :
                                ((status_data.ch1_busy || status_data.ch2_busy) ? DEV_RUNNING : DEV_IDLE);
  return DEVICE_API_OK;
}

static void Vacum_Command(uint8_t device_id, int argc, char *argv[])
{
  uint8_t max_vac;
  uint8_t min_vac;
  uint8_t timeout;
  ModbusResult result;
  Evs08Status status_data;

  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    return;
  }

  if (strcmp(argv[1], "set") == 0)
  {
    if (argc != 5)
    {
      printf("\nvacum set param_error");
      return;
    }

    if ((Vacum_ParsePercent(argv[2], &min_vac) != TRUE) ||
        (Vacum_ParsePercent(argv[3], &max_vac) != TRUE) ||
        (Vacum_ParseTimeout(argv[4], &timeout) != TRUE))
    {
      printf("\nvacum set range_error min/max=0~100 timeout=1~255(100ms)");
      return;
    }

    result = Evs08_SetParams(min_vac, max_vac, timeout);
    Vacum_PrintResult("set", result);
    if (result == MODBUS_OK)
    {
      printf(" min=%u%% max=%u%% timeout=%u(100ms)", min_vac, max_vac, timeout);
      devices[device_id].state = DEV_IDLE;
      devices[device_id].error_code = 0;
    }
    else
    {
      devices[device_id].state = DEV_ERROR;
      devices[device_id].error_code = result;
    }
    return;
  }

  if (strcmp(argv[1], "grip") == 0)
  {
    if (argc != 2)
    {
      printf("\nvacum grip param_error");
      return;
    }

    result = Evs08_Grip();
    Vacum_PrintResult("grip", result);
    devices[device_id].state = (result == MODBUS_OK) ? DEV_RUNNING : DEV_ERROR;
    devices[device_id].error_code = (result == MODBUS_OK) ? 0 : result;
    if (result == MODBUS_OK)
    {
      Vacum_HoldBegin();
    }
    return;
  }

  if (strcmp(argv[1], "release") == 0)
  {
    if (argc != 2)
    {
      printf("\nvacum release param_error");
      return;
    }

    result = Evs08_Release();
    Vacum_PrintResult("release", result);
    devices[device_id].state = (result == MODBUS_OK) ? DEV_IDLE : DEV_ERROR;
    devices[device_id].error_code = (result == MODBUS_OK) ? 0 : result;
    if (result == MODBUS_OK)
    {
      Vacum_HoldEnd();
    }
    return;
  }

  if (strcmp(argv[1], "stop") == 0)
  {
    if (argc != 2)
    {
      printf("\nvacum stop param_error");
      return;
    }

    result = Evs08_Stop();
    Vacum_PrintResult("stop", result);
    devices[device_id].state = (result == MODBUS_OK) ? DEV_IDLE : DEV_ERROR;
    devices[device_id].error_code = (result == MODBUS_OK) ? 0 : result;
    if (result == MODBUS_OK)
    {
      Vacum_HoldEnd();
    }
    return;
  }

  if (strcmp(argv[1], "status") == 0)
  {
    if (argc != 2)
    {
      printf("\nvacum status param_error");
      return;
    }

    result = Evs08_ReadStatus(&status_data);
    Vacum_PrintResult("status", result);
    if (result == MODBUS_OK)
    {
      Vacum_PrintStatusFields(&status_data);
      devices[device_id].enabled = (status_data.ch1_enabled || status_data.ch2_enabled) ? TRUE : FALSE;
      devices[device_id].value = status_data.ch1_status_reg;
      devices[device_id].error_code = 0;
      devices[device_id].state = (vacum_hold.active == TRUE) ? DEV_RUNNING :
                                  ((status_data.ch1_busy || status_data.ch2_busy) ? DEV_RUNNING : DEV_IDLE);
    }
    else
    {
      devices[device_id].state = DEV_ERROR;
      devices[device_id].error_code = result;
    }
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
}

static void Device_PrintFullStatus(void)
{
  char *mtor1_status[] = {"mtor1", "status"};
  char *mtor2_status[] = {"mtor2", "status"};
  char *clamp_status[] = {"clamp", "status"};
  char *vacum_status[] = {"vacum", "status"};

  Stepper_Command(DEVICE_MTOR1, 2, mtor1_status);
  Stepper_Command(DEVICE_MTOR2, 2, mtor2_status);
  Clamp_Command(DEVICE_CLAMP, 2, clamp_status);
  Vacum_Command(DEVICE_VACUM, 2, vacum_status);
  printf("\n");
}

static void Command_Dispatch(char *line)
{
  char *argv[8];
  int argc;
  int device_id;
  char *token;
  uint32_t monitor_interval;

  argc = 0;
  token = strtok(line, " \r\n");
  while ((token != 0) && (argc < 8))
  {
    argv[argc++] = token;
    token = strtok(0, " \r\n");
  }

  if (argc == 0)
  {
    return;
  }

  /* 特殊入口命令：? 打印帮助，status 打印全部设备状态。 */
  if (strcmp(argv[0], "?") == 0)
  {
    ShowCommandHelp();
    return;
  }

  if (strcmp(argv[0], "status") == 0)
  {
    Device_PrintFullStatus();
    return;
  }

  if (strcmp(argv[0], "monitor") == 0)
  {
    if (argc == 1)
    {
      printf("\nmonitor interval=%lums fast_check=%ums fault_hits=%u",
             (unsigned long)device_monitor_interval_ms,
             DEVICE_FAST_CHECK_INTERVAL_MS,
             DEVICE_STALL_HIT_LIMIT);
      return;
    }
    if (argc != 2)
    {
      printf("\nmonitor param_error");
      return;
    }

    monitor_interval = (uint32_t)atoi(argv[1]);
    if ((monitor_interval < DEVICE_MONITOR_INTERVAL_MIN_MS) ||
        (monitor_interval > DEVICE_MONITOR_INTERVAL_MAX_MS))
    {
      printf("\nmonitor range_error intervalMs=1000~60000");
      return;
    }

    device_monitor_interval_ms = monitor_interval;
    printf("\nmonitor interval=%lums fast_check=%ums fault_hits=%u",
           (unsigned long)device_monitor_interval_ms,
           DEVICE_FAST_CHECK_INTERVAL_MS,
           DEVICE_STALL_HIT_LIMIT);
    return;
  }

  device_id = Device_FindByName(argv[0]);
  if (device_id < 0)
  {
    printf("\nunknown device: %s", argv[0]);
    return;
  }

  /* 统一按设备类型分发到对应处理函数，后续新增末端设备时从这里扩展。 */
  switch (devices[device_id].type)
  {
    case DEVICE_TYPE_STEPPER:
      Stepper_Command((uint8_t)device_id, argc, argv);
      break;

    case DEVICE_TYPE_SERVO:
      Clamp_Command((uint8_t)device_id, argc, argv);
      break;

    case DEVICE_TYPE_VACUM:
      Vacum_Command((uint8_t)device_id, argc, argv);
      break;

    default:
      printf("\n%s type_error", devices[device_id].name);
      break;
  }
}

/**
  * @brief  处理串口接收到的数据，只分发任务，不等待设备完成
  * @param  无
  * @retval 无
  */
void DealSerialData(void)
{
  static char showflag = 1;

  Stepper_ApplyMechanicalConfig();

  if (showflag)
  {
    showflag = 0;
    return;
  }

  if (status.cmd == TRUE)
  {
    if (RosProtocol_TryDispatch((char *)UART_RxBuffer) != TRUE)
    {
      Command_Dispatch((char *)UART_RxBuffer);
    }
    status.cmd = FALSE;

    uart_FlushRxBuffer();
  }
}
