/**
  ******************************************************************************
  * @file    bsp_stepper_usart_ctl.c
  * @brief   双电机名称式串口任务分发（按设备输出轴机械单位输入）
  ******************************************************************************
  */

#include "./stepper/bsp_stepper_usart_ctl.h"

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
} EndDevice;

#define INPUT_REV_SCALE                10
#define MTOR_MAX_OUTPUT_REV_0P1        100000

static EndDevice devices[DEVICE_NUM] = {
  {DEVICE_MTOR1, "mtor1", DEVICE_TYPE_STEPPER, DEV_IDLE,      TRUE, 0, 0, 0, 0},
  {DEVICE_MTOR2, "mtor2", DEVICE_TYPE_STEPPER, DEV_IDLE,      TRUE, 0, 0, 0, 0},
  {DEVICE_CLAMP, "clamp", DEVICE_TYPE_SERVO,   DEV_NOT_READY, TRUE, 0, 0, 0, 0},
  {DEVICE_VACUM, "vacum", DEVICE_TYPE_VACUM,   DEV_NOT_READY, TRUE, 0, 0, 0, 0}
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

static int Device_FindByName(const char *name)
{
  uint8_t i;

  for (i = 0; i < STEPPER_NUM; i++)
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

void Device_Task(void)
{
  uint8_t i;

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
}

void Device_ReportDone(void)
{
  uint8_t i;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    if (devices[i].state == DEV_DONE)
    {
      printf("\n\r%s done rev=", devices[i].name);
      PrintFixed1Signed(devices[i].position);
      devices[i].state = DEV_IDLE;
    }
  }
}

static void Device_PrintStatus(uint8_t id)
{
  Device_Task();

  if (id >= DEVICE_NUM)
  {
    return;
  }

  printf("\n\r%s enabled=%d rev=", devices[id].name, devices[id].enabled);
  PrintFixed1Signed(devices[id].position);
  printf(" target=");
  PrintFixed1Signed(devices[id].target);
  printf(" err=%lu", (unsigned long)devices[id].error_code);
}

static void Device_PrintAllStatus(void)
{
  uint8_t i;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    Device_PrintStatus(i);
  }
  printf("\n\r");
}

static void Stepper_PrintDefault(uint8_t motor_id)
{
  printf("\n\r%s default rev=", devices[motor_id].name);
  PrintFixed1Signed(stepper_cfg[motor_id].param.rev_0p1);
  printf(" accel=%lurpm/s decel=%lurpm/s rpm=%lurpm",
         (unsigned long)stepper_cfg[motor_id].param.accel_rpm_s,
         (unsigned long)stepper_cfg[motor_id].param.decel_rpm_s,
         (unsigned long)stepper_cfg[motor_id].param.rpm);
}

static void Stepper_PrintLimit(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;

  printf("\n\r%s limit rev=", devices[motor_id].name);
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
  printf("\n\r命令说明:");
  printf("\n\r  mtor1 turn [rev]");
  printf("\n\r  mtor1 move [rev] [accel] [decel] [rpm]");
  printf("\n\r  mtor1 accel [value]");
  printf("\n\r  mtor1 decel [value]");
  printf("\n\r  mtor1 rpm [value]");
  printf("\n\r  mtor2 turn [rev]");
  printf("\n\r  mtor2 move [rev] [accel] [decel] [rpm]");
  printf("\n\r  mtor2 accel [value]");
  printf("\n\r  mtor2 decel [value]");
  printf("\n\r  mtor2 rpm [value]");
  printf("\n\r  status  查看全部设备状态");
  printf("\n\r示例: mtor1 move 50 100 100 100  -> 5圈, 100rpm/s, 100rpm/s, 100rpm");
  printf("\n\r");
}

void ShowHelp(void)
{
  Stepper_ApplyMechanicalConfig();

  printf("\n\r——————————————双电机参数概览——————————————");
  printf("\n\r设备:");
  printf("\n\r  mtor1 : 电机0 ENA PE0, DIR PE1, PUL PI5(TIM8_CH1)");
  printf("\n\r  mtor2 : 电机1 ENA PE4, DIR PI8, PUL PI6(TIM8_CH2)");
  printf("\n\r单位说明:");
  printf("\n\r  1rev = 0.1圈，默认值为50（5圈），范围为-10000~10000（-1000圈~1000圈）");
  printf("\n\r  1accel = 1rpm/s，默认值为100，范围为60~500");
  printf("\n\r  1decel = 1rpm/s，默认值为100，范围为60~500");
  printf("\n\r  1rpm = 1rpm，默认值为100，范围为1~600");
  printf("\n\r输入 ? 查看命令说明，输入 status 查看全部设备状态\n\r");
}

static void Stepper_PrintParam(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  mech = &stepper_cfg[motor_id].mech;

  printf("\n\r%s param rev=", devices[motor_id].name);
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
  printf("\n\r%s %s", devices[motor_id].name, Stepper_ResultName(result));
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

static void Stepper_Command(uint8_t device_id, int argc, char *argv[])
{
  uint8_t motor_id;
  uint32_t temp;
  int32_t temp_rev;
  int32_t step_target;
  uint32_t accel_internal;
  uint32_t decel_internal;
  uint32_t speed_internal;
  StepperCmdResult result;

  Stepper_ApplyMechanicalConfig();
  motor_id = device_id;

  if (argc < 2)
  {
    printf("\n\r%s missing action", devices[device_id].name);
    return;
  }

  /* 查询类命令：统一打印全部设备状态 */
  if (strcmp(argv[1], "status") == 0)
  {
    Device_PrintStatus(device_id);
    Stepper_PrintParam(motor_id);
    Stepper_PrintLimit(motor_id);
    return;
  }

  /* 运动类命令：更新目标参数，完成范围校验后启动电机 */
  if ((strcmp(argv[1], "turn") == 0) || (strcmp(argv[1], "move") == 0))
  {
    if ((strcmp(argv[1], "turn") == 0 && argc != 3) ||
        (strcmp(argv[1], "move") == 0 && argc != 6))
    {
      printf("\n\r%s %s param_error", devices[device_id].name, argv[1]);
      return;
    }

    temp_rev = atoi(argv[2]);
    stepper_cfg[motor_id].param.rev_0p1 = temp_rev;

    if (strcmp(argv[1], "move") == 0)
    {
      stepper_cfg[motor_id].param.accel_rpm_s = atoi(argv[3]);
      stepper_cfg[motor_id].param.decel_rpm_s = atoi(argv[4]);
      stepper_cfg[motor_id].param.rpm = atoi(argv[5]);
    }

    if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
        (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
    {
      printf("\n\r%s param_range_error", devices[device_id].name);
      Stepper_PrintLimit(motor_id);
      return;
    }

    step_target = Stepper_RevToStep(motor_id, stepper_cfg[motor_id].param.rev_0p1);
    accel_internal = Stepper_RpmPerSecToAccel(motor_id, stepper_cfg[motor_id].param.accel_rpm_s);
    decel_internal = Stepper_RpmPerSecToAccel(motor_id, stepper_cfg[motor_id].param.decel_rpm_s);
    speed_internal = Stepper_RpmToSpeed(motor_id, stepper_cfg[motor_id].param.rpm);

    devices[device_id].target = stepper_cfg[motor_id].param.rev_0p1;
    result = stepper_move_T(motor_id,
                            step_target,
                            accel_internal,
                            decel_internal,
                            speed_internal);
    if (result == STEPPER_CMD_OK)
    {
      devices[device_id].state = DEV_RUNNING;
    }
    Stepper_PrintStartResult(motor_id, result);
    return;
  }

  /* 参数设置类命令：单独修改加速度、减速度或速度 */
  if ((strcmp(argv[1], "accel") == 0) ||
      (strcmp(argv[1], "decel") == 0) ||
      (strcmp(argv[1], "rpm") == 0))
  {
    if (argc != 3)
    {
      printf("\n\r%s %s param_error", devices[device_id].name, argv[1]);
      return;
    }

    temp = atoi(argv[2]);

    if (strcmp(argv[1], "accel") == 0)
    {
      stepper_cfg[motor_id].param.accel_rpm_s = temp;
      if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
          (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
      {
        printf("\n\r%s accel range_error", devices[device_id].name);
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
        printf("\n\r%s decel range_error", devices[device_id].name);
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
        printf("\n\r%s rpm range_error", devices[device_id].name);
        Stepper_PrintLimit(motor_id);
        return;
      }
    }

    Stepper_PrintParam(motor_id);
    Stepper_PrintLimit(motor_id);
    return;
  }

  printf("\n\r%s unknown action: %s", devices[device_id].name, argv[1]);
}

static void Placeholder_Command(uint8_t device_id, int argc, char *argv[])
{
  if ((argc >= 2) && (strcmp(argv[1], "status") == 0))
  {
    Device_PrintStatus(device_id);
    return;
  }

  printf("\n\r%s not_ready", devices[device_id].name);
}

static void Command_Dispatch(char *line)
{
  char *argv[8];
  int argc;
  int device_id;
  char *token;

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

  if (strcmp(argv[0], "?") == 0)
  {
    ShowCommandHelp();
    return;
  }

  if (strcmp(argv[0], "status") == 0)
  {
    Device_PrintAllStatus();
    Stepper_PrintDefault(STEPPER_MOTOR_0);
    Stepper_PrintDefault(STEPPER_MOTOR_1);
    Stepper_PrintLimit(STEPPER_MOTOR_0);
    Stepper_PrintLimit(STEPPER_MOTOR_1);
    return;
  }

  device_id = Device_FindByName(argv[0]);
  if (device_id < 0)
  {
    printf("\n\runknown device: %s", argv[0]);
    return;
  }

  switch (devices[device_id].type)
  {
    case DEVICE_TYPE_STEPPER:
      Stepper_Command((uint8_t)device_id, argc, argv);
      break;

    case DEVICE_TYPE_SERVO:
    case DEVICE_TYPE_VACUM:
      Placeholder_Command((uint8_t)device_id, argc, argv);
      break;

    default:
      printf("\n\r%s type_error", devices[device_id].name);
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
    Command_Dispatch((char *)UART_RxBuffer);
    status.cmd = FALSE;
    uart_FlushRxBuffer();
  }
}
