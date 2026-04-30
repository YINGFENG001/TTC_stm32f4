/**
  ******************************************************************************
 * @file    bsp_device_usart_ctl.c
 * @brief   多设备名称式串口命令分发（按设备输出轴机械单位输入）
  ******************************************************************************
  */

#include "./stepper/bsp_device_usart_ctl.h"
#include "./gripper/bsp_bus_servo.h"
#include "./gripper/bsp_gripper.h"

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

static void Clamp_PrintStatusFields(const BusServoStatus *status_data);

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

      printf("\n\rclamp status id=%u result=ok", GRIPPER_SERVO_ID_DEFAULT);
      Clamp_PrintStatusFields(&clamp_status);
    }
    else
    {
      devices[id].state = DEV_ERROR;
      devices[id].error_code = clamp_result;
      printf("\n\rclamp status id=%u result=%s",
             GRIPPER_SERVO_ID_DEFAULT,
             BusServo_ResultName(clamp_result));
      if (servo_error != 0)
      {
        printf(" servo_err=0x%02X", servo_error);
      }
    }
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

  for (i = 0; i < DEVICE_NUM; i++)
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
  printf("\n\r  mtor1 turn [rev: -10000~10000(0.1圈)]");
  printf("\n\r  mtor1 move [rev: -10000~10000(0.1圈)] [accel: 60~500(rpm/s)] [decel: 60~500(rpm/s)] [rpm: 1~600]");
  printf("\n\r  mtor1 accel [value: 60~500(rpm/s)]");
  printf("\n\r  mtor1 decel [value: 60~500(rpm/s)]");
  printf("\n\r  mtor1 rpm [value: 1~600]");
  printf("\n\r  mtor2 turn [rev: -10000~10000(0.1圈)]");
  printf("\n\r  mtor2 move [rev: -10000~10000(0.1圈)] [accel: 60~500(rpm/s)] [decel: 60~500(rpm/s)] [rpm: 1~600]");
  printf("\n\r  mtor2 accel [value: 60~500(rpm/s)]");
  printf("\n\r  mtor2 decel [value: 60~500(rpm/s)]");
  printf("\n\r  mtor2 rpm [value: 1~600]");
  printf("\n\r  clamp ping [id: 0~253]");
  printf("\n\r  clamp status [id: 0~253]");
  printf("\n\r  clamp open [speed: 1~3000]");
  printf("\n\r  clamp close [speed: 1~3000]");
  printf("\n\r  clamp move [position: 800~2048] [speed: 1~3000]");
  printf("\n\r  clamp grip [load: 100~900(0.1%%)] [speed: 1~3000] [step: 5~100]");
  printf("\n\r  clamp release [delta: 20~400] [speed: 100~1000]");
  printf("\n\r  status");
  printf("\n\r示例: mtor1 move 50 100 100 100  -> 5.0圈, 100rpm/s, 100rpm/s, 100rpm");
  printf("\n\r");
}

void ShowHelp(void)
{
  Stepper_ApplyMechanicalConfig();

  printf("\n\r================ 设备概览 ================");
  printf("\n\r设备:");
  printf("\n\r  mtor1 : 电机0 ENA PE0, DIR PE1, PUL PI5(TIM8_CH1)");
  printf("\n\r  mtor2 : 电机1 ENA PE4, DIR PI8, PUL PI6(TIM8_CH2)");
  printf("\n\r  clamp : 夹爪舵机 UART5 TX PC12, RX PD2, 默认 id=10");
  printf("\n\r单位说明:");
  printf("\n\r  mtor rev   = 0.1圈，默认 50 = 5.0圈，范围 -10000~10000");
  printf("\n\r  mtor accel = 1rpm/s，默认 100，范围 60~500");
  printf("\n\r  mtor decel = 1rpm/s，默认 100，范围 60~500");
  printf("\n\r  mtor rpm   = 1rpm，默认 100，范围 1~600");
  printf("\n\r  clamp position = 800~2048");
  printf("\n\r  clamp load     = 0.1%%");
  printf("\n\r  clamp current  = 6.5mA");
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

  /* 查询类命令：打印该设备当前状态、默认参数和安全范围 */
  if (strcmp(argv[1], "status") == 0)
  {
    Device_PrintStatus(device_id);
    Stepper_PrintParam(motor_id);
    Stepper_PrintLimit(motor_id);
    return;
  }

  /* 运动类命令：更新目标参数，完成范围校验后再启动电机 */
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
  printf("\n\rclamp %s id=%u result=%s",
         action,
         servo_id,
         BusServo_ResultName(result));
  if (servo_error != 0)
  {
    printf(" servo_err=0x%02X", servo_error);
  }
}

static uint16_t Clamp_ParseSpeed(int argc, char *argv[], int index)
{
  int speed;

  if (argc <= index)
  {
    return GRIPPER_SPEED_DEFAULT;
  }

  speed = atoi(argv[index]);
  if (speed < 0)
  {
    return 0;
  }

  return (uint16_t)speed;
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
  printf("\n\r");
}

static void Clamp_PrintMoveResult(const char *action, uint8_t servo_id,
                                  uint16_t position, uint16_t speed,
                                  GripperResult result, BusServoResult servo_result,
                                  const BusServoStatus *status_data)
{
  printf("\n\rclamp %s id=%u target=%u speed=%u result=%s",
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
  printf("\n\r");
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

  if (argc < 2)
  {
    printf("\n\r%s missing action", devices[device_id].name);
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
    printf("\n\r");

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
      devices[device_id].state = (status_data.moving != 0) ? DEV_RUNNING : DEV_IDLE;
      devices[device_id].position = status_data.position;
      devices[device_id].value = status_data.status;
      devices[device_id].error_code = 0;

      Clamp_PrintStatusFields(&status_data);
      printf("\n\r");
    }
    else
    {
      devices[device_id].state = DEV_ERROR;
      devices[device_id].error_code = result;
      printf("\n\r");
    }
    return;
  }

  /* 位置类命令：open/close/move 都走反馈式运动，并在持续堵转时由底层卸力停机。 */
  if (strcmp(argv[1], "open") == 0)
  {
    speed = Clamp_ParseSpeed(argc, argv, 2);
    gripper_result = Gripper_MoveFeedback(servo_id, GRIPPER_POS_OPEN_MAX, speed, &final_status, &servo_result);
    Clamp_PrintMoveResult("open", servo_id, GRIPPER_POS_OPEN_MAX, speed, gripper_result, servo_result, &final_status);

    devices[device_id].target = GRIPPER_POS_OPEN_MAX;
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  if (strcmp(argv[1], "close") == 0)
  {
    speed = Clamp_ParseSpeed(argc, argv, 2);
    gripper_result = Gripper_MoveFeedback(servo_id, GRIPPER_POS_CLOSE_MIN, speed, &final_status, &servo_result);
    Clamp_PrintMoveResult("close", servo_id, GRIPPER_POS_CLOSE_MIN, speed, gripper_result, servo_result, &final_status);

    devices[device_id].target = GRIPPER_POS_CLOSE_MIN;
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  if (strcmp(argv[1], "move") == 0)
  {
    if (argc < 3)
    {
      printf("\n\rclamp move param_error");
      return;
    }

    if (atoi(argv[2]) < 0)
    {
      printf("\n\rclamp move range_error");
      return;
    }

    position = (uint16_t)atoi(argv[2]);
    speed = Clamp_ParseSpeed(argc, argv, 3);
    gripper_result = Gripper_MoveFeedback(servo_id, position, speed, &final_status, &servo_result);
    Clamp_PrintMoveResult("move", servo_id, position, speed, gripper_result, servo_result, &final_status);

    devices[device_id].target = position;
    devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  /* 夹持类命令：grip 通过逐步闭合和负载阈值判断建立夹持，不主动卸力。 */
  if (strcmp(argv[1], "grip") == 0)
  {
    load_threshold = (argc >= 3) ? (uint16_t)atoi(argv[2]) : GRIPPER_GRIP_LOAD_DEFAULT;
    speed = Clamp_ParseSpeed(argc, argv, 3);
    step = (argc >= 5) ? (uint16_t)atoi(argv[4]) : GRIPPER_GRIP_STEP_DEFAULT;

    gripper_result = Gripper_Grip(servo_id, load_threshold, speed, step, &final_status, &servo_result);
    printf("\n\rclamp grip id=%u load=%u speed=%u step=%u result=%s",
           servo_id,
           load_threshold,
           speed,
           step,
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
    devices[device_id].state = (gripper_result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
    return;
  }

  /* 松开类命令：release 先卸力、读位置，再重新上扭矩退开一点。 */
  if (strcmp(argv[1], "release") == 0)
  {
    delta = (argc >= 3) ? (uint16_t)atoi(argv[2]) : GRIPPER_RELEASE_DELTA_DEFAULT;
    speed = (argc >= 4) ? (uint16_t)atoi(argv[3]) : GRIPPER_RELEASE_SPEED_DEFAULT;
    cur_pos = 0;
    target_pos = 0;

    gripper_result = Gripper_Release(servo_id, delta, speed, &cur_pos, &target_pos, &final_status, &servo_result);
    printf("\n\rclamp release id=%u cur=%d target=%u delta=%u speed=%u result=%s",
           servo_id,
           (int)cur_pos,
           target_pos,
           delta,
           speed,
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

  /* 特殊入口命令：? 打印帮助，status 打印全部设备状态。 */
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
