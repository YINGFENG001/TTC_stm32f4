/**
  ******************************************************************************
 * @file    bsp_stepper_ctl.c
 * @brief   Stepper device command and API implementation.
  ******************************************************************************
  */

#include "./stepper/bsp_stepper_ctl.h"

typedef struct {
  uint16_t motor_steps_per_rev;
  uint16_t micro_step;
  uint16_t gear_num;
  uint16_t gear_den;
} StepperMechanicalConfig;

typedef struct {
  int32_t min_rev_0p1;
  int32_t max_rev_0p1;
  uint32_t min_rpm;
  uint32_t max_rpm;
  uint32_t min_accel_rpm_s;
  uint32_t max_accel_rpm_s;
  uint32_t min_decel_rpm_s;
  uint32_t max_decel_rpm_s;
  uint32_t max_motor_rpm;
} StepperSafetyLimit;

typedef struct {
  int32_t rev_0p1;
  uint32_t accel_rpm_s;
  uint32_t decel_rpm_s;
  uint32_t rpm;
} StepperParam;

typedef struct {
  StepperMechanicalConfig mech;
  StepperSafetyLimit limit;
  StepperParam param;
} StepperDeviceConfig;

#define INPUT_REV_SCALE                10
#define MTOR_MAX_OUTPUT_REV_0P1        100000

static StepperDeviceConfig stepper_cfg[STEPPER_NUM] = {
  {
    {200, 8, 1, 1},
    {-MTOR_MAX_OUTPUT_REV_0P1, MTOR_MAX_OUTPUT_REV_0P1, 1, 600, 60, 500, 60, 500, 600},
    {50, 100, 100, 100}
  },
  {
    {200, 8, 20, 1},
    {-MTOR_MAX_OUTPUT_REV_0P1, MTOR_MAX_OUTPUT_REV_0P1, 1, 30, 60, 500, 60, 500, 600},
    {50, 100, 100, 30}
  }
};

static const char *Stepper_ResultName(StepperCmdResult result);
static uint32_t Stepper_GetMotorPulsesPerRev(uint8_t motor_id);
static int32_t Stepper_GetPulsesPerOutputRev(uint8_t motor_id);
static uint32_t Stepper_OutputRpmToMotorRpm(uint8_t motor_id, uint32_t rpm);
static uint32_t Stepper_OutputAccelToMotorAccel(uint8_t motor_id, uint32_t accel_rpm_s);
static int32_t Stepper_RevToStep(uint8_t motor_id, int32_t rev_0p1);
static uint32_t Stepper_RpmToSpeed(uint8_t motor_id, uint32_t rpm);
static uint32_t Stepper_RpmPerSecToAccel(uint8_t motor_id, uint32_t accel_rpm_s);
static uint32_t Stepper_CalcInitialStepDelay(uint8_t motor_id, uint32_t accel_internal);
static uint8_t Stepper_ParseMoveRev(const char *text, int32_t *rev_0p1, uint8_t *continuous, uint8_t *dir);
static void Stepper_PrintLimit(uint8_t motor_id);
static void Stepper_PrintParam(uint8_t motor_id);
static void Stepper_PrintStartResult(uint8_t motor_id, StepperCmdResult result);
static void Stepper_PrintStopResult(uint8_t motor_id, StepperCmdResult result);
static uint8_t Stepper_CheckUserRange(uint8_t motor_id, const StepperParam *param);
static uint8_t Stepper_CheckMotorEquivalentRange(uint8_t motor_id, const StepperParam *param);
static DeviceApiResult DeviceApi_FromStepperResult(StepperCmdResult result);
static DeviceApiResult DeviceApi_StepperStart(uint8_t motor_id, const StepperParam *param);

void Stepper_ApplyMechanicalConfig(void)
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

int32_t Stepper_StepToRev0p1(uint8_t motor_id, int32_t step)
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
 * 5. mtor1 为 200步/圈、8细分、1:1直驱；mtor2 为 200步/圈、8细分、20:1减速机
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

static void Stepper_CommandStatus(uint8_t motor_id)
{
  Device_PrintStatus(motor_id);
  Stepper_PrintParam(motor_id);
  Stepper_PrintLimit(motor_id);
}

static void Stepper_CommandStop(uint8_t motor_id, int argc)
{
  StepperCmdResult result;

  if (argc != 2)
  {
    printf("\n%s stop param_error", devices[motor_id].name);
    return;
  }

  Device_Task();
  result = Stepper_Stop(motor_id);
  devices[motor_id].target = devices[motor_id].position;
  devices[motor_id].state = DEV_IDLE;
  Stepper_PrintStopResult(motor_id, result);
}

static StepperCmdResult Stepper_ApiResultToCmdResult(DeviceApiResult api_result)
{
  if (api_result == DEVICE_API_OK)
  {
    return STEPPER_CMD_OK;
  }
  if (api_result == DEVICE_API_BUSY)
  {
    return STEPPER_CMD_BUSY;
  }
  if (api_result == DEVICE_API_DISABLED)
  {
    return STEPPER_CMD_DISABLED;
  }
  if (api_result == DEVICE_API_ID_ERROR)
  {
    return STEPPER_CMD_ID_ERROR;
  }
  return STEPPER_CMD_PARAM_ERROR;
}

static void Stepper_CommandMove(uint8_t motor_id, int argc, char *argv[])
{
  int32_t temp_rev;
  uint8_t continuous;
  uint8_t dir;
  DeviceApiResult api_result;

  if ((argc != 3) || (Stepper_ParseMoveRev(argv[2], &temp_rev, &continuous, &dir) != TRUE))
  {
    printf("\n%s %s param_error", devices[motor_id].name, argv[1]);
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
    printf("\n%s param_range_error", devices[motor_id].name);
    Stepper_PrintLimit(motor_id);
    return;
  }

  Stepper_PrintStartResult(motor_id, Stepper_ApiResultToCmdResult(api_result));
  if ((api_result == DEVICE_API_OK) && (continuous == TRUE))
  {
    printf(" continuous dir=%s", (dir == CCW) ? "ccw" : "cw");
  }
}

static void Stepper_CommandSetParam(uint8_t motor_id, int argc, char *argv[])
{
  uint32_t temp;

  if (argc != 3)
  {
    printf("\n%s %s param_error", devices[motor_id].name, argv[1]);
    return;
  }

  temp = atoi(argv[2]);
  if (strcmp(argv[1], "accel") == 0)
  {
    stepper_cfg[motor_id].param.accel_rpm_s = temp;
    if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
        (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
    {
      printf("\n%s accel range_error", devices[motor_id].name);
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
      printf("\n%s decel range_error", devices[motor_id].name);
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
      printf("\n%s rpm range_error", devices[motor_id].name);
      Stepper_PrintLimit(motor_id);
      return;
    }
  }

  Stepper_PrintParam(motor_id);
  Stepper_PrintLimit(motor_id);
}

void Stepper_Command(uint8_t device_id, int argc, char *argv[])
{
  Stepper_ApplyMechanicalConfig();

  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    return;
  }
  if (strcmp(argv[1], "status") == 0)
  {
    Stepper_CommandStatus(device_id);
    return;
  }
  if (strcmp(argv[1], "stop") == 0)
  {
    Stepper_CommandStop(device_id, argc);
    return;
  }
  if (strcmp(argv[1], "move") == 0)
  {
    Stepper_CommandMove(device_id, argc, argv);
    return;
  }
  if ((strcmp(argv[1], "accel") == 0) ||
      (strcmp(argv[1], "decel") == 0) ||
      (strcmp(argv[1], "rpm") == 0))
  {
    Stepper_CommandSetParam(device_id, argc, argv);
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
}
