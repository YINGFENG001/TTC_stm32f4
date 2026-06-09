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
#define MTOR_MIN_MICRO_STEP            1
#define MTOR_MAX_MICRO_STEP            256
#define MTOR_MIN_GEAR_NUM              1
#define MTOR_MAX_GEAR_NUM              1000
#define MTOR_MIN_GEAR_DEN              1
#define MTOR_MAX_GEAR_DEN              1000
#define MTOR_MIN_MOTOR_ACCEL_RPM_S     40
#define MTOR_MAX_MOTOR_ACCEL_RPM_S     1000
#define MTOR_MAX_MOTOR_RPM             1500

static StepperDeviceConfig stepper_cfg[STEPPER_NUM] = {
  {
    {200, 4, 1, 1},
    {-MTOR_MAX_OUTPUT_REV_0P1, MTOR_MAX_OUTPUT_REV_0P1, 1, MTOR_MAX_MOTOR_RPM, MTOR_MIN_MOTOR_ACCEL_RPM_S, MTOR_MAX_MOTOR_ACCEL_RPM_S, MTOR_MIN_MOTOR_ACCEL_RPM_S, MTOR_MAX_MOTOR_ACCEL_RPM_S, MTOR_MAX_MOTOR_RPM},
    {50, 400, 400, 200}
  },
  {
    {200, 4, 20, 1},
    {-MTOR_MAX_OUTPUT_REV_0P1, MTOR_MAX_OUTPUT_REV_0P1, 1, MTOR_MAX_MOTOR_RPM, MTOR_MIN_MOTOR_ACCEL_RPM_S, MTOR_MAX_MOTOR_ACCEL_RPM_S, MTOR_MIN_MOTOR_ACCEL_RPM_S, MTOR_MAX_MOTOR_ACCEL_RPM_S, MTOR_MAX_MOTOR_RPM},
    {50, 30, 30, 30}
  }
};

static const char *Stepper_ResultName(StepperCmdResult result);
static const char *Stepper_DirName(uint8_t dir);
static uint32_t Stepper_GetMotorPulsesPerRev(uint8_t motor_id);
static int32_t Stepper_GetPulsesPerOutputRev(uint8_t motor_id);
static uint32_t Stepper_OutputRpmToMotorRpm(uint8_t motor_id, uint32_t rpm);
static uint32_t Stepper_GetEffectiveMaxRpm(uint8_t motor_id);
static uint32_t Stepper_GetEffectiveMinAccel(uint8_t motor_id);
static uint32_t Stepper_GetEffectiveMaxAccel(uint8_t motor_id);
static uint32_t Stepper_OutputAccelToMotorAccel(uint8_t motor_id, uint32_t accel_rpm_s);
static int32_t Stepper_RevToStep(uint8_t motor_id, int32_t rev_0p1);
static uint32_t Stepper_RpmToSpeed(uint8_t motor_id, uint32_t rpm);
static uint32_t Stepper_RpmPerSecToAccel(uint8_t motor_id, uint32_t accel_rpm_s);
static uint32_t Stepper_CalcInitialStepDelay(uint8_t motor_id, uint32_t accel_internal);
static uint8_t Stepper_ParseMoveRev(const char *text, int32_t *rev_0p1, uint8_t *continuous, uint8_t *dir);
static uint8_t Stepper_ParseGear(const char *text, uint16_t *gear_num, uint16_t *gear_den);
static void Stepper_UpdateMotorPulses(uint8_t motor_id);
static void Stepper_PrintLimit(uint8_t motor_id);
static void Stepper_PrintSetting(uint8_t motor_id);
static void Stepper_PrintRpmSettingValue(uint8_t motor_id, int32_t rpm);
static void Stepper_PrintStartResult(uint8_t motor_id, StepperCmdResult result);
static void Stepper_PrintStopResult(uint8_t motor_id, StepperCmdResult result);
static void Stepper_PrintCommandNewline(void);
static uint8_t Stepper_CheckUserRange(uint8_t motor_id, const StepperParam *param);
static uint8_t Stepper_CheckMotorEquivalentRange(uint8_t motor_id, const StepperParam *param);
static DeviceApiResult DeviceApi_FromStepperResult(StepperCmdResult result);
static DeviceApiResult DeviceApi_StepperStart(uint8_t motor_id, const StepperParam *param);

void Stepper_ApplyMechanicalConfig(void)
{
  static uint8_t applied = FALSE;
  uint8_t i;

  if (applied == TRUE)
  {
    return;
  }

  for (i = 0; i < STEPPER_NUM; i++)
  {
    Stepper_UpdateMotorPulses(i);
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

static const char *Stepper_DirName(uint8_t dir)
{
  return (dir == CCW) ? "CCW" : "CW";
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
  return (uint32_t)(((rpm * mech->gear_num) + mech->gear_den - 1U) / mech->gear_den);
}

static uint32_t Stepper_GetEffectiveMaxRpm(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;
  uint32_t motor_limited_rpm;

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;
  motor_limited_rpm = (uint32_t)((limit->max_motor_rpm * mech->gear_den) / mech->gear_num);
  if (motor_limited_rpm == 0)
  {
    motor_limited_rpm = 1;
  }
  if (motor_limited_rpm > limit->max_rpm)
  {
    return limit->max_rpm;
  }
  return motor_limited_rpm;
}

static uint32_t Stepper_GetEffectiveMinAccel(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;
  uint32_t min_accel;

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;
  min_accel = (uint32_t)(((limit->min_accel_rpm_s * mech->gear_den) + mech->gear_num - 1U) / mech->gear_num);
  if (min_accel == 0)
  {
    min_accel = 1;
  }
  return min_accel;
}

static uint32_t Stepper_GetEffectiveMaxAccel(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;
  uint32_t max_accel;

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;
  max_accel = (uint32_t)((limit->max_accel_rpm_s * mech->gear_den) / mech->gear_num);
  if (max_accel == 0)
  {
    max_accel = 1;
  }
  return max_accel;
}

static uint32_t Stepper_OutputAccelToMotorAccel(uint8_t motor_id, uint32_t accel_rpm_s)
{
  const StepperMechanicalConfig *mech;

  mech = &stepper_cfg[motor_id].mech;
  return (uint32_t)(((accel_rpm_s * mech->gear_num) + mech->gear_den - 1U) / mech->gear_den);
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
 * 3. accel_rpm_s  : rpm/s（按设备输出轴计算，内部按减速比换算到电机轴）
 * 4. decel_rpm_s  : rpm/s（按设备输出轴计算，内部按减速比换算到电机轴）
 *
 * 当前硬件前提：
 * 1. 系统时钟 SystemCoreClock = 168MHz
 * 2. TIM8 采用输出比较 Toggle 模式输出步进脉冲
 * 3. TIM_PRESCALER = 31，定时器计数频率 = 168MHz / (31 + 1) = 5.25MHz
 * 4. TIM_PERIOD = 0xFFFF，当前使用16位计数器
 * 5. mtor1 为 200步/圈、4细分、1:1直驱；mtor2 为 200步/圈、4细分、20:1减速机
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

static uint8_t Stepper_ParseGear(const char *text, uint16_t *gear_num, uint16_t *gear_den)
{
  const char *colon;
  uint32_t num;
  uint32_t den;

  if ((text == 0) || (gear_num == 0) || (gear_den == 0))
  {
    return FALSE;
  }

  colon = strchr(text, ':');
  if (colon == 0)
  {
    return FALSE;
  }

  num = (uint32_t)atoi(text);
  den = (uint32_t)atoi(colon + 1);
  if ((num < MTOR_MIN_GEAR_NUM) || (num > MTOR_MAX_GEAR_NUM) ||
      (den < MTOR_MIN_GEAR_DEN) || (den > MTOR_MAX_GEAR_DEN))
  {
    return FALSE;
  }

  *gear_num = (uint16_t)num;
  *gear_den = (uint16_t)den;
  return TRUE;
}

static void Stepper_UpdateMotorPulses(uint8_t motor_id)
{
  uint32_t pulses_per_motor_rev;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  pulses_per_motor_rev = (uint32_t)stepper_cfg[motor_id].mech.motor_steps_per_rev *
                         stepper_cfg[motor_id].mech.micro_step;
  Stepper_SetMotorPulsesPerRev(motor_id, pulses_per_motor_rev);
}

static void Stepper_PrintLimit(uint8_t motor_id)
{
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;
  uint32_t max_rpm;
  uint32_t min_accel;
  uint32_t max_accel;

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;
  max_rpm = Stepper_GetEffectiveMaxRpm(motor_id);
  min_accel = Stepper_GetEffectiveMinAccel(motor_id);
  max_accel = Stepper_GetEffectiveMaxAccel(motor_id);

  printf("\n%s limit rev=", devices[motor_id].name);
  PrintFixed1Signed(limit->min_rev_0p1);
  printf("~");
  PrintFixed1Signed(limit->max_rev_0p1);
  printf(" rpm=%lu~%lu accel=%lu~%lu decel=%lu~%lu gear=%u:%u micro=%u motor_rpm_max=%lu motor_accel=%lu~%lu",
         (unsigned long)limit->min_rpm,
         (unsigned long)max_rpm,
         (unsigned long)min_accel,
         (unsigned long)max_accel,
         (unsigned long)min_accel,
         (unsigned long)max_accel,
         mech->gear_num,
         mech->gear_den,
         mech->micro_step,
         (unsigned long)limit->max_motor_rpm,
         (unsigned long)limit->min_accel_rpm_s,
         (unsigned long)limit->max_accel_rpm_s);
}

static void Stepper_PrintSetting(uint8_t motor_id)
{
  Stepper_PrintRpmSettingValue(motor_id, (int32_t)stepper_cfg[motor_id].param.rpm);
}

static void Stepper_PrintRpmSettingValue(uint8_t motor_id, int32_t rpm)
{
  const StepperSafetyLimit *limit;
  uint32_t max_rpm;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  limit = &stepper_cfg[motor_id].limit;
  max_rpm = Stepper_GetEffectiveMaxRpm(motor_id);

  printf("\n%s setting rpm=%ld(%lu~%lu rpm)",
         devices[motor_id].name,
         (long)rpm,
         (unsigned long)limit->min_rpm,
         (unsigned long)max_rpm);
}

static void Stepper_PrintStartResult(uint8_t motor_id, StepperCmdResult result)
{
  printf("\n%s %s", devices[motor_id].name, Stepper_ResultName(result));
  if (result == STEPPER_CMD_OK)
  {
    printf(" rev=");
    PrintFixed1Signed(stepper_cfg[motor_id].param.rev_0p1);
    printf(" rpm=%lu",
           (unsigned long)stepper_cfg[motor_id].param.rpm);
  }
}

static void Stepper_PrintStopResult(uint8_t motor_id, StepperCmdResult result)
{
  printf("\n%s stop %s rev=", devices[motor_id].name, Stepper_ResultName(result));
  PrintFixed1Signed(devices[motor_id].position);
}

static void Stepper_PrintCommandNewline(void)
{
  printf("\n");
}

static uint8_t Stepper_CheckUserRange(uint8_t motor_id, const StepperParam *param)
{
  const StepperSafetyLimit *limit;
  uint32_t max_rpm;
  uint32_t min_accel;
  uint32_t max_accel;

  limit = &stepper_cfg[motor_id].limit;
  max_rpm = Stepper_GetEffectiveMaxRpm(motor_id);
  min_accel = Stepper_GetEffectiveMinAccel(motor_id);
  max_accel = Stepper_GetEffectiveMaxAccel(motor_id);

  if ((param->rev_0p1 < limit->min_rev_0p1) || (param->rev_0p1 > limit->max_rev_0p1))
  {
    return FALSE;
  }
  if ((param->accel_rpm_s < min_accel) || (param->accel_rpm_s > max_accel))
  {
    return FALSE;
  }
  if ((param->decel_rpm_s < min_accel) || (param->decel_rpm_s > max_accel))
  {
    return FALSE;
  }
  if ((param->rpm < limit->min_rpm) || (param->rpm > max_rpm))
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

  limit = &stepper_cfg[motor_id].limit;
  motor_rpm = Stepper_OutputRpmToMotorRpm(motor_id, param->rpm);

  if (motor_rpm > limit->max_motor_rpm)
  {
    return FALSE;
  }

  accel_internal = Stepper_RpmPerSecToAccel(motor_id, param->accel_rpm_s);
  decel_internal = Stepper_RpmPerSecToAccel(motor_id, param->decel_rpm_s);
  if ((Stepper_CalcInitialStepDelay(motor_id, accel_internal) == 0) ||
      (Stepper_CalcInitialStepDelay(motor_id, decel_internal) == 0))
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
  out->gear_num = stepper_cfg[motor_id].mech.gear_num;
  out->gear_den = stepper_cfg[motor_id].mech.gear_den;
  out->micro = stepper_cfg[motor_id].mech.micro_step;
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

  Stepper_ApplyMechanicalConfig();

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
  StepperRuntimeSnapshot snapshot;
  uint32_t accel_internal;
  uint32_t decel_internal;
  uint32_t speed_internal;
  StepperCmdResult result;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  if (Stepper_GetRuntimeSnapshot(motor_id, &snapshot) != TRUE)
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

  if (snapshot.running == TRUE)
  {
    accel_internal = Stepper_RpmPerSecToAccel(motor_id, param.accel_rpm_s);
    decel_internal = Stepper_RpmPerSecToAccel(motor_id, param.decel_rpm_s);
    speed_internal = Stepper_RpmToSpeed(motor_id, param.rpm);
    result = Stepper_UpdateRunningSpeed(motor_id, accel_internal, decel_internal, speed_internal);
    if (result != STEPPER_CMD_OK)
    {
      return DeviceApi_FromStepperResult(result);
    }
  }

  stepper_cfg[motor_id].param = param;
  return DEVICE_API_OK;
}

DeviceApiResult DeviceApi_StepperSetSignedRpm(uint8_t motor_id, int32_t rpm)
{
  StepperParam param;
  StepperRuntimeSnapshot snapshot;
  uint32_t abs_rpm;
  uint32_t accel_internal;
  uint32_t decel_internal;
  uint32_t speed_internal;
  uint8_t dir;
  uint8_t stop;
  StepperCmdResult result;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  if (Stepper_GetRuntimeSnapshot(motor_id, &snapshot) != TRUE)
  {
    return DEVICE_API_PARAM_ERROR;
  }

  if ((snapshot.running != TRUE) ||
      (snapshot.continuous != TRUE))
  {
    if (rpm <= 0)
    {
      return DEVICE_API_RANGE_ERROR;
    }
    return DeviceApi_StepperSetRpm(motor_id, (uint32_t)rpm);
  }

  stop = (rpm == 0) ? TRUE : FALSE;
  if (stop == TRUE)
  {
    abs_rpm = 0;
    dir = snapshot.dir;
  }
  else if (rpm < 0)
  {
    abs_rpm = (uint32_t)(-rpm);
    dir = CCW;
  }
  else
  {
    abs_rpm = (uint32_t)rpm;
    dir = CW;
  }

  Stepper_ApplyMechanicalConfig();

  param = stepper_cfg[motor_id].param;
  if (stop != TRUE)
  {
    param.rpm = abs_rpm;
  }

  if (stop == TRUE)
  {
    if ((param.accel_rpm_s < Stepper_GetEffectiveMinAccel(motor_id)) ||
        (param.accel_rpm_s > Stepper_GetEffectiveMaxAccel(motor_id)) ||
        (param.decel_rpm_s < Stepper_GetEffectiveMinAccel(motor_id)) ||
        (param.decel_rpm_s > Stepper_GetEffectiveMaxAccel(motor_id)))
    {
      return DEVICE_API_RANGE_ERROR;
    }
  }
  else if ((Stepper_CheckUserRange(motor_id, &param) != TRUE) ||
           (Stepper_CheckMotorEquivalentRange(motor_id, &param) != TRUE))
  {
    return DEVICE_API_RANGE_ERROR;
  }

  accel_internal = Stepper_RpmPerSecToAccel(motor_id, param.accel_rpm_s);
  decel_internal = Stepper_RpmPerSecToAccel(motor_id, param.decel_rpm_s);
  speed_internal = (stop == TRUE) ? 0U : Stepper_RpmToSpeed(motor_id, param.rpm);
  result = Stepper_UpdateContinuousSpeedSigned(motor_id,
                                               dir,
                                               stop,
                                               accel_internal,
                                               decel_internal,
                                               speed_internal);
  if (result != STEPPER_CMD_OK)
  {
    return DeviceApi_FromStepperResult(result);
  }

  if (stop != TRUE)
  {
    stepper_cfg[motor_id].param = param;
  }
  return DEVICE_API_OK;
}

DeviceApiResult DeviceApi_StepperSet(uint8_t motor_id, uint32_t accel, uint32_t decel,
                                     uint16_t gear_num, uint16_t gear_den,
                                     uint16_t micro)
{
  StepperDeviceConfig old_cfg;
  StepperRuntimeSnapshot snapshot;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return DEVICE_API_ID_ERROR;
  }

  if ((micro < MTOR_MIN_MICRO_STEP) || (micro > MTOR_MAX_MICRO_STEP) ||
      (gear_num < MTOR_MIN_GEAR_NUM) || (gear_num > MTOR_MAX_GEAR_NUM) ||
      (gear_den < MTOR_MIN_GEAR_DEN) || (gear_den > MTOR_MAX_GEAR_DEN))
  {
    return DEVICE_API_PARAM_ERROR;
  }

  if (Stepper_GetRuntimeSnapshot(motor_id, &snapshot) != TRUE)
  {
    return DEVICE_API_PARAM_ERROR;
  }

  if (snapshot.running == TRUE)
  {
    return DEVICE_API_BUSY;
  }

  old_cfg = stepper_cfg[motor_id];
  stepper_cfg[motor_id].param.accel_rpm_s = accel;
  stepper_cfg[motor_id].param.decel_rpm_s = decel;
  stepper_cfg[motor_id].mech.gear_num = gear_num;
  stepper_cfg[motor_id].mech.gear_den = gear_den;
  stepper_cfg[motor_id].mech.micro_step = micro;

  if ((Stepper_CheckUserRange(motor_id, &stepper_cfg[motor_id].param) != TRUE) ||
      (Stepper_CheckMotorEquivalentRange(motor_id, &stepper_cfg[motor_id].param) != TRUE))
  {
    stepper_cfg[motor_id] = old_cfg;
    return DEVICE_API_RANGE_ERROR;
  }

  Stepper_UpdateMotorPulses(motor_id);
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
  const StepperMechanicalConfig *mech;
  const StepperSafetyLimit *limit;
  uint32_t max_rpm;
  uint32_t min_accel;
  uint32_t max_accel;

  Device_Task();

  mech = &stepper_cfg[motor_id].mech;
  limit = &stepper_cfg[motor_id].limit;
  max_rpm = Stepper_GetEffectiveMaxRpm(motor_id);
  min_accel = Stepper_GetEffectiveMinAccel(motor_id);
  max_accel = Stepper_GetEffectiveMaxAccel(motor_id);

  printf("\n%s enabled=%d err=%lu rev=",
         devices[motor_id].name,
         devices[motor_id].enabled,
         (unsigned long)devices[motor_id].error_code);
  PrintFixed1Signed(devices[motor_id].position);
  printf("(");
  PrintFixed1Signed(limit->min_rev_0p1);
  printf("~");
  PrintFixed1Signed(limit->max_rev_0p1);
  printf(") accel=%lu(%lu~%lu) decel=%lu(%lu~%lu) rpm=%lu(%lu~%lu) gear=%u:%u micro=%u",
         (unsigned long)stepper_cfg[motor_id].param.accel_rpm_s,
         (unsigned long)min_accel,
         (unsigned long)max_accel,
         (unsigned long)stepper_cfg[motor_id].param.decel_rpm_s,
         (unsigned long)min_accel,
         (unsigned long)max_accel,
         (unsigned long)stepper_cfg[motor_id].param.rpm,
         (unsigned long)limit->min_rpm,
         (unsigned long)max_rpm,
         mech->gear_num,
         mech->gear_den,
         mech->micro_step);
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
  if (api_result == DEVICE_API_OK)
  {
    if (continuous == TRUE)
    {
      printf(" continuous");
    }
    printf(" dir=%s", Stepper_DirName(dir));
  }
}

static void Stepper_CommandRpm(uint8_t motor_id, int argc, char *argv[])
{
  int32_t temp;
  DeviceApiResult api_result;

  if (argc != 3)
  {
    printf("\n%s rpm param_error", devices[motor_id].name);
    return;
  }

  temp = atoi(argv[2]);
  api_result = DeviceApi_StepperSetSignedRpm(motor_id, temp);
  if (api_result == DEVICE_API_RANGE_ERROR)
  {
    printf("\n%s rpm range_error", devices[motor_id].name);
    Stepper_PrintSetting(motor_id);
    return;
  }
  if (api_result == DEVICE_API_BUSY)
  {
    printf("\n%s rpm busy", devices[motor_id].name);
    Stepper_PrintSetting(motor_id);
    return;
  }
  if (api_result != DEVICE_API_OK)
  {
    printf("\n%s rpm param_error", devices[motor_id].name);
    Stepper_PrintSetting(motor_id);
    return;
  }

  if (temp == 0)
  {
    Stepper_PrintSetting(motor_id);
    printf(" dir=STOP");
  }
  else
  {
    Stepper_PrintRpmSettingValue(motor_id, (temp < 0) ? -temp : temp);
    printf(" dir=%s", Stepper_DirName((temp < 0) ? CCW : CW));
  }
}

static void Stepper_CommandSet(uint8_t motor_id, int argc, char *argv[])
{
  uint32_t accel;
  uint32_t decel;
  uint32_t micro;
  uint16_t gear_num;
  uint16_t gear_den;
  DeviceApiResult api_result;

  if (argc != 6)
  {
    printf("\n%s set param_error", devices[motor_id].name);
    return;
  }

  accel = (uint32_t)atoi(argv[2]);
  decel = (uint32_t)atoi(argv[3]);
  micro = (uint32_t)atoi(argv[5]);
  if ((Stepper_ParseGear(argv[4], &gear_num, &gear_den) != TRUE) ||
      (micro < MTOR_MIN_MICRO_STEP) || (micro > MTOR_MAX_MICRO_STEP))
  {
    printf("\n%s set param_error", devices[motor_id].name);
    Stepper_PrintSetting(motor_id);
    return;
  }

  api_result = DeviceApi_StepperSet(motor_id, accel, decel, gear_num, gear_den, (uint16_t)micro);
  if (api_result == DEVICE_API_BUSY)
  {
    printf("\n%s set busy", devices[motor_id].name);
    return;
  }
  if (api_result == DEVICE_API_RANGE_ERROR)
  {
    printf("\n%s set range_error", devices[motor_id].name);
    Stepper_PrintSetting(motor_id);
    return;
  }
  if (api_result != DEVICE_API_OK)
  {
    printf("\n%s set param_error", devices[motor_id].name);
    Stepper_PrintSetting(motor_id);
    return;
  }

  Stepper_PrintSetting(motor_id);
}

void Stepper_Command(uint8_t device_id, int argc, char *argv[])
{
  Stepper_ApplyMechanicalConfig();

  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    Stepper_PrintCommandNewline();
    return;
  }
  if (strcmp(argv[1], "status") == 0)
  {
    Stepper_CommandStatus(device_id);
    Stepper_PrintCommandNewline();
    return;
  }
  if (strcmp(argv[1], "stop") == 0)
  {
    Stepper_CommandStop(device_id, argc);
    Stepper_PrintCommandNewline();
    return;
  }
  if (strcmp(argv[1], "move") == 0)
  {
    Stepper_CommandMove(device_id, argc, argv);
    Stepper_PrintCommandNewline();
    return;
  }
  if (strcmp(argv[1], "rpm") == 0)
  {
    Stepper_CommandRpm(device_id, argc, argv);
    Stepper_PrintCommandNewline();
    return;
  }
  if (strcmp(argv[1], "set") == 0)
  {
    Stepper_CommandSet(device_id, argc, argv);
    Stepper_PrintCommandNewline();
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
  Stepper_PrintCommandNewline();
}
