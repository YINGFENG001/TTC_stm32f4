/**
  ******************************************************************************
 * @file    bsp_clamp_ctl.c
 * @brief   Clamp command, hold control and API implementation.
  ******************************************************************************
  */

#include "./gripper/bsp_clamp_ctl.h"

typedef struct {
  uint16_t speed;
  uint16_t grip_step;
  uint16_t release_delta;
} ClampParam;

typedef struct {
  uint8_t active;
  uint16_t target_load;
  uint8_t low_load_hits;
  uint8_t comm_fail_count;
  int16_t integral;
  uint16_t last_target_pos;
  uint16_t low_load_start_pos;
  int16_t last_load_abs;
  uint32_t last_adjust_print_tick;
  uint32_t last_stable_print_tick;
  BusServoStatus last_status;
} ClampHoldContext;

#define CLAMP_DROP_LOW_LOAD_PERCENT    40U
#define CLAMP_DROP_LOW_LOAD_HITS       5U
#define CLAMP_DROP_RECOVERY_POS_DELTA  200U
#define CLAMP_GRIP_FINE_STEP           10U
#define CLAMP_GRIP_PRELOAD_PERCENT     60U
#define CLAMP_HOLD_ADJUST_PRINT_INTERVAL_MS 2000U
#define CLAMP_HOLD_STABLE_PRINT_INTERVAL_MS 20000U
#define CLAMP_HOLD_STABLE_ERROR        20
#define CLAMP_HOLD_KP_NUM              10
#define CLAMP_HOLD_KI_NUM              2
#define CLAMP_HOLD_GAIN_DEN            100
#define CLAMP_HOLD_INTEGRAL_LIMIT      1000
#define CLAMP_HOLD_DELTA_LIMIT         15
#define CLAMP_HOLD_COMM_FAIL_LIMIT     2U

static ClampParam clamp_param = {
  GRIPPER_SPEED_DEFAULT,
  GRIPPER_GRIP_STEP_DEFAULT,
  GRIPPER_RELEASE_DELTA_DEFAULT
};

static ClampHoldContext clamp_hold = {FALSE, 0, 0, 0, 0, 0, 0, 0, 0, 0, {0}};

static int16_t Device_Abs16(int16_t value);
static uint16_t Clamp_LimitPosition(int32_t position);
static uint16_t Clamp_LoadPercent(uint16_t load, uint16_t percent);
static int16_t Clamp_LimitDelta(int32_t delta);
static int16_t Clamp_LimitIntegral(int32_t integral);
static GripperResult Clamp_ReadServoStatus(uint8_t servo_id, BusServoStatus *status_data, BusServoResult *servo_result);
static void Clamp_HoldBegin(uint16_t target_load, const BusServoStatus *status_data);
static void Clamp_HoldEnd(void);
static uint8_t Clamp_PositionValid(uint16_t position);

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
  clamp_hold.comm_fail_count = 0;
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
  clamp_hold.comm_fail_count = 0;
  clamp_hold.integral = 0;
  clamp_hold.last_target_pos = 0;
  clamp_hold.low_load_start_pos = 0;
  clamp_hold.last_load_abs = 0;
  clamp_hold.last_adjust_print_tick = 0;
  clamp_hold.last_stable_print_tick = 0;
}
static GripperResult Clamp_GripEnableAndRead(uint8_t servo_id, BusServoStatus *status_data,
                                               BusServoStatus *final_status, BusServoResult *servo_result)
{
  BusServoResult result;
  GripperResult grip_result;

  result = BusServo_SetTorqueEnable(servo_id, 1);
  if (servo_result != 0)
  {
    *servo_result = result;
  }
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_TORQUE_ON_ERROR;
  }

  grip_result = Clamp_ReadServoStatus(servo_id, status_data, servo_result);
  if ((grip_result == GRIPPER_OK) && (final_status != 0))
  {
    *final_status = *status_data;
  }
  return grip_result;
}

static GripperResult Clamp_GripMoveToPosition(uint8_t servo_id, uint16_t target_pos,
                                              BusServoStatus *status_data,
                                              BusServoStatus *final_status,
                                              BusServoResult *servo_result)
{
  BusServoResult result;
  GripperResult grip_result;
  uint16_t loops;
  uint16_t i;

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
  loops = (loops == 0) ? 1 : loops;
  for (i = 0; i < loops; i++)
  {
    delay_ms(GRIPPER_POLL_INTERVAL_MS);
    grip_result = Clamp_ReadServoStatus(servo_id, status_data, servo_result);
    if (grip_result != GRIPPER_OK)
    {
      return grip_result;
    }
    if (final_status != 0)
    {
      *final_status = *status_data;
    }
    if (Device_Abs16((int16_t)(target_pos - status_data->position)) <= GRIPPER_MOVE_TOLERANCE)
    {
      break;
    }
  }
  return GRIPPER_OK;
}

static GripperResult Clamp_GripFineClose(uint8_t servo_id, uint16_t preload_load,
                                         BusServoStatus *status_data,
                                         BusServoStatus *final_status,
                                         BusServoResult *servo_result)
{
  BusServoResult result;
  GripperResult grip_result;
  uint16_t target_pos;

  target_pos = Clamp_LimitPosition(status_data->position);
  while (target_pos < GRIPPER_POS_CLOSE_MIN)
  {
    if ((uint16_t)Device_Abs16(status_data->load) >= preload_load)
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
    grip_result = Clamp_ReadServoStatus(servo_id, status_data, servo_result);
    if (grip_result != GRIPPER_OK)
    {
      return grip_result;
    }
    if (final_status != 0)
    {
      *final_status = *status_data;
    }
  }
  return (Device_Abs16(status_data->load) >= (int16_t)preload_load) ? GRIPPER_OK : GRIPPER_NO_OBJECT;
}

static GripperResult Clamp_GripTwoStage(uint8_t servo_id, uint16_t load_threshold,
                                        uint8_t has_position, uint16_t position,
                                        BusServoStatus *final_status, BusServoResult *servo_result)
{
  GripperResult grip_result;
  BusServoStatus status_data;
  uint16_t preload_load;

  if ((load_threshold < GRIPPER_GRIP_LOAD_MIN) ||
      (load_threshold > GRIPPER_GRIP_LOAD_MAX) ||
      ((has_position == TRUE) && (Clamp_PositionValid(position) == FALSE)))
  {
    return GRIPPER_RANGE_ERROR;
  }

  grip_result = Clamp_GripEnableAndRead(servo_id, &status_data, final_status, servo_result);
  if (grip_result != GRIPPER_OK)
  {
    return grip_result;
  }

  preload_load = Clamp_LoadPercent(load_threshold, CLAMP_GRIP_PRELOAD_PERCENT);
  if (has_position == TRUE)
  {
    grip_result = Clamp_GripMoveToPosition(servo_id, position, &status_data, final_status, servo_result);
    if (grip_result != GRIPPER_OK)
    {
      return grip_result;
    }
  }

  return Clamp_GripFineClose(servo_id, preload_load, &status_data, final_status, servo_result);
}

static uint8_t Clamp_HoldReadStatus(BusServoStatus *status_data)
{
  uint8_t servo_error;
  BusServoResult result;

  servo_error = 0;
  result = BusServo_ReadStatus(GRIPPER_SERVO_ID_DEFAULT, status_data, &servo_error);
  if (result == BUS_SERVO_OK)
  {
    clamp_hold.comm_fail_count = 0;
    return TRUE;
  }

  clamp_hold.comm_fail_count++;
  if (clamp_hold.comm_fail_count < CLAMP_HOLD_COMM_FAIL_LIMIT)
  {
    printf("\n@warn dev=clamp event=status_error result=%s servo_err=0x%02X fail_count=%u action=retry",
           BusServo_ResultName(result),
           servo_error,
           clamp_hold.comm_fail_count);
    return FALSE;
  }

  devices[DEVICE_CLAMP].state = DEV_ERROR;
  devices[DEVICE_CLAMP].error_code = result;
  Clamp_HoldEnd();
  (void)BusServo_SetTorqueEnable(GRIPPER_SERVO_ID_DEFAULT, 0);
  printf("\n@fault dev=clamp event=status_error result=%s servo_err=0x%02X fail_count=%u action=torque_off",
         BusServo_ResultName(result),
         servo_error,
         CLAMP_HOLD_COMM_FAIL_LIMIT);
  return FALSE;
}

static void Clamp_HoldUpdateStatus(const BusServoStatus *status_data)
{
  clamp_hold.last_status = *status_data;
  devices[DEVICE_CLAMP].position = status_data->position;
  devices[DEVICE_CLAMP].target = status_data->position;
  devices[DEVICE_CLAMP].value = status_data->status;
  devices[DEVICE_CLAMP].error_code = 0;
  devices[DEVICE_CLAMP].state = DEV_RUNNING;
}

static uint8_t Clamp_HoldHandleStable(uint32_t now, const BusServoStatus *status_data,
                                      int16_t load_abs, int16_t error)
{
  if (Device_Abs16(error) > CLAMP_HOLD_STABLE_ERROR)
  {
    return FALSE;
  }

  clamp_hold.integral = 0;
  clamp_hold.low_load_hits = 0;
  clamp_hold.low_load_start_pos = 0;
  clamp_hold.last_load_abs = load_abs;
  clamp_hold.last_target_pos = Clamp_LimitPosition(status_data->position);
  if ((clamp_hold.last_stable_print_tick == 0) ||
      ((now - clamp_hold.last_stable_print_tick) >= CLAMP_HOLD_STABLE_PRINT_INTERVAL_MS))
  {
    clamp_hold.last_stable_print_tick = now;
    printf("\n@info dev=clamp event=hold_stable load=%d target=%u error=%d pos=%d",
           status_data->load,
           clamp_hold.target_load,
           error,
           status_data->position);
  }
  return TRUE;
}

static int16_t Clamp_HoldCalcDelta(int16_t error)
{
  int16_t delta_pos;
  int32_t pi_out;

  clamp_hold.integral = Clamp_LimitIntegral((int32_t)clamp_hold.integral + error);
  pi_out = ((int32_t)CLAMP_HOLD_KP_NUM * error) +
           ((int32_t)CLAMP_HOLD_KI_NUM * clamp_hold.integral);
  delta_pos = Clamp_LimitDelta(pi_out / CLAMP_HOLD_GAIN_DEN);
  if (delta_pos == 0)
  {
    delta_pos = (error > 0) ? 1 : -1;
  }
  return delta_pos;
}

static uint8_t Clamp_HoldCheckDrop(const BusServoStatus *status_data, int16_t load_abs,
                                   int16_t delta_pos)
{
  uint16_t low_load_limit;
  uint16_t recovery_delta;
  uint16_t fault_target_load;

  low_load_limit = Clamp_LoadPercent(clamp_hold.target_load, CLAMP_DROP_LOW_LOAD_PERCENT);
  if ((load_abs >= low_load_limit) || (delta_pos <= 0))
  {
    clamp_hold.low_load_hits = 0;
    clamp_hold.low_load_start_pos = 0;
    return FALSE;
  }

  if (clamp_hold.low_load_hits == 0)
  {
    clamp_hold.low_load_start_pos = Clamp_LimitPosition(status_data->position);
  }
  clamp_hold.low_load_hits++;
  recovery_delta = (status_data->position >= (int16_t)clamp_hold.low_load_start_pos) ?
                   (uint16_t)(status_data->position - (int16_t)clamp_hold.low_load_start_pos) : 0;
  if ((clamp_hold.low_load_hits < CLAMP_DROP_LOW_LOAD_HITS) ||
      (recovery_delta < CLAMP_DROP_RECOVERY_POS_DELTA))
  {
    return FALSE;
  }

  fault_target_load = clamp_hold.target_load;
  Clamp_HoldEnd();
  (void)BusServo_SetTorqueEnable(GRIPPER_SERVO_ID_DEFAULT, 0);
  devices[DEVICE_CLAMP].state = DEV_ERROR;
  devices[DEVICE_CLAMP].error_code = DEVICE_FAULT_DROP;
  printf("\n@fault dev=clamp event=drop load=%d target=%u pos=%d recover=%u action=torque_off",
         status_data->load,
         fault_target_load,
         status_data->position,
         recovery_delta);
  return TRUE;
}

static void Clamp_HoldApplyAdjust(uint32_t now, uint16_t next_pos,
                                  BusServoStatus *status_data, int16_t load_abs)
{
  BusServoResult result;
  int16_t error;

  result = BusServo_MoveRaw(GRIPPER_SERVO_ID_DEFAULT, next_pos, clamp_param.speed);
  if (result != BUS_SERVO_OK)
  {
    return;
  }

  clamp_hold.last_target_pos = next_pos;
  clamp_hold.last_load_abs = load_abs;
  devices[DEVICE_CLAMP].target = next_pos;
  delay_ms(GRIPPER_POLL_INTERVAL_MS);
  if (Clamp_ReadServoStatus(GRIPPER_SERVO_ID_DEFAULT, status_data, &result) == GRIPPER_OK)
  {
    load_abs = Device_Abs16(status_data->load);
    clamp_hold.last_status = *status_data;
    clamp_hold.last_load_abs = load_abs;
    devices[DEVICE_CLAMP].position = status_data->position;
    devices[DEVICE_CLAMP].value = status_data->status;
    devices[DEVICE_CLAMP].error_code = 0;
  }

  error = (int16_t)clamp_hold.target_load - load_abs;
  if ((clamp_hold.last_adjust_print_tick == 0) ||
      ((now - clamp_hold.last_adjust_print_tick) >= CLAMP_HOLD_ADJUST_PRINT_INTERVAL_MS))
  {
    clamp_hold.last_adjust_print_tick = now;
    printf("\n@info dev=clamp event=hold_pi load=%d target=%u error=%d pos=%d",
           status_data->load,
           clamp_hold.target_load,
           error,
           status_data->position);
  }
}

void Clamp_HoldTask(uint32_t now)
{
  static uint32_t last_control_tick = 0;
  BusServoStatus status_data;
  int16_t load_abs;
  int16_t error;
  int16_t delta_pos;
  uint16_t next_pos;

  if ((clamp_hold.active != TRUE) || ((now - last_control_tick) < DEVICE_FAST_CHECK_INTERVAL_MS))
  {
    return;
  }
  last_control_tick = now;

  if (Clamp_HoldReadStatus(&status_data) != TRUE)
  {
    return;
  }
  Clamp_FillExtraStatus(GRIPPER_SERVO_ID_DEFAULT, &status_data);
  Clamp_HoldUpdateStatus(&status_data);

  load_abs = Device_Abs16(status_data.load);
  error = (int16_t)clamp_hold.target_load - load_abs;
  if (Clamp_HoldHandleStable(now, &status_data, load_abs, error) == TRUE)
  {
    return;
  }

  delta_pos = Clamp_HoldCalcDelta(error);
  if (clamp_hold.last_target_pos == 0)
  {
    clamp_hold.last_target_pos = Clamp_LimitPosition(status_data.position);
  }
  next_pos = Clamp_LimitPosition((int32_t)clamp_hold.last_target_pos + delta_pos);
  if (Clamp_HoldCheckDrop(&status_data, load_abs, delta_pos) == TRUE)
  {
    return;
  }
  Clamp_HoldApplyAdjust(now, next_pos, &status_data, load_abs);
}

void Clamp_FillExtraStatus(uint8_t servo_id, BusServoStatus *status_data)
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

static uint8_t Clamp_PositionValid(uint16_t position)
{
  return ((position >= GRIPPER_POS_OPEN_MAX) && (position <= GRIPPER_POS_CLOSE_MIN)) ? TRUE : FALSE;
}

static uint16_t Clamp_OpenPercentToPosition(uint16_t open_pct)
{
  return Gripper_OpenPercentToPosition(open_pct);
}

static uint16_t Clamp_PositionToOpenPctX10(uint16_t position)
{
  return Gripper_PositionToOpenPercentX10(Clamp_LimitPosition(position));
}

static uint8_t Clamp_ParsePositionArg(const char *text, uint16_t *position)
{
  char *end;
  long parsed;

  if ((text == 0) || (position == 0))
  {
    return FALSE;
  }
  if (text[0] == '-')
  {
    return FALSE;
  }

  parsed = strtol(text, &end, 10);
  if (end == text)
  {
    return FALSE;
  }

  if (*end == '%')
  {
    if (*(end + 1) != '\0')
    {
      return FALSE;
    }
    if ((parsed < 0) || (parsed > 100))
    {
      return FALSE;
    }
    *position = Clamp_OpenPercentToPosition((uint16_t)parsed);
    return TRUE;
  }

  if (*end != '\0')
  {
    return FALSE;
  }

  if ((parsed < GRIPPER_POS_OPEN_MAX) || (parsed > GRIPPER_POS_CLOSE_MIN))
  {
    return FALSE;
  }

  *position = (uint16_t)parsed;
  return TRUE;
}

static void Clamp_PrintPositionPct(uint16_t position)
{
  uint16_t pct_x10;

  pct_x10 = Clamp_PositionToOpenPctX10(position);
  printf(" openPos/Pct = %u (%u.%u%%)",
         position,
         pct_x10 / 10U,
         pct_x10 % 10U);
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

void Clamp_PrintStatusFields(const BusServoStatus *status_data)
{
  int32_t current_ma_x10;
  int32_t current_ma_abs_x10;
  int32_t load_abs_x10;

  current_ma_x10 = (int32_t)status_data->current * 65;
  current_ma_abs_x10 = (current_ma_x10 < 0) ? -current_ma_x10 : current_ma_x10;
  load_abs_x10 = (status_data->load < 0) ? -(int32_t)status_data->load : (int32_t)status_data->load;
  Clamp_PrintPositionPct((uint16_t)Clamp_LimitPosition(status_data->position));
  printf(" speed=%d load=%d(%s%ld.%ld%%) voltage=%u.%uV temp=%u current=%d(%s%ld.%ldmA) state=0x%02X",
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
  printf("\nclamp %s id=%u",
         action,
         servo_id);
  Clamp_PrintPositionPct(position);
  printf(" speed=%u result=%s",
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

DeviceApiResult DeviceApi_ClampMove(uint16_t position, DeviceClampStatus *out)
{
  if (Clamp_PositionValid(position) == FALSE)
  {
    return DEVICE_API_RANGE_ERROR;
  }

  return DeviceApi_ClampMovePosition(position, out);
}

DeviceApiResult DeviceApi_ClampOpen(DeviceClampStatus *out)
{
  return DeviceApi_ClampMovePosition(GRIPPER_POS_OPEN_MAX, out);
}

DeviceApiResult DeviceApi_ClampClose(DeviceClampStatus *out)
{
  return DeviceApi_ClampMovePosition(GRIPPER_POS_CLOSE_MIN, out);
}

static DeviceApiResult DeviceApi_ClampGripInternal(uint16_t load, uint8_t has_position,
                                                   uint16_t position, DeviceClampStatus *out)
{
  BusServoResult servo_result;
  BusServoStatus final_status;
  GripperResult gripper_result;
  DeviceApiResult ret;

  memset(&final_status, 0, sizeof(final_status));
  gripper_result = Clamp_GripTwoStage(GRIPPER_SERVO_ID_DEFAULT, load,
                                      has_position, position,
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

DeviceApiResult DeviceApi_ClampGripAt(uint16_t load, uint16_t position, DeviceClampStatus *out)
{
  if (Clamp_PositionValid(position) == FALSE)
  {
    return DEVICE_API_RANGE_ERROR;
  }

  return DeviceApi_ClampGripInternal(load, TRUE, position, out);
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

static void Clamp_CommandPing(uint8_t device_id, int argc, char *argv[])
{
  uint8_t servo_id;
  uint8_t servo_error;
  BusServoResult result;

  servo_id = Clamp_ParseServoId(argc, argv, GRIPPER_SERVO_ID_DEFAULT);
  servo_error = 0;
  result = BusServo_Ping(servo_id, &servo_error);
  Clamp_PrintResult("ping", servo_id, result, servo_error);
  printf("\n");

  devices[device_id].state = (result == BUS_SERVO_OK) ? DEV_IDLE : DEV_ERROR;
  devices[device_id].error_code = (result == BUS_SERVO_OK) ? 0 : result;
}

static void Clamp_CommandStatus(uint8_t device_id, int argc, char *argv[])
{
  uint8_t servo_id;
  uint8_t servo_error;
  BusServoResult result;
  BusServoStatus status_data;

  servo_id = Clamp_ParseServoId(argc, argv, GRIPPER_SERVO_ID_DEFAULT);
  servo_error = 0;
  result = BusServo_ReadStatus(servo_id, &status_data, &servo_error);
  Clamp_PrintResult("status", servo_id, result, servo_error);
  if (result != BUS_SERVO_OK)
  {
    devices[device_id].state = DEV_ERROR;
    devices[device_id].error_code = result;
    printf("\n");
    return;
  }

  Clamp_FillExtraStatus(servo_id, &status_data);
  devices[device_id].state = (clamp_hold.active == TRUE) ? DEV_RUNNING :
                             ((status_data.moving != 0) ? DEV_RUNNING : DEV_IDLE);
  devices[device_id].position = status_data.position;
  devices[device_id].value = status_data.status;
  devices[device_id].error_code = 0;
  Clamp_PrintStatusFields(&status_data);
  printf("\n");
}

static void Clamp_CommandReadReg(int argc, char *argv[])
{
  uint8_t servo_error;
  uint8_t reg_addr;
  uint8_t reg_value;
  BusServoResult result;

  if (argc != 3)
  {
    printf("\nclamp readreg param_error");
    return;
  }
  if ((atoi(argv[2]) < 0) || (atoi(argv[2]) > 255))
  {
    printf("\nclamp readreg range_error");
    return;
  }

  servo_error = 0;
  reg_addr = (uint8_t)atoi(argv[2]);
  reg_value = 0;
  result = BusServo_ReadData(GRIPPER_SERVO_ID_DEFAULT, reg_addr, 1, &reg_value, &servo_error);
  printf("\nclamp readreg id=%u addr=%u(0x%02X) result=%s",
         GRIPPER_SERVO_ID_DEFAULT,
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
}

static void Clamp_CommandSet(int argc, char *argv[])
{
  uint16_t speed;
  uint16_t step;
  uint16_t delta;

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
}

static void Clamp_UpdateMoveState(uint8_t device_id, uint16_t target_pos, GripperResult result)
{
  devices[device_id].target = target_pos;
  devices[device_id].error_code = (result == GRIPPER_OK) ? 0 : result;
  devices[device_id].state = (result == GRIPPER_OK) ? DEV_IDLE : DEV_ERROR;
}

static void Clamp_CommandOpenClose(uint8_t device_id, int argc, uint16_t target_pos, const char *action)
{
  BusServoStatus final_status;
  BusServoResult servo_result;
  GripperResult gripper_result;

  if (argc != 2)
  {
    printf("\nclamp %s param_error", action);
    return;
  }

  memset(&final_status, 0, sizeof(final_status));
  Clamp_HoldEnd();
  gripper_result = Gripper_MoveFeedback(GRIPPER_SERVO_ID_DEFAULT, target_pos, clamp_param.speed,
                                        &final_status, &servo_result);
  Clamp_PrintMoveResult(action, GRIPPER_SERVO_ID_DEFAULT, target_pos, clamp_param.speed,
                        gripper_result, servo_result, &final_status);
  Clamp_UpdateMoveState(device_id, target_pos, gripper_result);
}

static void Clamp_CommandMove(uint8_t device_id, int argc, char *argv[])
{
  uint16_t position;
  BusServoStatus final_status;
  BusServoResult servo_result;
  GripperResult gripper_result;

  if (argc != 3)
  {
    printf("\nclamp move param_error");
    return;
  }
  if (Clamp_ParsePositionArg(argv[2], &position) != TRUE)
  {
    printf("\nclamp move range_error");
    return;
  }

  memset(&final_status, 0, sizeof(final_status));
  Clamp_HoldEnd();
  gripper_result = Gripper_MoveFeedback(GRIPPER_SERVO_ID_DEFAULT, position, clamp_param.speed,
                                        &final_status, &servo_result);
  Clamp_PrintMoveResult("move", GRIPPER_SERVO_ID_DEFAULT, position, clamp_param.speed,
                        gripper_result, servo_result, &final_status);
  Clamp_UpdateMoveState(device_id, position, gripper_result);
}

static void Clamp_CommandGrip(uint8_t device_id, int argc, char *argv[])
{
  BusServoResult servo_result;
  BusServoStatus final_status;
  GripperResult gripper_result;
  uint16_t load_threshold;
  uint16_t position;

  if ((argc != 3) && (argc != 4))
  {
    printf("\nclamp grip param_error");
    return;
  }

  load_threshold = (uint16_t)atoi(argv[2]);
  position = 0;
  if ((argc == 4) && (Clamp_ParsePositionArg(argv[3], &position) != TRUE))
  {
    printf("\nclamp grip range_error");
    return;
  }

  memset(&final_status, 0, sizeof(final_status));
  gripper_result = Clamp_GripTwoStage(GRIPPER_SERVO_ID_DEFAULT, load_threshold,
                                      (argc == 4) ? TRUE : FALSE,
                                      position,
                                      &final_status, &servo_result);
  printf("\nclamp grip id=%u load=%u", GRIPPER_SERVO_ID_DEFAULT, load_threshold);
  if (argc == 4)
  {
    Clamp_PrintPositionPct(position);
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
    devices[device_id].position = final_status.position;
    devices[device_id].target = final_status.position;
  }

  devices[device_id].error_code = (gripper_result == GRIPPER_OK) ? 0 : gripper_result;
  if (gripper_result == GRIPPER_OK)
  {
    Clamp_HoldBegin(load_threshold, &final_status);
    return;
  }
  devices[device_id].state = DEV_ERROR;
}

static void Clamp_CommandRelease(uint8_t device_id, int argc)
{
  int16_t cur_pos;
  uint16_t target_pos;
  BusServoResult servo_result;
  BusServoStatus final_status;
  GripperResult gripper_result;

  if (argc != 2)
  {
    printf("\nclamp release param_error");
    return;
  }

  cur_pos = 0;
  target_pos = 0;
  memset(&final_status, 0, sizeof(final_status));
  Clamp_HoldEnd();
  gripper_result = Gripper_Release(GRIPPER_SERVO_ID_DEFAULT, clamp_param.release_delta,
                                   clamp_param.speed, &cur_pos, &target_pos,
                                   &final_status, &servo_result);
  printf("\nclamp release id=%u cur=%d target=%u delta=%u speed=%u result=%s",
         GRIPPER_SERVO_ID_DEFAULT,
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
}

void Clamp_Command(uint8_t device_id, int argc, char *argv[])
{
  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    return;
  }
  if (strcmp(argv[1], "ping") == 0)
  {
    Clamp_CommandPing(device_id, argc, argv);
    return;
  }
  if (strcmp(argv[1], "status") == 0)
  {
    Clamp_CommandStatus(device_id, argc, argv);
    return;
  }
  if (strcmp(argv[1], "readreg") == 0)
  {
    Clamp_CommandReadReg(argc, argv);
    return;
  }
  if (strcmp(argv[1], "set") == 0)
  {
    Clamp_CommandSet(argc, argv);
    return;
  }
  if (strcmp(argv[1], "open") == 0)
  {
    Clamp_CommandOpenClose(device_id, argc, GRIPPER_POS_OPEN_MAX, "open");
    return;
  }
  if (strcmp(argv[1], "close") == 0)
  {
    Clamp_CommandOpenClose(device_id, argc, GRIPPER_POS_CLOSE_MIN, "close");
    return;
  }
  if (strcmp(argv[1], "move") == 0)
  {
    Clamp_CommandMove(device_id, argc, argv);
    return;
  }
  if (strcmp(argv[1], "grip") == 0)
  {
    Clamp_CommandGrip(device_id, argc, argv);
    return;
  }
  if (strcmp(argv[1], "release") == 0)
  {
    Clamp_CommandRelease(device_id, argc);
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
}
