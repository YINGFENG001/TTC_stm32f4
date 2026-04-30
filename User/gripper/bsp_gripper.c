#include "./gripper/bsp_gripper.h"
#include "./delay/core_delay.h"

static uint8_t Gripper_PositionValid(uint16_t position)
{
  return ((position >= GRIPPER_POS_OPEN_MAX) && (position <= GRIPPER_POS_CLOSE_MIN)) ? 1 : 0;
}

static uint8_t Gripper_SpeedValid(uint16_t speed)
{
  return ((speed >= GRIPPER_SPEED_MIN) && (speed <= GRIPPER_SPEED_MAX)) ? 1 : 0;
}

static int16_t Gripper_Abs16(int16_t value)
{
  return (value < 0) ? (int16_t)(-value) : value;
}

static int16_t Gripper_PositionDiff(uint16_t target, int16_t current)
{
  return (int16_t)((int32_t)target - current);
}

static void Gripper_SetServoResult(BusServoResult *servo_result, BusServoResult result)
{
  if (servo_result != 0)
  {
    *servo_result = result;
  }
}

static void Gripper_CopyStatus(BusServoStatus *dst, const BusServoStatus *src)
{
  if ((dst != 0) && (src != 0))
  {
    *dst = *src;
  }
}

static GripperResult Gripper_StopOnStall(uint8_t servo_id, BusServoResult *servo_result)
{
  BusServoResult result;

  result = BusServo_SetTorqueEnable(servo_id, 0);
  Gripper_SetServoResult(servo_result, result);
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_TORQUE_OFF_ERROR;
  }

  return GRIPPER_STALL;
}

GripperResult Gripper_MoveFeedback(uint8_t servo_id, uint16_t position, uint16_t speed,
                                   BusServoStatus *final_status, BusServoResult *servo_result)
{
  BusServoStatus status;
  BusServoResult result;
  uint16_t loops;
  uint16_t i;
  uint8_t stall_hits;

  Gripper_SetServoResult(servo_result, BUS_SERVO_OK);

  if ((Gripper_PositionValid(position) == 0) || (Gripper_SpeedValid(speed) == 0))
  {
    return GRIPPER_RANGE_ERROR;
  }

  result = BusServo_MoveRaw(servo_id, position, speed);
  Gripper_SetServoResult(servo_result, result);
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_MOVE_ERROR;
  }

  /* 发送目标后进入轮询反馈阶段：优先判断到位，其次判断持续堵转，最后由超时兜底。 */
  loops = (uint16_t)(GRIPPER_MOVE_TIMEOUT_MS / GRIPPER_POLL_INTERVAL_MS);
  if (loops == 0)
  {
    loops = 1;
  }

  stall_hits = 0;
  for (i = 0; i < loops; i++)
  {
    delay_ms(GRIPPER_POLL_INTERVAL_MS);

    result = BusServo_ReadStatus(servo_id, &status, 0);
    Gripper_SetServoResult(servo_result, result);
    if (result != BUS_SERVO_OK)
    {
      return GRIPPER_STATUS_ERROR;
    }

    Gripper_CopyStatus(final_status, &status);

    if (Gripper_Abs16(Gripper_PositionDiff(position, status.position)) <= GRIPPER_MOVE_TOLERANCE)
    {
      return GRIPPER_OK;
    }

    if (Gripper_Abs16(status.load) >= GRIPPER_STALL_LOAD_LIMIT)
    {
      stall_hits++;
      if (stall_hits >= GRIPPER_STALL_HIT_LIMIT)
      {
        return Gripper_StopOnStall(servo_id, servo_result);
      }
    }
    else
    {
      stall_hits = 0;
    }
  }

  return GRIPPER_TIMEOUT;
}

GripperResult Gripper_Grip(uint8_t servo_id, uint16_t load_threshold, uint16_t speed,
                           uint16_t step, BusServoStatus *final_status,
                           BusServoResult *servo_result)
{
  BusServoStatus status;
  BusServoResult result;
  uint16_t target;
  uint8_t load_hits;

  Gripper_SetServoResult(servo_result, BUS_SERVO_OK);

  if ((load_threshold < GRIPPER_GRIP_LOAD_MIN) ||
      (load_threshold > GRIPPER_GRIP_LOAD_MAX) ||
      (Gripper_SpeedValid(speed) == 0) ||
      (step < GRIPPER_GRIP_STEP_MIN) ||
      (step > GRIPPER_GRIP_STEP_MAX))
  {
    return GRIPPER_RANGE_ERROR;
  }

  result = BusServo_ReadStatus(servo_id, &status, 0);
  Gripper_SetServoResult(servo_result, result);
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_STATUS_ERROR;
  }
  Gripper_CopyStatus(final_status, &status);

  load_hits = 0;
  target = (status.position < GRIPPER_POS_OPEN_MAX) ? GRIPPER_POS_OPEN_MAX : (uint16_t)status.position;

  /* grip 不直接冲到闭合终点，而是按 step 逐步逼近，通过负载阈值判断是否已经夹住物体。 */
  while (target < GRIPPER_POS_CLOSE_MIN)
  {
    if ((uint16_t)(GRIPPER_POS_CLOSE_MIN - target) < step)
    {
      target = GRIPPER_POS_CLOSE_MIN;
    }
    else
    {
      target = (uint16_t)(target + step);
    }

    result = BusServo_MoveRaw(servo_id, target, speed);
    Gripper_SetServoResult(servo_result, result);
    if (result != BUS_SERVO_OK)
    {
      return GRIPPER_MOVE_ERROR;
    }

    delay_ms(GRIPPER_POLL_INTERVAL_MS);

    result = BusServo_ReadStatus(servo_id, &status, 0);
    Gripper_SetServoResult(servo_result, result);
    if (result != BUS_SERVO_OK)
    {
      return GRIPPER_STATUS_ERROR;
    }
    Gripper_CopyStatus(final_status, &status);

    if (Gripper_Abs16(status.load) >= (int16_t)load_threshold)
    {
      load_hits++;
      if (load_hits >= GRIPPER_GRIP_HIT_LIMIT)
      {
        return GRIPPER_OK;
      }
    }
    else
    {
      load_hits = 0;
    }
  }

  return GRIPPER_NO_OBJECT;
}

GripperResult Gripper_Release(uint8_t servo_id, uint16_t delta, uint16_t speed,
                              int16_t *cur_pos, uint16_t *target_pos,
                              BusServoStatus *final_status, BusServoResult *servo_result)
{
  BusServoStatus status;
  BusServoResult result;
  uint16_t release_pos;

  Gripper_SetServoResult(servo_result, BUS_SERVO_OK);

  if ((delta < GRIPPER_RELEASE_DELTA_MIN) ||
      (delta > GRIPPER_RELEASE_DELTA_MAX) ||
      (speed < GRIPPER_RELEASE_SPEED_MIN) ||
      (speed > GRIPPER_RELEASE_SPEED_MAX))
  {
    return GRIPPER_RANGE_ERROR;
  }

  result = BusServo_SetTorqueEnable(servo_id, 0);
  Gripper_SetServoResult(servo_result, result);
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_TORQUE_OFF_ERROR;
  }

  /* 先卸力再读取当前位置，拿到夹爪在自然回弹/受力状态下的实际位置，再重新上扭矩退开。 */
  result = BusServo_ReadStatus(servo_id, &status, 0);
  Gripper_SetServoResult(servo_result, result);
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_STATUS_ERROR;
  }

  if (cur_pos != 0)
  {
    *cur_pos = status.position;
  }

  if (status.position <= (int16_t)(GRIPPER_POS_OPEN_MAX + delta))
  {
    release_pos = GRIPPER_POS_OPEN_MAX;
  }
  else
  {
    release_pos = (uint16_t)(status.position - (int16_t)delta);
  }

  if (target_pos != 0)
  {
    *target_pos = release_pos;
  }

  result = BusServo_SetTorqueEnable(servo_id, 1);
  Gripper_SetServoResult(servo_result, result);
  if (result != BUS_SERVO_OK)
  {
    return GRIPPER_TORQUE_ON_ERROR;
  }

  return Gripper_MoveFeedback(servo_id, release_pos, speed, final_status, servo_result);
}

GripperResult Gripper_Open(uint8_t servo_id, uint16_t speed, BusServoResult *servo_result)
{
  return Gripper_MoveFeedback(servo_id, GRIPPER_POS_OPEN_MAX, speed, 0, servo_result);
}

GripperResult Gripper_Close(uint8_t servo_id, uint16_t speed, BusServoResult *servo_result)
{
  return Gripper_MoveFeedback(servo_id, GRIPPER_POS_CLOSE_MIN, speed, 0, servo_result);
}

const char *Gripper_ResultName(GripperResult result)
{
  switch (result)
  {
    case GRIPPER_OK:               return "ok";
    case GRIPPER_RANGE_ERROR:      return "range_error";
    case GRIPPER_SERVO_ERROR:      return "servo_error";
    case GRIPPER_STATUS_ERROR:     return "status_error";
    case GRIPPER_STALL:            return "stall";
    case GRIPPER_TIMEOUT:          return "timeout";
    case GRIPPER_NO_OBJECT:        return "no_object";
    case GRIPPER_TORQUE_OFF_ERROR: return "torque_off_error";
    case GRIPPER_TORQUE_ON_ERROR:  return "torque_on_error";
    case GRIPPER_MOVE_ERROR:       return "move_error";
    default:                       return "unknown";
  }
}
