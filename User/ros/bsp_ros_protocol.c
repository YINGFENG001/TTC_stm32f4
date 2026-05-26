#include "./ros/bsp_ros_protocol.h"
#include "./stepper/bsp_device_usart_ctl.h"
#include "./gripper/bsp_gripper.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint32_t id;
  const char *dev;
  const char *cmd;
} RosCmdContext;

static int Ros_ParseArgv(char *line, char *argv[], int max_argc)
{
  int argc;
  char *token;

  argc = 0;
  token = strtok(line, " \r\n");
  while ((token != 0) && (argc < max_argc))
  {
    argv[argc++] = token;
    token = strtok(0, " \r\n");
  }
  return argc;
}

static uint8_t Ros_ParseI32(const char *text, int32_t *out)
{
  char *end;
  long value;

  if ((text == 0) || (out == 0))
  {
    return FALSE;
  }

  value = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return FALSE;
  }

  *out = (int32_t)value;
  return TRUE;
}

static uint8_t Ros_ParseU32(const char *text, uint32_t *out)
{
  char *end;
  unsigned long value;

  if ((text == 0) || (out == 0) || (text[0] == '-'))
  {
    return FALSE;
  }

  value = strtoul(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return FALSE;
  }

  *out = (uint32_t)value;
  return TRUE;
}

static uint8_t Ros_ParseU16(const char *text, uint16_t *out)
{
  uint32_t value;

  if ((Ros_ParseU32(text, &value) != TRUE) || (value > 65535U))
  {
    return FALSE;
  }

  *out = (uint16_t)value;
  return TRUE;
}

static uint8_t Ros_ParseU8(const char *text, uint8_t *out)
{
  uint32_t value;

  if ((Ros_ParseU32(text, &value) != TRUE) || (value > 255U))
  {
    return FALSE;
  }

  *out = (uint8_t)value;
  return TRUE;
}

static uint16_t Ros_ClampOpenPctX10(uint16_t position)
{
  uint32_t span;

  if (position < GRIPPER_POS_OPEN_MAX)
  {
    position = GRIPPER_POS_OPEN_MAX;
  }
  if (position > GRIPPER_POS_CLOSE_MIN)
  {
    position = GRIPPER_POS_CLOSE_MIN;
  }

  span = (uint32_t)(GRIPPER_POS_CLOSE_MIN - GRIPPER_POS_OPEN_MAX);
  return (uint16_t)((((uint32_t)(GRIPPER_POS_CLOSE_MIN - position) * 1000U) + (span / 2U)) / span);
}

static uint16_t Ros_ClampOpenPctToPosition(uint16_t open_pct)
{
  uint32_t span;

  if (open_pct > 100)
  {
    open_pct = 100;
  }

  span = (uint32_t)(GRIPPER_POS_CLOSE_MIN - GRIPPER_POS_OPEN_MAX);
  return (uint16_t)(GRIPPER_POS_CLOSE_MIN - ((span * open_pct + 50U) / 100U));
}

static uint8_t Ros_ParseClampPosition(const char *text, uint16_t *position)
{
  char *end;
  unsigned long value;

  if ((text == 0) || (position == 0))
  {
    return FALSE;
  }

  if (text[0] == '-')
  {
    return FALSE;
  }

  value = strtoul(text, &end, 10);
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
    if (value > 100)
    {
      return FALSE;
    }
    *position = Ros_ClampOpenPctToPosition((uint16_t)value);
    return TRUE;
  }

  if ((*end != '\0') || (value > 65535U))
  {
    return FALSE;
  }
  if ((value < GRIPPER_POS_OPEN_MAX) || (value > GRIPPER_POS_CLOSE_MIN))
  {
    return FALSE;
  }

  *position = (uint16_t)value;
  return TRUE;
}

static void Ros_PrintClampPosition(uint16_t position)
{
  uint16_t pct_x10;

  pct_x10 = Ros_ClampOpenPctX10(position);
  printf(" openPos/Pct = %u (%u.%u%%)",
         position,
         pct_x10 / 10U,
         pct_x10 % 10U);
}

static void Ros_PrintAck(const RosCmdContext *ctx, const char *result)
{
  printf("\n@ack id=%lu dev=%s cmd=%s result=%s",
         (unsigned long)ctx->id, ctx->dev, ctx->cmd, result);
}

static void Ros_PrintDone(const RosCmdContext *ctx, const char *result)
{
  printf("\n@done id=%lu dev=%s cmd=%s result=%s",
         (unsigned long)ctx->id, ctx->dev, ctx->cmd, result);
}

static void Ros_PrintState(uint32_t id, const char *dev, const char *result)
{
  printf("\n@state id=%lu dev=%s result=%s",
         (unsigned long)id, dev, result);
}

static void Ros_PrintErr(const RosCmdContext *ctx, const char *code, const char *detail)
{
  printf("\n@err id=%lu dev=%s cmd=%s code=%s",
         (unsigned long)ctx->id, ctx->dev, ctx->cmd, code);
  if (detail != 0)
  {
    printf(" detail=%s", detail);
  }
}

static const char *Ros_ResultName(DeviceApiResult ret)
{
  switch (ret)
  {
    case DEVICE_API_OK:          return "ok";
    case DEVICE_API_BUSY:        return "busy";
    case DEVICE_API_DISABLED:    return "disabled";
    case DEVICE_API_PARAM_ERROR: return "param_error";
    case DEVICE_API_RANGE_ERROR: return "range_error";
    case DEVICE_API_UART_ERROR:  return "uart_error";
    case DEVICE_API_TIMEOUT:     return "timeout";
    case DEVICE_API_CRC_ERROR:   return "crc_error";
    case DEVICE_API_ID_ERROR:    return "id_error";
    default:                     return "device_error";
  }
}

static uint8_t Ros_StepperMotorId(const char *dev)
{
  return (strcmp(dev, "mtor1") == 0) ? 0 : 1;
}

static void Ros_PrintStepperState(uint32_t id, const char *dev)
{
  uint8_t motor_id;
  DeviceStepperStatus s;
  DeviceApiResult ret;

  motor_id = Ros_StepperMotorId(dev);
  ret = DeviceApi_StepperStatus(motor_id, &s);
  if (ret == DEVICE_API_OK)
  {
    Ros_PrintState(id, dev, "ok");
    printf(" enabled=%u running=%u rev=%ld target=%ld err=%lu accel=%lu decel=%lu rpm=%lu",
           s.enabled, s.running, (long)s.rev_0p1, (long)s.target_0p1,
           (unsigned long)s.err, (unsigned long)s.accel_rpm_s,
           (unsigned long)s.decel_rpm_s, (unsigned long)s.rpm);
  }
  else
  {
    RosCmdContext ctx = {id, dev, "status"};
    Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
  }
}

static void Ros_HandleStepper(uint32_t id, const char *dev, int argc, char *argv[])
{
  uint8_t motor_id;
  int32_t rev;
  uint32_t rpm;
  DeviceApiResult ret;
  RosCmdContext ctx;

  if (argc < 1)
  {
    ctx.id = id;
    ctx.dev = dev;
    ctx.cmd = "none";
    Ros_PrintErr(&ctx, "param_error", 0);
    return;
  }

  motor_id = Ros_StepperMotorId(dev);
  ctx.id = id;
  ctx.dev = dev;
  ctx.cmd = argv[0];

  if (strcmp(argv[0], "status") == 0)
  {
    if (argc != 1)
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }
    Ros_PrintStepperState(id, dev);
    return;
  }

  if (strcmp(argv[0], "move") == 0)
  {
    if (argc != 2)
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    if (strcmp(argv[1], "+0") == 0)
    {
      ret = DeviceApi_StepperRun(motor_id, CW);
      rev = 0;
    }
    else if (strcmp(argv[1], "-0") == 0)
    {
      ret = DeviceApi_StepperRun(motor_id, CCW);
      rev = 0;
    }
    else if ((Ros_ParseI32(argv[1], &rev) == TRUE) && (rev != 0))
    {
      ret = DeviceApi_StepperMove(motor_id, rev);
    }
    else
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    if (ret == DEVICE_API_OK)
    {
      DeviceApi_BindStepperRosCmd(motor_id, id, "move");
      Ros_PrintAck(&ctx, "ok");
      printf(" rev=%ld", (long)rev);
      if ((strcmp(argv[1], "+0") == 0) || (strcmp(argv[1], "-0") == 0))
      {
        printf(" continuous=1 dir=%s", (strcmp(argv[1], "-0") == 0) ? "ccw" : "cw");
      }
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if (strcmp(argv[0], "stop") == 0)
  {
    if (argc != 1)
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    ret = DeviceApi_StepperStop(motor_id, &rev);
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintAck(&ctx, "ok");
      printf(" rev=%ld", (long)rev);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if ((strcmp(argv[0], "accel") == 0) ||
      (strcmp(argv[0], "decel") == 0) ||
      (strcmp(argv[0], "rpm") == 0))
  {
    if ((argc != 2) || (Ros_ParseU32(argv[1], &rpm) != TRUE))
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    if (strcmp(argv[0], "accel") == 0)
    {
      ret = DeviceApi_StepperSetAccel(motor_id, rpm);
    }
    else if (strcmp(argv[0], "decel") == 0)
    {
      ret = DeviceApi_StepperSetDecel(motor_id, rpm);
    }
    else
    {
      ret = DeviceApi_StepperSetRpm(motor_id, rpm);
    }

    if (ret == DEVICE_API_OK)
    {
      Ros_PrintAck(&ctx, "ok");
      printf(" value=%lu", (unsigned long)rpm);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  Ros_PrintErr(&ctx, "unknown_action", 0);
}

static void Ros_PrintClampDone(const RosCmdContext *ctx, const DeviceClampStatus *s)
{
  Ros_PrintDone(ctx, "ok");
  Ros_PrintClampPosition((uint16_t)s->pos);
  printf(" load=%d current=%d state=0x%02X",
         s->load, s->current, s->state);
}

static void Ros_HandleClamp(uint32_t id, int argc, char *argv[])
{
  uint16_t position;
  uint16_t speed;
  uint16_t load;
  uint16_t step;
  uint16_t delta;
  uint8_t servo_id;
  DeviceClampStatus s;
  DeviceApiResult ret;
  RosCmdContext ctx;

  if (argc < 1)
  {
    ctx.id = id;
    ctx.dev = "clamp";
    ctx.cmd = "none";
    Ros_PrintErr(&ctx, "param_error", 0);
    return;
  }

  ctx.id = id;
  ctx.dev = "clamp";
  ctx.cmd = argv[0];

  if (strcmp(argv[0], "status") == 0)
  {
    if ((argc > 2) || ((argc == 2) && (Ros_ParseU8(argv[1], &servo_id) != TRUE)))
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }
    if (argc == 1)
    {
      servo_id = 10;
    }

    ret = DeviceApi_ClampStatus(servo_id, &s);
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintState(id, "clamp", "ok");
      printf(" servo_id=%u", s.servo_id);
      Ros_PrintClampPosition((uint16_t)s.pos);
      printf(" speed=%d load=%d voltage=%u temp=%u current=%d state=0x%02X",
             s.speed, s.load, s.voltage, s.temp, s.current, s.state);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if (strcmp(argv[0], "move") == 0)
  {
    if ((argc != 2) ||
        (Ros_ParseClampPosition(argv[1], &position) != TRUE))
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    ret = DeviceApi_ClampMove(position, &s);
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintClampDone(&ctx, &s);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if ((strcmp(argv[0], "open") == 0) || (strcmp(argv[0], "close") == 0))
  {
    if (argc != 1)
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    if (strcmp(argv[0], "open") == 0)
    {
      ret = DeviceApi_ClampOpen(&s);
    }
    else
    {
      ret = DeviceApi_ClampClose(&s);
    }

    if (ret == DEVICE_API_OK)
    {
      position = (uint16_t)s.pos;
      Ros_PrintClampDone(&ctx, &s);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if (strcmp(argv[0], "grip") == 0)
  {
    if (((argc != 2) && (argc != 3)) || (Ros_ParseU16(argv[1], &load) != TRUE))
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    if (argc == 3)
    {
      if (Ros_ParseClampPosition(argv[2], &position) != TRUE)
      {
        Ros_PrintErr(&ctx, "param_error", 0);
        return;
      }
      ret = DeviceApi_ClampGripAt(load, position, &s);
    }
    else
    {
      ret = DeviceApi_ClampGrip(load, &s);
    }
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintDone(&ctx, "ok");
      printf(" load=%u", load);
      if (argc == 3)
      {
        Ros_PrintClampPosition(position);
      }
      printf(" current=%d state=0x%02X",
             s.current, s.state);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if (strcmp(argv[0], "release") == 0)
  {
    if (argc != 1)
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    ret = DeviceApi_ClampRelease(&s);
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintDone(&ctx, "ok");
      Ros_PrintClampPosition((uint16_t)s.pos);
      printf(" current=%d state=0x%02X",
             s.current, s.state);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  if (strcmp(argv[0], "set") == 0)
  {
    if ((argc != 4) ||
        (Ros_ParseU16(argv[1], &speed) != TRUE) ||
        (Ros_ParseU16(argv[2], &step) != TRUE) ||
        (Ros_ParseU16(argv[3], &delta) != TRUE))
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    ret = DeviceApi_ClampSet(speed, step, delta);
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintAck(&ctx, "ok");
      printf(" speed=%u gripStep=%u releaseDelta=%u", speed, step, delta);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
    }
    return;
  }

  Ros_PrintErr(&ctx, "unknown_action", 0);
}

static void Ros_PrintVacumState(uint32_t id)
{
  DeviceVacumStatus s;
  DeviceApiResult ret;
  RosCmdContext ctx;

  ret = DeviceApi_VacumStatus(&s);
  if (ret == DEVICE_API_OK)
  {
    Ros_PrintState(id, "vacum", "ok");
    printf(" state1=0x%04X state2=0x%04X fault=0x%04X busy1=%u busy2=%u obj1=%u obj2=%u vac1=%u vac2=%u temp=%u bus=%u",
           s.state1, s.state2, s.fault, s.busy1, s.busy2, s.obj1, s.obj2,
           s.vac1, s.vac2, s.temp, s.bus_x10);
  }
  else
  {
    ctx.id = id;
    ctx.dev = "vacum";
    ctx.cmd = "status";
    Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
  }
}

static void Ros_HandleVacum(uint32_t id, int argc, char *argv[])
{
  uint8_t min_vac;
  uint8_t max_vac;
  uint8_t timeout;
  DeviceApiResult ret;
  RosCmdContext ctx;

  if (argc < 1)
  {
    ctx.id = id;
    ctx.dev = "vacum";
    ctx.cmd = "none";
    Ros_PrintErr(&ctx, "param_error", 0);
    return;
  }

  ctx.id = id;
  ctx.dev = "vacum";
  ctx.cmd = argv[0];

  if (strcmp(argv[0], "status") == 0)
  {
    if (argc != 1)
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }
    Ros_PrintVacumState(id);
    return;
  }

  if (strcmp(argv[0], "set") == 0)
  {
    if ((argc != 4) ||
        (Ros_ParseU8(argv[1], &min_vac) != TRUE) ||
        (Ros_ParseU8(argv[2], &max_vac) != TRUE) ||
        (Ros_ParseU8(argv[3], &timeout) != TRUE))
    {
      Ros_PrintErr(&ctx, "param_error", 0);
      return;
    }

    ret = DeviceApi_VacumSet(min_vac, max_vac, timeout);
    if (ret == DEVICE_API_OK)
    {
      Ros_PrintAck(&ctx, "ok");
      printf(" min=%u max=%u timeout=%u", min_vac, max_vac, timeout);
    }
    else
    {
      Ros_PrintErr(&ctx, Ros_ResultName(ret), "min_max_timeout");
    }
    return;
  }

  if (argc != 1)
  {
    Ros_PrintErr(&ctx, "param_error", 0);
    return;
  }

  if (strcmp(argv[0], "grip") == 0)
  {
    ret = DeviceApi_VacumGrip();
  }
  else if (strcmp(argv[0], "release") == 0)
  {
    ret = DeviceApi_VacumRelease();
  }
  else if (strcmp(argv[0], "stop") == 0)
  {
    ret = DeviceApi_VacumStop();
  }
  else
  {
    Ros_PrintErr(&ctx, "unknown_action", 0);
    return;
  }

  if (ret == DEVICE_API_OK)
  {
    Ros_PrintAck(&ctx, "ok");
  }
  else
  {
    Ros_PrintErr(&ctx, Ros_ResultName(ret), 0);
  }
}

static void Ros_HandleSystemStatus(uint32_t id)
{
  RosCmdContext ctx = {id, "system", "status"};
  char *clamp_status[] = {"status"};

  Ros_PrintStepperState(id, "mtor1");
  Ros_PrintStepperState(id, "mtor2");
  Ros_HandleClamp(id, 1, clamp_status);
  Ros_PrintVacumState(id);
  Ros_PrintDone(&ctx, "ok");
}

uint8_t RosProtocol_TryDispatch(char *line)
{
  char *argv[8];
  int argc;
  uint32_t id;

  if ((line == 0) || (line[0] != '#'))
  {
    return FALSE;
  }

  argc = Ros_ParseArgv(line, argv, 8);
  if (argc < 2)
  {
    printf("\n@err id=0 dev=unknown cmd=none code=param_error");
    return TRUE;
  }

  if ((Ros_ParseU32(&argv[0][1], &id) != TRUE) || (id == 0))
  {
    printf("\n@err id=0 dev=unknown cmd=none code=id_error");
    return TRUE;
  }

  if (strcmp(argv[1], "status") == 0)
  {
    if (argc == 2)
    {
      Ros_HandleSystemStatus(id);
    }
    else
    {
      RosCmdContext ctx = {id, "system", "status"};
      Ros_PrintErr(&ctx, "param_error", 0);
    }
    return TRUE;
  }

  if ((strcmp(argv[1], "mtor1") == 0) || (strcmp(argv[1], "mtor2") == 0))
  {
    Ros_HandleStepper(id, argv[1], argc - 2, &argv[2]);
  }
  else if (strcmp(argv[1], "clamp") == 0)
  {
    Ros_HandleClamp(id, argc - 2, &argv[2]);
  }
  else if ((strcmp(argv[1], "vacum") == 0) || (strcmp(argv[1], "vacuum") == 0))
  {
    Ros_HandleVacum(id, argc - 2, &argv[2]);
  }
  else
  {
    RosCmdContext ctx = {id, "unknown", "none"};
    Ros_PrintErr(&ctx, "unknown_device", argv[1]);
  }

  return TRUE;
}

void RosProtocol_ReportStepperDone(uint32_t id, const char *dev,
                                   const char *cmd, int32_t rev_0p1)
{
  printf("\n@done id=%lu dev=%s cmd=%s result=ok rev=%ld",
         (unsigned long)id, dev, cmd, (long)rev_0p1);
}
