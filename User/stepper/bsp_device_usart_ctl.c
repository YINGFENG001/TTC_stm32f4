/**
  ******************************************************************************
 * @file    bsp_device_usart_ctl.c
 * @brief   Multi-device serial command dispatcher.
  ******************************************************************************
  */

#include "./stepper/bsp_device_usart_ctl.h"
#include "./stepper/bsp_device_context.h"
#include "./stepper/bsp_stepper_ctl.h"
#include "./gripper/bsp_clamp_ctl.h"
#include "./vacum/bsp_vacum_ctl.h"
#include "./ros/bsp_ros_protocol.h"

EndDevice devices[DEVICE_NUM] = {
  {DEVICE_MTOR1, "mtor1", DEVICE_TYPE_STEPPER, DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0},
  {DEVICE_MTOR2, "mtor2", DEVICE_TYPE_STEPPER, DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0},
  {DEVICE_CLAMP, "clamp", DEVICE_TYPE_SERVO,   DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0},
  {DEVICE_VACUM, "vacum", DEVICE_TYPE_VACUM,   DEV_IDLE,      TRUE, 0, 0, 0, 0, FALSE, 0, 0}
};

static int Device_FindByName(const char *name);
static void Device_PrintFullStatus(void);
static void ShowCommandHelp(void);
static void Command_Dispatch(char *line);

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

void PrintFixed1Signed(int32_t value)
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

void Device_Task(void)
{
  uint8_t i;
  uint32_t now;
  StepperRuntimeSnapshot stepper_snapshot;

  now = HAL_GetTick();
  Stepper_ApplyMechanicalConfig();

  for (i = 0; i < STEPPER_NUM; i++)
  {
    if (Stepper_GetRuntimeSnapshot(i, &stepper_snapshot) != TRUE)
    {
      continue;
    }

    devices[i].position = Stepper_StepToRev0p1(i, stepper_snapshot.position_steps);
    devices[i].enabled = stepper_snapshot.out_ena;

    if (stepper_snapshot.out_ena != TRUE)
    {
      devices[i].state = DEV_DISABLED;
    }
    else if (stepper_snapshot.running == TRUE)
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

void Device_PrintStatus(uint8_t id)
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

static void ShowCommandHelp(void)
{
  printf("\n命令说明:");
  printf("\n  mtor1/2 move [rev: -10000~10000(0.1圈), +0/-0 continuous]");
  printf("\n  mtor1/2 stop");
  printf("\n  mtor1/2 rpm [value]");
  printf("\n  mtor1/2 set [accel] [decel] [gear] [micro]");
  printf("\n  mtor1/2 status");
  printf("\n  clamp ping [id: 0~253]");
  printf("\n  clamp status [id: 0~253]");
  printf("\n  clamp readreg [addr: 0~255]");
  printf("\n  clamp open");
  printf("\n  clamp close");
  printf("\n  clamp move [openPos/Pct: 700~2048 or measured 0%%~100%%]");
  printf("\n  clamp grip [load: 100~900(0.1%%)] [openPos/Pct: 700~2048 or measured 0%%~100%%, optional]");
  printf("\n  clamp release");
  printf("\n  clamp set [speed: 1~3000] [gripStep: 5~100] [releaseDelta: 20~400]");
  printf("\n  vacum set [min_vac:0~100(%%)] [max_vac:0~100(%%)] [timeout:1~255(100ms)]");
  printf("\n  vacum grip");
  printf("\n  vacum release");
  printf("\n  vacum stop");
  printf("\n  vacum status");
  printf("\n  status");
  printf("\n示例: mtor1 move 50 -> 5.0圈 CW; mtor1 move -50 -> 5.0圈 CCW; mtor1 move +0 -> continuous CW until stop; continuous中rpm N=CW, rpm -N=CCW, rpm 0减速停机");
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
  printf("\n       rpm   = output rpm");
  printf("\n       set   = accel/decel(rpm/s), gear xx:1, micro");
  printf("\n       stop  = 立即停止当前运动并保持当前位置");
  printf("\n  clamp position = 700~2048");
  printf("\n        move     = openPos/Pct, e.g. 700 or measured 100%%");
  printf("\n        default  = speed 1000, gripStep 30, releaseDelta 100");
  printf("\n        load     = 0.1%%");
  printf("\n        current  = 6.5mA");
  printf("\n  fast check = 200ms, vacum drop hits = 5");
  printf("\n  vacum min/max_vac = 真空度(%%)");
  printf("\n        timeout     = 1~255(100ms)");
	printf("\n        grip        = 真空吸取");
	printf("\n        release     = 破真空停止");
  printf("\n        stop        = 直接停止");
  printf("\n输入 ? 查看详细命令说明，输入 status 查看全部设备状态\n");
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

static int Command_ParseArgv(char *line, char *argv[], int max_argc)
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

static void Command_DispatchDevice(int device_id, int argc, char *argv[])
{
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

static void Command_Dispatch(char *line)
{
  char *argv[8];
  int argc;
  int device_id;

  argc = Command_ParseArgv(line, argv, 8);
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
    Device_PrintFullStatus();
    return;
  }
  device_id = Device_FindByName(argv[0]);
  if (device_id < 0)
  {
    printf("\nunknown device: %s", argv[0]);
    return;
  }
  Command_DispatchDevice(device_id, argc, argv);
}

/**
  * @brief  处理串口接收到的数据，只分发任务，不等待设备完成
  * @param  无
  * @retval 无
  */

void DealSerialData(void)
{
  static char showflag = 1;
  char cmd_line[UART_RX_BUFFER_SIZE];

  Stepper_ApplyMechanicalConfig();

  if (showflag)
  {
    showflag = 0;
    return;
  }

  if (DebugUsart_PopCommand(cmd_line, sizeof(cmd_line)) == TRUE)
  {
    if (RosProtocol_TryDispatch(cmd_line) != TRUE)
    {
      Command_Dispatch(cmd_line);
    }
  }
}
