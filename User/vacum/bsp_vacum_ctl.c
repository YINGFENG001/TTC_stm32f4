/**
  ******************************************************************************
 * @file    bsp_vacum_ctl.c
 * @brief   Vacuum gripper command, drop detection and API implementation.
  ******************************************************************************
  */

#include "./vacum/bsp_vacum_ctl.h"

typedef struct {
  uint8_t active;
  uint8_t drop_hits;
  uint8_t comm_fail_count;
  Evs08Status last_status;
} VacumHoldContext;

#define VACUM_DROP_PERCENT_LIMIT       5U
#define VACUM_COMM_FAIL_LIMIT          2U

static VacumHoldContext vacum_hold = {FALSE, 0, 0, {0}};

static void Vacum_HoldBegin(void);
static void Vacum_HoldEnd(void);
static DeviceApiResult DeviceApi_FromModbusResult(ModbusResult result);
static void DeviceApi_FillVacumStatus(const Evs08Status *src, DeviceVacumStatus *out);
static void Vacum_PrintStatusFields(const Evs08Status *status_data);
static uint8_t Vacum_ParsePercent(char *text, uint8_t *value);
static uint8_t Vacum_ParseTimeout(char *text, uint8_t *value);
static void Vacum_PrintResult(const char *action, ModbusResult result);

static void Vacum_HoldBegin(void)
{
  vacum_hold.active = TRUE;
  vacum_hold.drop_hits = 0;
  vacum_hold.comm_fail_count = 0;
  devices[DEVICE_VACUM].state = DEV_RUNNING;
}

static void Vacum_HoldEnd(void)
{
  vacum_hold.active = FALSE;
  vacum_hold.drop_hits = 0;
  vacum_hold.comm_fail_count = 0;
}

void Vacum_MonitorTask(uint32_t now)
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
    vacum_hold.comm_fail_count++;
    if (vacum_hold.comm_fail_count < VACUM_COMM_FAIL_LIMIT)
    {
      printf("\n@warn dev=vacum event=status_error result=%s fail_count=%u action=retry",
             Evs08_ResultName(result),
             vacum_hold.comm_fail_count);
      return;
    }

    devices[DEVICE_VACUM].state = DEV_ERROR;
    devices[DEVICE_VACUM].error_code = result;
    Vacum_HoldEnd();
    (void)Evs08_Stop();
    printf("\n@fault dev=vacum event=status_error result=%s fail_count=%u action=stop",
           Evs08_ResultName(result),
           VACUM_COMM_FAIL_LIMIT);
    return;
  }

  vacum_hold.comm_fail_count = 0;
  vacum_hold.last_status = status_data;
  devices[DEVICE_VACUM].enabled = (status_data.ch1_enabled || status_data.ch2_enabled) ? TRUE : FALSE;
  devices[DEVICE_VACUM].value = status_data.ch1_status_reg;
  devices[DEVICE_VACUM].error_code = 0;
  devices[DEVICE_VACUM].state = DEV_RUNNING;

  if ((status_data.ch1_vac_percent <= VACUM_DROP_PERCENT_LIMIT) &&
      (status_data.ch2_vac_percent <= VACUM_DROP_PERCENT_LIMIT))
  {
    vacum_hold.drop_hits++;
    if (vacum_hold.drop_hits >= VACUM_DROP_HIT_LIMIT)
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

static void Vacum_CommandSet(uint8_t device_id, int argc, char *argv[])
{
  uint8_t max_vac;
  uint8_t min_vac;
  uint8_t timeout;
  ModbusResult result;

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
    return;
  }
  devices[device_id].state = DEV_ERROR;
  devices[device_id].error_code = result;
}

static void Vacum_CommandGrip(uint8_t device_id, int argc)
{
  ModbusResult result;

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
}

static void Vacum_CommandRelease(uint8_t device_id, int argc)
{
  ModbusResult result;

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
}

static void Vacum_CommandStop(uint8_t device_id, int argc)
{
  ModbusResult result;

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
}

static void Vacum_CommandStatus(uint8_t device_id, int argc)
{
  ModbusResult result;
  Evs08Status status_data;

  if (argc != 2)
  {
    printf("\nvacum status param_error");
    return;
  }

  result = Evs08_ReadStatus(&status_data);
  Vacum_PrintResult("status", result);
  if (result != MODBUS_OK)
  {
    devices[device_id].state = DEV_ERROR;
    devices[device_id].error_code = result;
    return;
  }

  Vacum_PrintStatusFields(&status_data);
  devices[device_id].enabled = (status_data.ch1_enabled || status_data.ch2_enabled) ? TRUE : FALSE;
  devices[device_id].value = status_data.ch1_status_reg;
  devices[device_id].error_code = 0;
  devices[device_id].state = (vacum_hold.active == TRUE) ? DEV_RUNNING :
                              ((status_data.ch1_busy || status_data.ch2_busy) ? DEV_RUNNING : DEV_IDLE);
}

void Vacum_Command(uint8_t device_id, int argc, char *argv[])
{
  if (argc < 2)
  {
    printf("\n%s missing action", devices[device_id].name);
    return;
  }
  if (strcmp(argv[1], "set") == 0)
  {
    Vacum_CommandSet(device_id, argc, argv);
    return;
  }
  if (strcmp(argv[1], "grip") == 0)
  {
    Vacum_CommandGrip(device_id, argc);
    return;
  }
  if (strcmp(argv[1], "release") == 0)
  {
    Vacum_CommandRelease(device_id, argc);
    return;
  }
  if (strcmp(argv[1], "stop") == 0)
  {
    Vacum_CommandStop(device_id, argc);
    return;
  }
  if (strcmp(argv[1], "status") == 0)
  {
    Vacum_CommandStatus(device_id, argc);
    return;
  }

  printf("\n%s unknown action: %s", devices[device_id].name, argv[1]);
}
