#include "./vacum/bsp_evs08.h"

#define EVS08_REG_CONTROL       0x03E8
#define EVS08_REG_STATUS        0x07D0
#define EVS08_PARAM_REG_COUNT   6
#define EVS08_STATUS_REG_COUNT  8

#define EVS08_CTRL_ACT          0x0001
#define EVS08_CTRL_MODE_CH1     0x0002
#define EVS08_CTRL_ATR_CH1      0x0004
#define EVS08_CTRL_GTO_CH1      0x0008
#define EVS08_CTRL_MODE_CH2     0x0010
#define EVS08_CTRL_ATR_CH2      0x0020
#define EVS08_CTRL_GTO_CH2      0x0040

#define EVS08_CTRL_IDLE         (EVS08_CTRL_ACT | EVS08_CTRL_MODE_CH1 | EVS08_CTRL_MODE_CH2)
#define EVS08_CTRL_GRIP         (EVS08_CTRL_IDLE | EVS08_CTRL_GTO_CH1 | EVS08_CTRL_GTO_CH2)
#define EVS08_CTRL_RELEASE      (EVS08_CTRL_IDLE | EVS08_CTRL_ATR_CH1 | EVS08_CTRL_ATR_CH2)

static uint8_t evs08_slave_id = EVS08_DEFAULT_SLAVE_ID;
static uint8_t evs08_max_vac = EVS08_DEFAULT_MAX_VAC_PERCENT;
static uint8_t evs08_min_vac = EVS08_DEFAULT_MIN_VAC_PERCENT;
static uint8_t evs08_timeout = EVS08_DEFAULT_TIMEOUT;

static uint8_t Evs08_ClampPercent(uint8_t value)
{
  if (value > 100)
  {
    return 100;
  }
  return value;
}

static uint8_t Evs08_PercentToReg(uint8_t vac_percent)
{
  vac_percent = Evs08_ClampPercent(vac_percent);
  return (uint8_t)(100 - vac_percent);
}

static uint8_t Evs08_ClampTimeout(uint8_t timeout)
{
  if (timeout == 0)
  {
    return 1;
  }
  return timeout;
}

static uint8_t Evs08_RegToPercent(uint8_t reg_value)
{
  if (reg_value > 100)
  {
    return 0;
  }
  return (uint8_t)(100 - reg_value);
}

static ModbusResult Evs08_WriteControl(uint16_t control)
{
  return Modbus_WriteMultipleRegisters(evs08_slave_id, EVS08_REG_CONTROL, &control, 1);
}

static ModbusResult Evs08_WriteParams(uint16_t control)
{
  uint16_t regs[EVS08_PARAM_REG_COUNT];
  uint8_t max_reg;
  uint8_t min_reg;
  uint8_t timeout_reg;

  max_reg = Evs08_PercentToReg(evs08_max_vac);
  min_reg = Evs08_PercentToReg(evs08_min_vac);
  timeout_reg = Evs08_ClampTimeout(evs08_timeout);

  regs[0] = control;
  regs[1] = (uint16_t)max_reg << 8;
  regs[2] = ((uint16_t)min_reg << 8) | timeout_reg;
  regs[3] = ((uint16_t)timeout_reg << 8) | max_reg;
  regs[4] = min_reg;
  regs[5] = 0x0000;

  return Modbus_WriteMultipleRegisters(evs08_slave_id,
                                       EVS08_REG_CONTROL,
                                       regs,
                                       EVS08_PARAM_REG_COUNT);
}

void Evs08_Init(void)
{
  Modbus_RTU_Init();
  (void)Evs08_SetParams(evs08_min_vac, evs08_max_vac, evs08_timeout);
}

ModbusResult Evs08_SetParams(uint8_t min_vac_percent, uint8_t max_vac_percent,
                             uint8_t timeout)
{
  evs08_max_vac = Evs08_ClampPercent(max_vac_percent);
  evs08_min_vac = Evs08_ClampPercent(min_vac_percent);
  evs08_timeout = Evs08_ClampTimeout(timeout);

  return Evs08_WriteParams(EVS08_CTRL_IDLE);
}

ModbusResult Evs08_Grip(void)
{
  ModbusResult result;

  result = Evs08_WriteParams(EVS08_CTRL_GRIP);
  if (result != MODBUS_OK)
  {
    return result;
  }

  return MODBUS_OK;
}

ModbusResult Evs08_Release(void)
{
  return Evs08_WriteParams(EVS08_CTRL_RELEASE);
}

ModbusResult Evs08_Stop(void)
{
  return Evs08_WriteControl(EVS08_CTRL_IDLE);
}

ModbusResult Evs08_ReadStatus(Evs08Status *status)
{
  uint16_t regs[EVS08_STATUS_REG_COUNT];
  ModbusResult result;

  if (status == 0)
  {
    return MODBUS_PARAM_ERROR;
  }

  result = Modbus_ReadInputRegisters(evs08_slave_id,
                                     EVS08_REG_STATUS,
                                     EVS08_STATUS_REG_COUNT,
                                     regs);
  if (result != MODBUS_OK)
  {
    return result;
  }

  status->ch1_status_reg = regs[0];
  status->fault_reg = regs[1];
  status->ch1_pressure_reg = regs[2];
  status->ch2_status_reg = regs[3];
  status->ch2_pressure_reg = regs[5];
  status->ch1_enabled = (regs[0] & 0x0001) ? 1 : 0;
  status->ch2_enabled = (regs[3] & 0x0001) ? 1 : 0;
  status->ch1_busy = (regs[0] & 0x0008) ? 1 : 0;
  status->ch2_busy = (regs[3] & 0x0008) ? 1 : 0;
  status->ch1_obj = (uint8_t)((regs[0] >> 6) & 0x03);
  status->ch2_obj = (uint8_t)((regs[3] >> 6) & 0x03);
  status->ch1_vac_percent = Evs08_RegToPercent((uint8_t)((regs[2] >> 8) & 0x00FF));
  status->ch2_vac_percent = Evs08_RegToPercent((uint8_t)((regs[5] >> 8) & 0x00FF));
  status->temperature = (uint8_t)((regs[6] >> 8) & 0x00FF);
  status->bus_voltage_x10 = regs[7];

  return MODBUS_OK;
}

const char *Evs08_ResultName(ModbusResult result)
{
  return Modbus_ResultName(result);
}
