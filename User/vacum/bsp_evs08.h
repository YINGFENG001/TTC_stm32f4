#ifndef __BSP_EVS08_H
#define __BSP_EVS08_H

#include "stm32f4xx.h"
#include "./vacum/bsp_modbus_rtu.h"

#define EVS08_DEFAULT_SLAVE_ID          9
#define EVS08_DEFAULT_MAX_VAC_PERCENT   70
#define EVS08_DEFAULT_MIN_VAC_PERCENT   30
#define EVS08_DEFAULT_TIMEOUT           10

typedef struct {
  uint16_t ch1_status_reg;
  uint16_t fault_reg;
  uint16_t ch1_pressure_reg;
  uint16_t ch2_status_reg;
  uint16_t ch2_pressure_reg;
  uint8_t ch1_enabled;
  uint8_t ch2_enabled;
  uint8_t ch1_busy;
  uint8_t ch2_busy;
  uint8_t ch1_obj;
  uint8_t ch2_obj;
  uint8_t ch1_vac_percent;
  uint8_t ch2_vac_percent;
  uint8_t temperature;
  uint16_t bus_voltage_x10;
} Evs08Status;

void Evs08_Init(void);
ModbusResult Evs08_SetParams(uint8_t min_vac_percent, uint8_t max_vac_percent,
                             uint8_t timeout);
ModbusResult Evs08_Grip(void);
ModbusResult Evs08_Release(void);
ModbusResult Evs08_Stop(void);
ModbusResult Evs08_ReadStatus(Evs08Status *status);
const char *Evs08_ResultName(ModbusResult result);

#endif
