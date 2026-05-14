#ifndef __BSP_MODBUS_RTU_H
#define __BSP_MODBUS_RTU_H

#include "stm32f4xx.h"

typedef enum {
  MODBUS_OK = 0,
  MODBUS_UART_ERROR,
  MODBUS_TIMEOUT,
  MODBUS_CRC_ERROR,
  MODBUS_ID_ERROR,
  MODBUS_FUNC_ERROR,
  MODBUS_LENGTH_ERROR,
  MODBUS_EXCEPTION,
  MODBUS_PARAM_ERROR
} ModbusResult;

void Modbus_RTU_Init(void);
ModbusResult Modbus_ReadHoldingRegisters(uint8_t slave_id, uint16_t address,
                                         uint16_t quantity, uint16_t *regs);
ModbusResult Modbus_ReadInputRegisters(uint8_t slave_id, uint16_t address,
                                       uint16_t quantity, uint16_t *regs);
ModbusResult Modbus_WriteMultipleRegisters(uint8_t slave_id, uint16_t address,
                                           const uint16_t *regs, uint16_t quantity);
const char *Modbus_ResultName(ModbusResult result);

#endif
