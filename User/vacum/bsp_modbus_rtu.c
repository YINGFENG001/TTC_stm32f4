#include "./vacum/bsp_modbus_rtu.h"
#include "./rs485/bsp_rs485.h"

#define MODBUS_MAX_REGS             16
#define MODBUS_TX_TIMEOUT_MS        80
#define MODBUS_RX_TIMEOUT_MS        120
#define MODBUS_FUNC_READ_HOLDING    0x03
#define MODBUS_FUNC_READ_INPUT      0x04
#define MODBUS_FUNC_WRITE_MULTI     0x10

#ifndef TRUE
#define TRUE                        1
#endif

#ifndef FALSE
#define FALSE                       0
#endif

static uint16_t Modbus_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc;
  uint16_t i;
  uint8_t j;

  crc = 0xFFFF;
  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0; j < 8; j++)
    {
      if ((crc & 0x0001) != 0)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static void Modbus_AppendCrc(uint8_t *frame, uint16_t len_without_crc)
{
  uint16_t crc;

  crc = Modbus_Crc16(frame, len_without_crc);
  frame[len_without_crc] = (uint8_t)(crc & 0xFF);
  frame[len_without_crc + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

static uint8_t Modbus_CheckCrc(const uint8_t *frame, uint16_t len)
{
  uint16_t crc;
  uint16_t recv_crc;

  if (len < 4)
  {
    return FALSE;
  }

  crc = Modbus_Crc16(frame, (uint16_t)(len - 2));
  recv_crc = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);

  return (crc == recv_crc) ? TRUE : FALSE;
}

void Modbus_RTU_Init(void)
{
  RS485_UART_Config();
}

static ModbusResult Modbus_ReadRegisters(uint8_t slave_id, uint8_t func,
                                         uint16_t address, uint16_t quantity,
                                         uint16_t *regs)
{
  uint8_t tx[8];
  uint8_t rx[5 + MODBUS_MAX_REGS * 2];
  uint16_t expected_len;
  uint16_t i;

  if ((regs == 0) || (quantity == 0) || (quantity > MODBUS_MAX_REGS))
  {
    return MODBUS_PARAM_ERROR;
  }

  tx[0] = slave_id;
  tx[1] = func;
  tx[2] = (uint8_t)(address >> 8);
  tx[3] = (uint8_t)(address & 0xFF);
  tx[4] = (uint8_t)(quantity >> 8);
  tx[5] = (uint8_t)(quantity & 0xFF);
  Modbus_AppendCrc(tx, 6);

  RS485_UART_FlushRx();
  if (RS485_UART_Send(tx, sizeof(tx), MODBUS_TX_TIMEOUT_MS) != HAL_OK)
  {
    return MODBUS_UART_ERROR;
  }

  expected_len = (uint16_t)(5 + quantity * 2);
  if (RS485_UART_Recv(rx, expected_len, MODBUS_RX_TIMEOUT_MS) != HAL_OK)
  {
    return MODBUS_TIMEOUT;
  }

  if (Modbus_CheckCrc(rx, expected_len) != TRUE)
  {
    return MODBUS_CRC_ERROR;
  }
  if (rx[0] != slave_id)
  {
    return MODBUS_ID_ERROR;
  }
  if ((rx[1] & 0x80) != 0)
  {
    return MODBUS_EXCEPTION;
  }
  if (rx[1] != func)
  {
    return MODBUS_FUNC_ERROR;
  }
  if (rx[2] != (uint8_t)(quantity * 2))
  {
    return MODBUS_LENGTH_ERROR;
  }

  for (i = 0; i < quantity; i++)
  {
    regs[i] = ((uint16_t)rx[3 + i * 2] << 8) | rx[4 + i * 2];
  }

  return MODBUS_OK;
}

ModbusResult Modbus_ReadHoldingRegisters(uint8_t slave_id, uint16_t address,
                                         uint16_t quantity, uint16_t *regs)
{
  return Modbus_ReadRegisters(slave_id,
                              MODBUS_FUNC_READ_HOLDING,
                              address,
                              quantity,
                              regs);
}

ModbusResult Modbus_ReadInputRegisters(uint8_t slave_id, uint16_t address,
                                       uint16_t quantity, uint16_t *regs)
{
  return Modbus_ReadRegisters(slave_id,
                              MODBUS_FUNC_READ_INPUT,
                              address,
                              quantity,
                              regs);
}

ModbusResult Modbus_WriteMultipleRegisters(uint8_t slave_id, uint16_t address,
                                           const uint16_t *regs, uint16_t quantity)
{
  uint8_t tx[9 + MODBUS_MAX_REGS * 2];
  uint8_t rx[8];
  uint16_t tx_len;
  uint16_t i;

  if ((regs == 0) || (quantity == 0) || (quantity > MODBUS_MAX_REGS))
  {
    return MODBUS_PARAM_ERROR;
  }

  tx[0] = slave_id;
  tx[1] = MODBUS_FUNC_WRITE_MULTI;
  tx[2] = (uint8_t)(address >> 8);
  tx[3] = (uint8_t)(address & 0xFF);
  tx[4] = (uint8_t)(quantity >> 8);
  tx[5] = (uint8_t)(quantity & 0xFF);
  tx[6] = (uint8_t)(quantity * 2);

  for (i = 0; i < quantity; i++)
  {
    tx[7 + i * 2] = (uint8_t)(regs[i] >> 8);
    tx[8 + i * 2] = (uint8_t)(regs[i] & 0xFF);
  }

  tx_len = (uint16_t)(9 + quantity * 2);
  Modbus_AppendCrc(tx, (uint16_t)(tx_len - 2));

  RS485_UART_FlushRx();
  if (RS485_UART_Send(tx, tx_len, MODBUS_TX_TIMEOUT_MS) != HAL_OK)
  {
    return MODBUS_UART_ERROR;
  }

  if (RS485_UART_Recv(rx, sizeof(rx), MODBUS_RX_TIMEOUT_MS) != HAL_OK)
  {
    return MODBUS_TIMEOUT;
  }

  if (Modbus_CheckCrc(rx, sizeof(rx)) != TRUE)
  {
    return MODBUS_CRC_ERROR;
  }
  if (rx[0] != slave_id)
  {
    return MODBUS_ID_ERROR;
  }
  if ((rx[1] & 0x80) != 0)
  {
    return MODBUS_EXCEPTION;
  }
  if (rx[1] != MODBUS_FUNC_WRITE_MULTI)
  {
    return MODBUS_FUNC_ERROR;
  }
  if ((((uint16_t)rx[2] << 8) | rx[3]) != address)
  {
    return MODBUS_LENGTH_ERROR;
  }
  if ((((uint16_t)rx[4] << 8) | rx[5]) != quantity)
  {
    return MODBUS_LENGTH_ERROR;
  }

  return MODBUS_OK;
}

const char *Modbus_ResultName(ModbusResult result)
{
  switch (result)
  {
    case MODBUS_OK:           return "ok";
    case MODBUS_UART_ERROR:   return "uart_error";
    case MODBUS_TIMEOUT:      return "timeout";
    case MODBUS_CRC_ERROR:    return "crc_error";
    case MODBUS_ID_ERROR:     return "id_error";
    case MODBUS_FUNC_ERROR:   return "func_error";
    case MODBUS_LENGTH_ERROR: return "length_error";
    case MODBUS_EXCEPTION:    return "exception";
    case MODBUS_PARAM_ERROR:  return "param_error";
    default:                  return "unknown";
  }
}
