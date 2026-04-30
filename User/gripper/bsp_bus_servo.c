#include "./gripper/bsp_bus_servo.h"
#include "./gripper/bsp_gripper_uart.h"
#include "./delay/core_delay.h"

#define BUS_SERVO_HEADER                    0xFF
#define BUS_SERVO_MAX_PARAM_LEN             16
#define BUS_SERVO_TX_TIMEOUT_MS             20
#define BUS_SERVO_RX_TIMEOUT_MS             30

static uint8_t BusServo_Checksum(uint8_t id, uint8_t length, uint8_t instruction, const uint8_t *params)
{
  uint16_t sum;
  uint8_t i;

  sum = (uint16_t)id + length + instruction;
  for (i = 0; i < (uint8_t)(length - 2); i++)
  {
    sum += params[i];
  }

  return (uint8_t)(~sum);
}

static BusServoResult BusServo_SendPacket(uint8_t id, uint8_t instruction,
                                          const uint8_t *params, uint8_t param_len)
{
  uint8_t packet[BUS_SERVO_MAX_PARAM_LEN + 6];
  uint8_t length;
  uint8_t i;

  if (param_len > BUS_SERVO_MAX_PARAM_LEN)
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  length = (uint8_t)(param_len + 2);
  packet[0] = BUS_SERVO_HEADER;
  packet[1] = BUS_SERVO_HEADER;
  packet[2] = id;
  packet[3] = length;
  packet[4] = instruction;

  for (i = 0; i < param_len; i++)
  {
    packet[5 + i] = params[i];
  }
  packet[5 + param_len] = BusServo_Checksum(id, length, instruction, params);

  Gripper_UART_FlushRx();
  if (Gripper_UART_Send(packet, (uint16_t)(param_len + 6), BUS_SERVO_TX_TIMEOUT_MS) != HAL_OK)
  {
    return BUS_SERVO_UART_ERROR;
  }

  return BUS_SERVO_OK;
}

static BusServoResult BusServo_RecvPacket(uint8_t expected_id, uint8_t *params,
                                          uint8_t expected_param_len, uint8_t *servo_error)
{
  uint8_t byte;
  uint8_t id;
  uint8_t length;
  uint8_t recv_buf[BUS_SERVO_MAX_PARAM_LEN + 2];
  uint8_t calc_checksum;
  uint8_t recv_checksum;
  uint8_t param_len;
  uint8_t i;

  if (servo_error != 0)
  {
    *servo_error = 0;
  }

  /* 应答包按 FF FF ID LEN ERR PARAM... CHKSUM 顺序解析，任何一段异常都直接返回。 */
  if (Gripper_UART_Recv(&byte, 1, BUS_SERVO_RX_TIMEOUT_MS) != HAL_OK)
  {
    return BUS_SERVO_TIMEOUT;
  }
  if (byte != BUS_SERVO_HEADER)
  {
    return BUS_SERVO_HEADER_ERROR;
  }

  if (Gripper_UART_Recv(&byte, 1, BUS_SERVO_RX_TIMEOUT_MS) != HAL_OK)
  {
    return BUS_SERVO_TIMEOUT;
  }
  if (byte != BUS_SERVO_HEADER)
  {
    return BUS_SERVO_HEADER_ERROR;
  }

  if (Gripper_UART_Recv(&id, 1, BUS_SERVO_RX_TIMEOUT_MS) != HAL_OK)
  {
    return BUS_SERVO_TIMEOUT;
  }
  if (id != expected_id)
  {
    return BUS_SERVO_ID_ERROR;
  }

  if (Gripper_UART_Recv(&length, 1, BUS_SERVO_RX_TIMEOUT_MS) != HAL_OK)
  {
    return BUS_SERVO_TIMEOUT;
  }
  if ((length < 2) || (length > (BUS_SERVO_MAX_PARAM_LEN + 2)))
  {
    return BUS_SERVO_LENGTH_ERROR;
  }

  if (Gripper_UART_Recv(recv_buf, length, BUS_SERVO_RX_TIMEOUT_MS) != HAL_OK)
  {
    return BUS_SERVO_TIMEOUT;
  }

  param_len = (uint8_t)(length - 2);
  if (param_len != expected_param_len)
  {
    return BUS_SERVO_LENGTH_ERROR;
  }

  if (servo_error != 0)
  {
    *servo_error = recv_buf[0];
  }

  recv_checksum = recv_buf[length - 1];
  calc_checksum = BusServo_Checksum(id, length, recv_buf[0], &recv_buf[1]);
  if (calc_checksum != recv_checksum)
  {
    return BUS_SERVO_CHECKSUM_ERROR;
  }

  if (recv_buf[0] != 0)
  {
    return BUS_SERVO_STATUS_ERROR;
  }

  for (i = 0; i < param_len; i++)
  {
    params[i] = recv_buf[1 + i];
  }

  return BUS_SERVO_OK;
}

static int16_t BusServo_ReadInt16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t BusServo_DecodePresentLoad(const uint8_t *data)
{
  uint16_t raw_value;
  int16_t magnitude;

  raw_value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  magnitude = (int16_t)(raw_value & 0x03FF);

  if ((raw_value & 0x0400) != 0)
  {
    return (int16_t)(-magnitude);
  }

  return magnitude;
}

static void BusServo_WriteUint16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
}

void BusServo_Init(void)
{
  HAL_InitTick(0);
  Gripper_UART_Config();
}

BusServoResult BusServo_Ping(uint8_t servo_id, uint8_t *servo_error)
{
  BusServoResult result;

  result = BusServo_SendPacket(servo_id, BUS_SERVO_INST_PING, 0, 0);
  if (result != BUS_SERVO_OK)
  {
    return result;
  }

  return BusServo_RecvPacket(servo_id, 0, 0, servo_error);
}

BusServoResult BusServo_ReadData(uint8_t servo_id, uint8_t address, uint8_t len,
                                 uint8_t *data, uint8_t *servo_error)
{
  uint8_t params[2];
  BusServoResult result;

  if ((data == 0) || (len > BUS_SERVO_MAX_PARAM_LEN))
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  params[0] = address;
  params[1] = len;

  result = BusServo_SendPacket(servo_id, BUS_SERVO_INST_READ_DATA, params, 2);
  if (result != BUS_SERVO_OK)
  {
    return result;
  }

  return BusServo_RecvPacket(servo_id, data, len, servo_error);
}

BusServoResult BusServo_WriteData(uint8_t servo_id, uint8_t address, const uint8_t *data,
                                  uint8_t len, uint8_t *servo_error)
{
  uint8_t params[BUS_SERVO_MAX_PARAM_LEN];
  uint8_t i;
  BusServoResult result;

  if ((data == 0) || (len == 0) || ((uint8_t)(len + 1) > BUS_SERVO_MAX_PARAM_LEN))
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  params[0] = address;
  for (i = 0; i < len; i++)
  {
    params[1 + i] = data[i];
  }

  result = BusServo_SendPacket(servo_id, BUS_SERVO_INST_WRITE_DATA, params, (uint8_t)(len + 1));
  if (result != BUS_SERVO_OK)
  {
    return result;
  }

  if (servo_id == BUS_SERVO_BROADCAST_ID)
  {
    return BUS_SERVO_OK;
  }

  return BusServo_RecvPacket(servo_id, 0, 0, servo_error);
}

BusServoResult BusServo_ReadStatus(uint8_t servo_id, BusServoStatus *status, uint8_t *servo_error)
{
  uint8_t data[8];
  BusServoResult result;

  if (status == 0)
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  /* 运动控制依赖的基础状态只读取 0x38 开始的 8 字节，避免附加寄存器读超时影响动作判定。 */
  result = BusServo_ReadData(servo_id, BUS_SERVO_ADDR_PRESENT_POSITION, 8, data, servo_error);
  if (result != BUS_SERVO_OK)
  {
    return result;
  }

  status->position = BusServo_ReadInt16(&data[0]);
  status->speed = BusServo_ReadInt16(&data[2]);
  status->load = BusServo_DecodePresentLoad(&data[4]);
  status->voltage = data[6];
  status->temperature = data[7];
  status->moving = 0;
  status->status = 0;
  status->current = 0;

  return BUS_SERVO_OK;
}

BusServoResult BusServo_ReadServoState(uint8_t servo_id, uint8_t *state, uint8_t *servo_error)
{
  uint8_t data;
  BusServoResult result;

  if (state == 0)
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  result = BusServo_ReadData(servo_id, BUS_SERVO_ADDR_STATUS, 1, &data, servo_error);
  if (result != BUS_SERVO_OK)
  {
    return result;
  }

  *state = data;
  return BUS_SERVO_OK;
}

BusServoResult BusServo_ReadCurrent(uint8_t servo_id, int16_t *current, uint8_t *servo_error)
{
  uint8_t data[2];
  BusServoResult result;

  if (current == 0)
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  result = BusServo_ReadData(servo_id, BUS_SERVO_ADDR_CURRENT, 2, data, servo_error);
  if (result != BUS_SERVO_OK)
  {
    return result;
  }

  *current = BusServo_ReadInt16(data);
  return BUS_SERVO_OK;
}

BusServoResult BusServo_SetTorqueEnable(uint8_t servo_id, uint8_t enable)
{
  uint8_t value;
  uint8_t servo_error;

  value = (enable != 0) ? 1 : 0;
  servo_error = 0;

  return BusServo_WriteData(servo_id, BUS_SERVO_ADDR_TORQUE_ENABLE, &value, 1, &servo_error);
}

BusServoResult BusServo_SyncWrite(uint8_t address, uint8_t data_len,
                                  const uint8_t *data, uint8_t data_len_total)
{
  uint8_t params[BUS_SERVO_MAX_PARAM_LEN];
  uint8_t i;

  if ((data == 0) || ((uint8_t)(data_len_total + 2) > BUS_SERVO_MAX_PARAM_LEN))
  {
    return BUS_SERVO_PARAM_ERROR;
  }

  params[0] = address;
  params[1] = data_len;
  for (i = 0; i < data_len_total; i++)
  {
    params[2 + i] = data[i];
  }

  return BusServo_SendPacket(BUS_SERVO_BROADCAST_ID,
                             BUS_SERVO_INST_SYNC_WRITE,
                             params,
                             (uint8_t)(data_len_total + 2));
}

BusServoResult BusServo_MoveRaw(uint8_t servo_id, uint16_t position, uint16_t speed)
{
  uint8_t data[7];

  data[0] = servo_id;
  BusServo_WriteUint16(&data[1], position);
  BusServo_WriteUint16(&data[3], BUS_SERVO_MOVE_PWM_MODE);
  BusServo_WriteUint16(&data[5], speed);

  return BusServo_SyncWrite(BUS_SERVO_ADDR_GOAL_POSITION, 6, data, sizeof(data));
}

const char *BusServo_ResultName(BusServoResult result)
{
  switch (result)
  {
    case BUS_SERVO_OK:             return "ok";
    case BUS_SERVO_UART_ERROR:     return "uart_error";
    case BUS_SERVO_TIMEOUT:        return "timeout";
    case BUS_SERVO_HEADER_ERROR:   return "header_error";
    case BUS_SERVO_ID_ERROR:       return "id_error";
    case BUS_SERVO_LENGTH_ERROR:   return "length_error";
    case BUS_SERVO_CHECKSUM_ERROR: return "checksum_error";
    case BUS_SERVO_STATUS_ERROR:   return "servo_status_error";
    case BUS_SERVO_PARAM_ERROR:    return "param_error";
    default:                       return "unknown";
  }
}
