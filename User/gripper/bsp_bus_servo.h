#ifndef __BSP_BUS_SERVO_H
#define __BSP_BUS_SERVO_H

#include "stm32f4xx.h"

#define BUS_SERVO_DEFAULT_ID                 10
#define BUS_SERVO_BROADCAST_ID              0xFE

#define BUS_SERVO_INST_PING                 0x01
#define BUS_SERVO_INST_READ_DATA            0x02
#define BUS_SERVO_INST_WRITE_DATA           0x03
#define BUS_SERVO_INST_SYNC_WRITE           0x83

#define BUS_SERVO_ADDR_PRESENT_POSITION     0x38
#define BUS_SERVO_ADDR_STATUS               0x41
#define BUS_SERVO_ADDR_CURRENT              0x45
#define BUS_SERVO_ADDR_PROTECTION_TORQUE    0x22
#define BUS_SERVO_ADDR_PROTECTION_TIME      0x23
#define BUS_SERVO_ADDR_OVERLOAD_TORQUE      0x24
#define BUS_SERVO_ADDR_TORQUE_ENABLE        0x28
#define BUS_SERVO_ADDR_GOAL_POSITION        0x2A
#define BUS_SERVO_ADDR_TORQUE_LIMIT         0x30
#define BUS_SERVO_MOVE_PWM_MODE             0x1000

typedef enum {
  BUS_SERVO_OK = 0,
  BUS_SERVO_UART_ERROR,
  BUS_SERVO_TIMEOUT,
  BUS_SERVO_HEADER_ERROR,
  BUS_SERVO_ID_ERROR,
  BUS_SERVO_LENGTH_ERROR,
  BUS_SERVO_CHECKSUM_ERROR,
  BUS_SERVO_STATUS_ERROR,
  BUS_SERVO_PARAM_ERROR
} BusServoResult;

typedef struct {
  int16_t position;
  int16_t speed;
  int16_t load;
  uint8_t voltage;
  uint8_t temperature;
  uint8_t status;
  uint8_t moving;
  int16_t current;
} BusServoStatus;

void BusServo_Init(void);
BusServoResult BusServo_Ping(uint8_t servo_id, uint8_t *servo_error);
BusServoResult BusServo_ReadData(uint8_t servo_id, uint8_t address, uint8_t len,
                                 uint8_t *data, uint8_t *servo_error);
BusServoResult BusServo_WriteData(uint8_t servo_id, uint8_t address, const uint8_t *data,
                                  uint8_t len, uint8_t *servo_error);
BusServoResult BusServo_ReadStatus(uint8_t servo_id, BusServoStatus *status, uint8_t *servo_error);
BusServoResult BusServo_ReadServoState(uint8_t servo_id, uint8_t *state, uint8_t *servo_error);
BusServoResult BusServo_ReadCurrent(uint8_t servo_id, int16_t *current, uint8_t *servo_error);
BusServoResult BusServo_SetTorqueEnable(uint8_t servo_id, uint8_t enable);
BusServoResult BusServo_SyncWrite(uint8_t address, uint8_t data_len,
                                  const uint8_t *data, uint8_t data_len_total);
BusServoResult BusServo_MoveRaw(uint8_t servo_id, uint16_t position, uint16_t speed);
const char *BusServo_ResultName(BusServoResult result);

#endif
