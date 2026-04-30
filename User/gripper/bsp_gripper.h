#ifndef __BSP_GRIPPER_H
#define __BSP_GRIPPER_H

#include "stm32f4xx.h"
#include "./gripper/bsp_bus_servo.h"

#define GRIPPER_SERVO_ID_DEFAULT             BUS_SERVO_DEFAULT_ID
#define GRIPPER_POS_OPEN_MAX                 800
#define GRIPPER_POS_CLOSE_MIN                2048
#define GRIPPER_SPEED_DEFAULT                1000
#define GRIPPER_SPEED_MIN                    1
#define GRIPPER_SPEED_MAX                    3000
#define GRIPPER_MOVE_TOLERANCE               10
#define GRIPPER_MOVE_TIMEOUT_MS              3000
#define GRIPPER_POLL_INTERVAL_MS             50
#define GRIPPER_STALL_LOAD_LIMIT             700
#define GRIPPER_STALL_HIT_LIMIT              3
#define GRIPPER_GRIP_LOAD_DEFAULT            400
#define GRIPPER_GRIP_LOAD_MIN                100
#define GRIPPER_GRIP_LOAD_MAX                900
#define GRIPPER_GRIP_STEP_DEFAULT            20
#define GRIPPER_GRIP_STEP_MIN                5
#define GRIPPER_GRIP_STEP_MAX                100
#define GRIPPER_GRIP_HIT_LIMIT               2
#define GRIPPER_RELEASE_DELTA_DEFAULT        100
#define GRIPPER_RELEASE_DELTA_MIN            20
#define GRIPPER_RELEASE_DELTA_MAX            400
#define GRIPPER_RELEASE_SPEED_DEFAULT        500
#define GRIPPER_RELEASE_SPEED_MIN            100
#define GRIPPER_RELEASE_SPEED_MAX            1000

typedef enum {
  GRIPPER_OK = 0,
  GRIPPER_RANGE_ERROR,
  GRIPPER_SERVO_ERROR,
  GRIPPER_STATUS_ERROR,
  GRIPPER_STALL,
  GRIPPER_TIMEOUT,
  GRIPPER_NO_OBJECT,
  GRIPPER_TORQUE_OFF_ERROR,
  GRIPPER_TORQUE_ON_ERROR,
  GRIPPER_MOVE_ERROR
} GripperResult;

GripperResult Gripper_MoveFeedback(uint8_t servo_id, uint16_t position, uint16_t speed,
                                   BusServoStatus *final_status, BusServoResult *servo_result);
GripperResult Gripper_Grip(uint8_t servo_id, uint16_t load_threshold, uint16_t speed,
                           uint16_t step, BusServoStatus *final_status,
                           BusServoResult *servo_result);
GripperResult Gripper_Release(uint8_t servo_id, uint16_t delta, uint16_t speed,
                              int16_t *cur_pos, uint16_t *target_pos,
                              BusServoStatus *final_status, BusServoResult *servo_result);
GripperResult Gripper_Open(uint8_t servo_id, uint16_t speed, BusServoResult *servo_result);
GripperResult Gripper_Close(uint8_t servo_id, uint16_t speed, BusServoResult *servo_result);
const char *Gripper_ResultName(GripperResult result);

#endif
