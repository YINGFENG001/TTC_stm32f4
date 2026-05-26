#ifndef __BSP_DEVICE_CONTEXT_H
#define __BSP_DEVICE_CONTEXT_H

#include "./stepper/bsp_device_usart_ctl.h"

typedef enum {
  DEVICE_MTOR1 = 0,
  DEVICE_MTOR2,
  DEVICE_CLAMP,
  DEVICE_VACUM,
  DEVICE_NUM
} DeviceId;

typedef enum {
  DEVICE_TYPE_STEPPER = 0,
  DEVICE_TYPE_SERVO,
  DEVICE_TYPE_VACUM
} DeviceType;

typedef enum {
  DEV_IDLE = 0,
  DEV_RUNNING,
  DEV_DONE,
  DEV_DISABLED,
  DEV_NOT_READY,
  DEV_ERROR
} DeviceState;

typedef struct {
  DeviceId id;
  const char *name;
  DeviceType type;
  DeviceState state;
  uint8_t enabled;
  int32_t position;
  int32_t target;
  int32_t value;
  uint32_t error_code;
  uint8_t has_ros_cmd;
  uint32_t ros_cmd_id;
  const char *ros_cmd_name;
} EndDevice;

#define DEVICE_FAST_CHECK_INTERVAL_MS  200U
#define VACUM_DROP_HIT_LIMIT           5U
#define DEVICE_FAULT_DROP              1002U

extern EndDevice devices[DEVICE_NUM];

void PrintFixed1Signed(int32_t value);
void Device_PrintStatus(uint8_t id);

#endif
