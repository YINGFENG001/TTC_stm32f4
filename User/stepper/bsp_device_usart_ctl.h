#ifndef __BSP_DEVICE_USART_CTL_H
#define	__BSP_DEVICE_USART_CTL_H

#include "stm32f4xx.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "./stepper/bsp_stepper_T_speed.h"
#include "./usart/bsp_debug_usart.h"
#include "./delay/core_delay.h"

void ShowHelp(void);
void DealSerialData(void);
void Device_Task(void);
void Device_ReportDone(void);

typedef enum {
  DEVICE_API_OK = 0,
  DEVICE_API_BUSY,
  DEVICE_API_DISABLED,
  DEVICE_API_PARAM_ERROR,
  DEVICE_API_RANGE_ERROR,
  DEVICE_API_UART_ERROR,
  DEVICE_API_TIMEOUT,
  DEVICE_API_CRC_ERROR,
  DEVICE_API_ID_ERROR,
  DEVICE_API_DEVICE_ERROR
} DeviceApiResult;

typedef struct {
  uint8_t enabled;
  uint8_t running;
  int32_t rev_0p1;
  int32_t target_0p1;
  uint32_t err;
  uint32_t accel_rpm_s;
  uint32_t decel_rpm_s;
  uint32_t rpm;
} DeviceStepperStatus;

typedef struct {
  uint8_t servo_id;
  int16_t pos;
  int16_t speed;
  int16_t load;
  uint8_t voltage;
  uint8_t temp;
  int16_t current;
  uint8_t state;
} DeviceClampStatus;

typedef struct {
  uint16_t state1;
  uint16_t state2;
  uint16_t fault;
  uint8_t busy1;
  uint8_t busy2;
  uint8_t obj1;
  uint8_t obj2;
  uint8_t vac1;
  uint8_t vac2;
  uint8_t temp;
  uint16_t bus_x10;
} DeviceVacumStatus;

DeviceApiResult DeviceApi_StepperStatus(uint8_t motor_id, DeviceStepperStatus *out);
DeviceApiResult DeviceApi_StepperMove(uint8_t motor_id, int32_t rev_0p1);
DeviceApiResult DeviceApi_StepperRun(uint8_t motor_id, uint8_t dir);
DeviceApiResult DeviceApi_StepperStop(uint8_t motor_id, int32_t *rev_0p1);
DeviceApiResult DeviceApi_StepperSetAccel(uint8_t motor_id, uint32_t accel);
DeviceApiResult DeviceApi_StepperSetDecel(uint8_t motor_id, uint32_t decel);
DeviceApiResult DeviceApi_StepperSetRpm(uint8_t motor_id, uint32_t rpm);
void DeviceApi_BindStepperRosCmd(uint8_t motor_id, uint32_t id, const char *cmd);

DeviceApiResult DeviceApi_ClampStatus(uint8_t servo_id, DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampMove(uint8_t open_percentage, DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampOpen(DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampClose(DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampGrip(uint16_t load, DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampGripAt(uint16_t load, uint8_t open_percentage, DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampRelease(DeviceClampStatus *out);
DeviceApiResult DeviceApi_ClampSet(uint16_t speed, uint16_t grip_step, uint16_t release_delta);

DeviceApiResult DeviceApi_VacumSet(uint8_t min_vac, uint8_t max_vac, uint8_t timeout);
DeviceApiResult DeviceApi_VacumGrip(void);
DeviceApiResult DeviceApi_VacumRelease(void);
DeviceApiResult DeviceApi_VacumStop(void);
DeviceApiResult DeviceApi_VacumStatus(DeviceVacumStatus *out);

#endif
