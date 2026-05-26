#ifndef __BSP_CLAMP_CTL_H
#define __BSP_CLAMP_CTL_H

#include "./stepper/bsp_device_context.h"
#include "./gripper/bsp_bus_servo.h"
#include "./gripper/bsp_gripper.h"

void Clamp_FillExtraStatus(uint8_t servo_id, BusServoStatus *status_data);
void Clamp_PrintStatusFields(const BusServoStatus *status_data);
void Clamp_HoldTask(uint32_t now);
void Clamp_Command(uint8_t device_id, int argc, char *argv[]);

#endif
