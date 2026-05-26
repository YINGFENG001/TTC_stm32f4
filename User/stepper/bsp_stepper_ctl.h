#ifndef __BSP_STEPPER_CTL_H
#define __BSP_STEPPER_CTL_H

#include "./stepper/bsp_device_context.h"

void Stepper_ApplyMechanicalConfig(void);
int32_t Stepper_StepToRev0p1(uint8_t motor_id, int32_t step);
void Stepper_Command(uint8_t device_id, int argc, char *argv[]);

#endif
