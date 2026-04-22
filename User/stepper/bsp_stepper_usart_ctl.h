#ifndef __BSP_STEPPER_USART_CTL_H
#define	__BSP_STEPPER_USART_CTL_H

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

#endif
