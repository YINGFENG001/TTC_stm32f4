#ifndef __BSP_VACUM_CTL_H
#define __BSP_VACUM_CTL_H

#include "./stepper/bsp_device_context.h"
#include "./vacum/bsp_evs08.h"

void Vacum_MonitorTask(uint32_t now);
void Vacum_Command(uint8_t device_id, int argc, char *argv[]);

#endif
