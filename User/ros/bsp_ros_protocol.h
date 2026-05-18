#ifndef __BSP_ROS_PROTOCOL_H
#define __BSP_ROS_PROTOCOL_H

#include "stm32f4xx.h"

uint8_t RosProtocol_TryDispatch(char *line);
void RosProtocol_ReportStepperDone(uint32_t id, const char *dev,
                                   const char *cmd, int32_t rev_0p1);

#endif
