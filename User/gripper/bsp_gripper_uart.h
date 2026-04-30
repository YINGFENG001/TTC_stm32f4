#ifndef __BSP_GRIPPER_UART_H
#define __BSP_GRIPPER_UART_H

#include "stm32f4xx.h"

#define GRIPPER_UART                         UART5
#define GRIPPER_UART_BAUDRATE                1000000

#define GRIPPER_UART_CLK_ENABLE()            __HAL_RCC_UART5_CLK_ENABLE()

#define GRIPPER_UART_TX_GPIO_PORT            GPIOC
#define GRIPPER_UART_TX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define GRIPPER_UART_TX_PIN                  GPIO_PIN_12
#define GRIPPER_UART_TX_AF                   GPIO_AF8_UART5

#define GRIPPER_UART_RX_GPIO_PORT            GPIOD
#define GRIPPER_UART_RX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()
#define GRIPPER_UART_RX_PIN                  GPIO_PIN_2
#define GRIPPER_UART_RX_AF                   GPIO_AF8_UART5

extern UART_HandleTypeDef GripperUartHandle;

void Gripper_UART_Config(void);
void Gripper_UART_FlushRx(void);
HAL_StatusTypeDef Gripper_UART_Send(uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef Gripper_UART_Recv(uint8_t *data, uint16_t len, uint32_t timeout);

#endif
