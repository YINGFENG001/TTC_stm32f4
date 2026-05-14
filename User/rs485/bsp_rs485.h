#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "stm32f4xx.h"

#define RS485_UART                         UART4
#define RS485_UART_BAUDRATE                115200

#define RS485_UART_CLK_ENABLE()            __UART4_CLK_ENABLE()

#define RS485_UART_TX_GPIO_PORT            GPIOC
#define RS485_UART_TX_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()
#define RS485_UART_TX_PIN                  GPIO_PIN_10
#define RS485_UART_TX_AF                   GPIO_AF8_UART4

#define RS485_UART_RX_GPIO_PORT            GPIOC
#define RS485_UART_RX_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()
#define RS485_UART_RX_PIN                  GPIO_PIN_11
#define RS485_UART_RX_AF                   GPIO_AF8_UART4

#define RS485_DIR_GPIO_PORT                GPIOH
#define RS485_DIR_GPIO_CLK_ENABLE()        __GPIOH_CLK_ENABLE()
#define RS485_DIR_PIN                      GPIO_PIN_9

extern UART_HandleTypeDef Rs485UartHandle;

void RS485_UART_Config(void);
void RS485_UART_FlushRx(void);
HAL_StatusTypeDef RS485_UART_Send(uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef RS485_UART_Recv(uint8_t *data, uint16_t len, uint32_t timeout);

#endif
