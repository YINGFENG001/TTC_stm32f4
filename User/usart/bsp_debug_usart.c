/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口1，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "./usart/bsp_debug_usart.h"
#include "./gripper/bsp_gripper_uart.h"
#include <string.h>

UART_HandleTypeDef UartHandle;

static volatile uint8_t uart_active_buf[UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_pending_buf[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_active_len;
static volatile uint8_t uart_cmd_ready;
static volatile uint8_t uart_overrun;

static uint8_t DebugUsart_IsSpace(uint8_t ch)
{
  return ((ch == ' ') || (ch == '\t')) ? 1U : 0U;
}

static uint8_t DebugUsart_TokenEquals(const volatile uint8_t *buf,
                                      uint16_t start,
                                      uint16_t end,
                                      const char *word)
{
  uint16_t i;

  i = 0;
  while ((start + i) < end)
  {
    if ((word[i] == '\0') || ((char)buf[start + i] != word[i]))
    {
      return 0U;
    }
    i++;
  }

  return (word[i] == '\0') ? 1U : 0U;
}

static uint8_t DebugUsart_IsPriorityStopCommand(const volatile uint8_t *buf,
                                                uint16_t len)
{
  uint16_t token_start[3];
  uint16_t token_end[3];
  uint16_t pos;
  uint8_t token_count;

  pos = 0;
  token_count = 0;
  while ((pos < len) && (token_count < 3U))
  {
    while ((pos < len) && (DebugUsart_IsSpace(buf[pos]) != 0U))
    {
      pos++;
    }
    if (pos >= len)
    {
      break;
    }

    token_start[token_count] = pos;
    while ((pos < len) && (DebugUsart_IsSpace(buf[pos]) == 0U))
    {
      pos++;
    }
    token_end[token_count] = pos;
    token_count++;
  }

  if (token_count < 2U)
  {
    return 0U;
  }

  if (buf[token_start[0]] == '#')
  {
    if (token_count < 3U)
    {
      return 0U;
    }
    return DebugUsart_TokenEquals(buf, token_start[2], token_end[2], "stop");
  }

  return DebugUsart_TokenEquals(buf, token_start[1], token_end[1], "stop");
}

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */  
void DEBUG_USART_Config(void)
{ 
  
  UartHandle.Instance          = DEBUG_USART;
  
  UartHandle.Init.BaudRate     = DEBUG_USART_BAUDRATE;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  
  HAL_UART_Init(&UartHandle);
    
 /*使能串口接收断 */
  __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);  
}

/**
  * @brief UART MSP 初始化 
  * @param huart: UART handle
  * @retval 无
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;

  if (huart->Instance == DEBUG_USART)
  {
    DEBUG_USART_CLK_ENABLE();
	
	DEBUG_USART_RX_GPIO_CLK_ENABLE();
  DEBUG_USART_TX_GPIO_CLK_ENABLE();
  
/**USART1 GPIO Configuration    
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX 
  */
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
  
  /* 配置Rx引脚为复用功能 */
  GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;
  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
 
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ ,0,1);	//抢占优先级0，子优先级1
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		    //使能USART1中断通道  
  }
  else if (huart->Instance == GRIPPER_UART)
  {
    GRIPPER_UART_CLK_ENABLE();
    GRIPPER_UART_TX_GPIO_CLK_ENABLE();
    GRIPPER_UART_RX_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = GRIPPER_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GRIPPER_UART_TX_AF;
    HAL_GPIO_Init(GRIPPER_UART_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRIPPER_UART_RX_PIN;
    GPIO_InitStruct.Alternate = GRIPPER_UART_RX_AF;
    HAL_GPIO_Init(GRIPPER_UART_RX_GPIO_PORT, &GPIO_InitStruct);
  }
}


/*****************  发送字符串 **********************/
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(&UartHandle,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
  
}

//清空发送缓冲
void uart_FlushRxBuffer(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  uart_active_len = 0;
  uart_active_buf[0] = 0;
  uart_cmd_ready = 0;
  uart_pending_buf[0] = 0;
  uart_overrun = 0;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void DebugUsart_RxByteFromIsr(uint8_t data)
{
  if (data == '\b')
  {
    if (uart_active_len > 0U)
    {
      Usart_SendByte('\b');
      Usart_SendByte(' ');
      Usart_SendByte('\b');
      uart_active_len--;
      uart_active_buf[uart_active_len] = 0;
    }
    return;
  }

  if ((data == '\r') || (data == '\n'))
  {
    if (uart_active_len == 0U)
    {
      return;
    }
    if ((uart_cmd_ready == 0U) ||
        (DebugUsart_IsPriorityStopCommand(uart_active_buf, uart_active_len) != 0U))
    {
      uart_overrun = (uart_cmd_ready == 0U) ? 0U : 1U;
      memcpy((void *)uart_pending_buf, (const void *)uart_active_buf, uart_active_len);
      uart_pending_buf[uart_active_len] = 0;
      uart_cmd_ready = 1U;
    }
    else
    {
      uart_overrun = 1U;
    }
    uart_active_len = 0;
    uart_active_buf[0] = 0;
    return;
  }

  if (uart_active_len < (UART_RX_BUFFER_SIZE - 1U))
  {
    uart_active_buf[uart_active_len] = data;
    uart_active_len++;
    uart_active_buf[uart_active_len] = 0;
  }
  else
  {
    uart_active_buf[UART_RX_BUFFER_SIZE - 2U] = data;
    Usart_SendByte('\b');
  }
  Usart_SendByte((char)data);
}

uint8_t DebugUsart_PopCommand(char *dest, uint16_t dest_size)
{
  uint32_t primask;
  uint16_t i;

  if ((dest == 0) || (dest_size == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (uart_cmd_ready == 0U)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    dest[0] = 0;
    return 0U;
  }

  for (i = 0; (i < (dest_size - 1U)) && (i < UART_RX_BUFFER_SIZE); i++)
  {
    dest[i] = (char)uart_pending_buf[i];
    if (dest[i] == 0)
    {
      break;
    }
  }
  if (i >= (dest_size - 1U))
  {
    dest[dest_size - 1U] = 0;
  }

  uart_cmd_ready = 0U;
  uart_pending_buf[0] = 0;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return 1U;
}

///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}


/*********************************************END OF FILE**********************/
