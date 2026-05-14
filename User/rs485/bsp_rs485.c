#include "./rs485/bsp_rs485.h"

UART_HandleTypeDef Rs485UartHandle;

static void RS485_Delay(__IO uint32_t nCount)
{
  while (nCount-- != 0)
  {
  }
}

static void RS485_SetTx(void)
{
  RS485_Delay(1000);
  HAL_GPIO_WritePin(RS485_DIR_GPIO_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
  RS485_Delay(1000);
}

static void RS485_SetRx(void)
{
  RS485_Delay(1000);
  HAL_GPIO_WritePin(RS485_DIR_GPIO_PORT, RS485_DIR_PIN, GPIO_PIN_SET);
  RS485_Delay(1000);
}

void RS485_UART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  RS485_UART_TX_GPIO_CLK_ENABLE();
  RS485_UART_RX_GPIO_CLK_ENABLE();
  RS485_DIR_GPIO_CLK_ENABLE();
  RS485_UART_CLK_ENABLE();

  GPIO_InitStruct.Pin = RS485_UART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = RS485_UART_TX_AF;
  HAL_GPIO_Init(RS485_UART_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RS485_UART_RX_PIN;
  GPIO_InitStruct.Alternate = RS485_UART_RX_AF;
  HAL_GPIO_Init(RS485_UART_RX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RS485_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(RS485_DIR_GPIO_PORT, &GPIO_InitStruct);

  Rs485UartHandle.Instance = RS485_UART;
  Rs485UartHandle.Init.BaudRate = RS485_UART_BAUDRATE;
  Rs485UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  Rs485UartHandle.Init.StopBits = UART_STOPBITS_1;
  Rs485UartHandle.Init.Parity = UART_PARITY_NONE;
  Rs485UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Rs485UartHandle.Init.Mode = UART_MODE_TX_RX;
  Rs485UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&Rs485UartHandle);

  RS485_SetRx();
  RS485_UART_FlushRx();
}

void RS485_UART_FlushRx(void)
{
  uint8_t dummy;

  __HAL_UART_CLEAR_OREFLAG(&Rs485UartHandle);
  while (__HAL_UART_GET_FLAG(&Rs485UartHandle, UART_FLAG_RXNE) != RESET)
  {
    dummy = (uint8_t)READ_REG(Rs485UartHandle.Instance->DR);
    (void)dummy;
  }
}

HAL_StatusTypeDef RS485_UART_Send(uint8_t *data, uint16_t len, uint32_t timeout)
{
  HAL_StatusTypeDef result;
  uint32_t tickstart;

  RS485_SetTx();
  result = HAL_UART_Transmit(&Rs485UartHandle, data, len, timeout);
  if (result == HAL_OK)
  {
    tickstart = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&Rs485UartHandle, UART_FLAG_TC) == RESET)
    {
      if ((timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) > timeout))
      {
        result = HAL_TIMEOUT;
        break;
      }
    }
  }
  RS485_SetRx();

  return result;
}

HAL_StatusTypeDef RS485_UART_Recv(uint8_t *data, uint16_t len, uint32_t timeout)
{
  return HAL_UART_Receive(&Rs485UartHandle, data, len, timeout);
}
