#include "./gripper/bsp_gripper_uart.h"

UART_HandleTypeDef GripperUartHandle;

void Gripper_UART_Config(void)
{
  GripperUartHandle.Instance = GRIPPER_UART;
  GripperUartHandle.Init.BaudRate = GRIPPER_UART_BAUDRATE;
  GripperUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  GripperUartHandle.Init.StopBits = UART_STOPBITS_1;
  GripperUartHandle.Init.Parity = UART_PARITY_NONE;
  GripperUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  GripperUartHandle.Init.Mode = UART_MODE_TX_RX;
  GripperUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  HAL_UART_Init(&GripperUartHandle);
  Gripper_UART_FlushRx();
}

void Gripper_UART_FlushRx(void)
{
  uint8_t dummy;

  __HAL_UART_CLEAR_OREFLAG(&GripperUartHandle);
  while (__HAL_UART_GET_FLAG(&GripperUartHandle, UART_FLAG_RXNE) != RESET)
  {
    dummy = (uint8_t)READ_REG(GripperUartHandle.Instance->DR);
    (void)dummy;
  }
}

HAL_StatusTypeDef Gripper_UART_Send(uint8_t *data, uint16_t len, uint32_t timeout)
{
  return HAL_UART_Transmit(&GripperUartHandle, data, len, timeout);
}

HAL_StatusTypeDef Gripper_UART_Recv(uint8_t *data, uint16_t len, uint32_t timeout)
{
  return HAL_UART_Receive(&GripperUartHandle, data, len, timeout);
}
