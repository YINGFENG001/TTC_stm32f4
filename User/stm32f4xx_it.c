/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "./usart/bsp_debug_usart.h"
#include "./stepper/bsp_stepper_T_speed.h"
#include "./stepper/bsp_stepper_init.h"
#include <stdio.h>


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define SAFETY_FAULT_MAGIC              0x53414645UL
#define SAFETY_FAULT_NONE               0U
#define SAFETY_FAULT_HARD               1U
#define SAFETY_FAULT_MEM                2U
#define SAFETY_FAULT_BUS                3U
#define SAFETY_FAULT_USAGE              4U
#define SAFETY_IWDG_RELOAD_15S          1874U

typedef struct {
  uint32_t magic;
  uint32_t fault_type;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t mmfar;
  uint32_t bfar;
} SafetyFaultRecord;

#define SAFETY_FAULT_RECORD             ((volatile SafetyFaultRecord *)BKPSRAM_BASE)

static IWDG_HandleTypeDef safety_iwdg_handle;
static uint8_t safety_iwdg_started;

/* Private function prototypes -----------------------------------------------*/
static void Safety_EnableBackupSramAccess(void);
static void Safety_SaveFault(uint32_t fault_type);
static void Safety_FaultShutdownStepper(void);
static void Safety_HandleFault(uint32_t fault_type);
static const char *Safety_FaultName(uint32_t fault_type);
/* Private functions ---------------------------------------------------------*/

static void Safety_EnableBackupSramAccess(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  PWR->CR |= PWR_CR_DBP;
  PWR->CSR |= PWR_CSR_BRE;
}

void Safety_WatchdogInit(void)
{
  safety_iwdg_handle.Instance = IWDG;
  safety_iwdg_handle.Init.Prescaler = IWDG_PRESCALER_256;
  safety_iwdg_handle.Init.Reload = SAFETY_IWDG_RELOAD_15S;

  if (HAL_IWDG_Init(&safety_iwdg_handle) == HAL_OK)
  {
    safety_iwdg_started = 1U;
  }
}

void Safety_WatchdogRefresh(void)
{
  if (safety_iwdg_started != 0U)
  {
    (void)HAL_IWDG_Refresh(&safety_iwdg_handle);
  }
}

static const char *Safety_FaultName(uint32_t fault_type)
{
  switch (fault_type)
  {
    case SAFETY_FAULT_HARD:  return "hard";
    case SAFETY_FAULT_MEM:   return "mem";
    case SAFETY_FAULT_BUS:   return "bus";
    case SAFETY_FAULT_USAGE: return "usage";
    default:                 return "unknown";
  }
}

void Safety_ReportLastFault(void)
{
  uint8_t iwdg_reset;

  Safety_EnableBackupSramAccess();
  iwdg_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ? 1U : 0U;

  if (SAFETY_FAULT_RECORD->magic == SAFETY_FAULT_MAGIC)
  {
    printf("\n@event level=fault dev=system event=last_fault type=%s cfsr=0x%08lX hfsr=0x%08lX mmfar=0x%08lX bfar=0x%08lX reset=%s",
           Safety_FaultName(SAFETY_FAULT_RECORD->fault_type),
           (unsigned long)SAFETY_FAULT_RECORD->cfsr,
           (unsigned long)SAFETY_FAULT_RECORD->hfsr,
           (unsigned long)SAFETY_FAULT_RECORD->mmfar,
           (unsigned long)SAFETY_FAULT_RECORD->bfar,
           iwdg_reset ? "iwdg" : "other");
    SAFETY_FAULT_RECORD->magic = 0U;
  }
  else if (iwdg_reset != 0U)
  {
    printf("\n@event level=fault dev=system event=last_reset reset=iwdg");
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static void Safety_SaveFault(uint32_t fault_type)
{
  Safety_EnableBackupSramAccess();
  SAFETY_FAULT_RECORD->fault_type = fault_type;
  SAFETY_FAULT_RECORD->cfsr = SCB->CFSR;
  SAFETY_FAULT_RECORD->hfsr = SCB->HFSR;
  SAFETY_FAULT_RECORD->mmfar = SCB->MMFAR;
  SAFETY_FAULT_RECORD->bfar = SCB->BFAR;
  SAFETY_FAULT_RECORD->magic = SAFETY_FAULT_MAGIC;
}

static void Safety_FaultShutdownStepper(void)
{
  uint8_t i;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[i].pul_channel, TIM_CCx_DISABLE);
    __HAL_TIM_DISABLE_IT(&TIM_TimeBaseStructure, stepper_hw[i].pul_it);
    MOTOR_EN(i, OFF);
  }
  __HAL_TIM_DISABLE(&TIM_TimeBaseStructure);
}

static void Safety_HandleFault(uint32_t fault_type)
{
  __disable_irq();
  Safety_FaultShutdownStepper();
  Safety_SaveFault(fault_type);
  if (safety_iwdg_started == 0U)
  {
    NVIC_SystemReset();
  }
  while (1)
  {
  }
}

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  Safety_HandleFault(SAFETY_FAULT_HARD);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  Safety_HandleFault(SAFETY_FAULT_MEM);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  Safety_HandleFault(SAFETY_FAULT_BUS);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  Safety_HandleFault(SAFETY_FAULT_USAGE);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}
  

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
extern UART_HandleTypeDef UartHandle;

void Usart_SendByte(char ch)
{
	WRITE_REG(UartHandle.Instance->DR,ch); 
}

void DEBUG_USART_IRQHandler(void)
{	
    unsigned char data;
    
	if(__HAL_UART_GET_FLAG( &UartHandle, UART_FLAG_RXNE ) != RESET)
	{	
			data = ( uint16_t)READ_REG(UartHandle.Instance->DR);
			DebugUsart_RxByteFromIsr(data);
    
    }	 

}
/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
