/**
  ******************************************************************************
  * @file    bsp_stepper_init.c
  * @brief   双路步进电机初始化
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_init.h"
#include "./stepper/bsp_stepper_T_speed.h"
#include "./delay/core_delay.h"
#include "stm32f4xx.h"
#include "math.h"
#include <math.h>

TIM_HandleTypeDef TIM_TimeBaseStructure;

const StepperHw stepper_hw[STEPPER_NUM] = {
  /* Motor0: ENA PE0, DIR PE1, PUL PI5 / TIM8_CH1 */
  {GPIOE, GPIO_PIN_1, GPIOE, GPIO_PIN_0, GPIOI, GPIO_PIN_5, 1, TIM_CHANNEL_1, TIM_IT_CC1, TIM_FLAG_CC1},
  /* Motor1: ENA PE4, DIR PI8, PUL PI6 / TIM8_CH2 */
  {GPIOI, GPIO_PIN_8, GPIOE, GPIO_PIN_4, GPIOI, GPIO_PIN_6, 0, TIM_CHANNEL_2, TIM_IT_CC2, TIM_FLAG_CC2}
};

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void Stepper_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t i;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    GPIO_InitStruct.Pin = stepper_hw[i].dir_pin;
    HAL_GPIO_Init(stepper_hw[i].dir_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = stepper_hw[i].en_pin;
    HAL_GPIO_Init(stepper_hw[i].en_port, &GPIO_InitStruct);

    MOTOR_EN(i, ON);
  }

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = MOTOR_PUL_GPIO_AF;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    GPIO_InitStruct.Pin = stepper_hw[i].pul_pin;
    HAL_GPIO_Init(stepper_hw[i].pul_port, &GPIO_InitStruct);
  }
}

/**
  * @brief  中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
  HAL_NVIC_SetPriority(MOTOR_PUL_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR_PUL_IRQn);
}

void TIM_PWMOUTPUT_Config(void)
{
  TIM_OC_InitTypeDef TIM_OCInitStructure;
  uint8_t i;

  MOTOR_PUL_CLK_ENABLE();

  TIM_TimeBaseStructure.Instance = MOTOR_PUL_TIM;
  TIM_TimeBaseStructure.Init.Period = TIM_PERIOD - 1;
  TIM_TimeBaseStructure.Init.Prescaler = TIM_PRESCALER - 1;
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.Init.RepetitionCounter = 0;
  HAL_TIM_OC_Init(&TIM_TimeBaseStructure);

  TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE;
  TIM_OCInitStructure.Pulse = 0;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_LOW;
  TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
  TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, stepper_hw[i].pul_channel);
    TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[i].pul_channel, TIM_CCx_DISABLE);
    __HAL_TIM_DISABLE_IT(&TIM_TimeBaseStructure, stepper_hw[i].pul_it);
  }
}

/**
  * @brief  引脚初始化
  * @retval 无
  */
void stepper_Init(void)
{
  Stepper_GPIO_Config();
  TIM_PWMOUTPUT_Config();
  TIMx_NVIC_Configuration();
}

/**
  * @brief  TIM8比较中断服务函数，分别处理CH1和CH2
  * @retval 无
  */
void MOTOR_PUL_IRQHandler(void)
{
  speed_decision(STEPPER_MOTOR_0);
  speed_decision(STEPPER_MOTOR_1);
}
