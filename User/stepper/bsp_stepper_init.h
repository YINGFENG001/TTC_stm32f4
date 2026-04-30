#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define STEPPER_NUM                    2
#define STEPPER_MOTOR_0                0
#define STEPPER_MOTOR_1                1

#define MOTOR_PUL_IRQn                 TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler           TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                  TIM8
#define MOTOR_PUL_CLK_ENABLE()         __TIM8_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF              GPIO_AF3_TIM8
#define TIM_PERIOD                     0xFFFF

#define HIGH                           1
#define LOW                            0

/* 驱动器 ENA 端为低电平有效 */
#define ON                             0
#define OFF                            (!0)

#define CLOCKWISE                      1
#define ANTI_CLOCKWISE                 0

#define GPIO_H(p,i)                    { (p)->BSRR = (i); }
#define GPIO_L(p,i)                    { (p)->BSRR = (uint32_t)(i) << 16; }
#define GPIO_T(p,i)                    { (p)->ODR ^= (i); }

/* 单路步进电机硬件资源映射表 */
typedef struct {
  GPIO_TypeDef *dir_port; /* 方向控制GPIO端口 */
  uint16_t dir_pin;       /* 方向控制GPIO引脚 */
  GPIO_TypeDef *en_port;  /* 使能控制GPIO端口 */
  uint16_t en_pin;        /* 使能控制GPIO引脚 */
  GPIO_TypeDef *pul_port; /* 脉冲输出GPIO端口 */
  uint16_t pul_pin;       /* 脉冲输出GPIO引脚 */
  uint8_t dir_inverted;   /* 方向是否反相：1表示输出电平取反 */
  uint32_t pul_channel;   /* TIM8 输出比较通道 */
  uint32_t pul_it;        /* TIM8 对应比较中断源 */
  uint32_t pul_flag;      /* TIM8 对应比较标志位 */
} StepperHw;

extern const StepperHw stepper_hw[STEPPER_NUM];
extern TIM_HandleTypeDef TIM_TimeBaseStructure;

#define STEPPER_ID_VALID(id)           ((id) < STEPPER_NUM)

#define MOTOR_EN(id,x)                 if (x)                                      \
                                       { GPIO_H(stepper_hw[(id)].en_port,          \
                                                stepper_hw[(id)].en_pin); }        \
                                       else                                        \
                                       { GPIO_L(stepper_hw[(id)].en_port,          \
                                                stepper_hw[(id)].en_pin); }

#define MOTOR_PUL(id,x)                if (x)                                      \
                                       { GPIO_H(stepper_hw[(id)].pul_port,         \
                                                stepper_hw[(id)].pul_pin); }       \
                                       else                                        \
                                       { GPIO_L(stepper_hw[(id)].pul_port,         \
                                                stepper_hw[(id)].pul_pin); }

#define MOTOR_DIR(id,x)                if (x)                                      \
                                       { GPIO_H(stepper_hw[(id)].dir_port,         \
                                                stepper_hw[(id)].dir_pin); }       \
                                       else                                        \
                                       { GPIO_L(stepper_hw[(id)].dir_port,         \
                                                stepper_hw[(id)].dir_pin); }

#define MOTOR_PUL_T(id)                GPIO_T(stepper_hw[(id)].pul_port,           \
                                              stepper_hw[(id)].pul_pin)

void stepper_Init(void);
void TIM_PWMOUTPUT_Config(void);

#endif
