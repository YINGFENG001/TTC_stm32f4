/**
  ******************************************************************************
  * @file    bsp_stepper_T_speed.c
  * @brief   双路步进电机梯形加减速算法
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_T_speed.h"
#include "./usart/bsp_debug_usart.h"

speedRampData srd[STEPPER_NUM];
int stepPosition[STEPPER_NUM] = {0};
struct GLOBAL_FLAGS status = {FALSE, FALSE, TRUE};
struct GLOBAL_FLAGS motor_status[STEPPER_NUM] = {
  {FALSE, FALSE, TRUE},
  {FALSE, FALSE, TRUE}
};
static uint32_t stepper_pulses_per_rev[STEPPER_NUM] = {
  STEPPER_DEFAULT_PULSES_PER_REV,
  STEPPER_DEFAULT_PULSES_PER_REV
};

void Stepper_SetMotorPulsesPerRev(uint8_t motor_id, uint32_t pulses_per_rev)
{
  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  if (pulses_per_rev == 0)
  {
    return;
  }

  stepper_pulses_per_rev[motor_id] = pulses_per_rev;
}

static uint8_t Stepper_IsOtherRunning(uint8_t motor_id)
{
  uint8_t i;
  for (i = 0; i < STEPPER_NUM; i++)
  {
    if ((i != motor_id) && (motor_status[i].running == TRUE))
    {
      return TRUE;
    }
  }
  return FALSE;
}

uint8_t Stepper_IsAnyRunning(void)
{
  uint8_t i;
  for (i = 0; i < STEPPER_NUM; i++)
  {
    if (motor_status[i].running == TRUE)
    {
      return TRUE;
    }
  }
  return FALSE;
}

static void Stepper_UpdateGlobalStatus(void)
{
  uint8_t i;
  status.running = Stepper_IsAnyRunning();
  status.out_ena = TRUE;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    if (motor_status[i].out_ena != TRUE)
    {
      status.out_ena = FALSE;
      break;
    }
  }
}

static uint8_t Stepper_GetDirLevel(uint8_t motor_id, uint8_t logic_dir)
{
  if (stepper_hw[motor_id].dir_inverted != 0)
  {
    return (uint8_t)(logic_dir ? 0 : 1);
  }

  return logic_dir;
}

/**
  * @brief  根据运动方向判断步进电机的运行位置
  * @param  motor_id 电机编号
  * @param  inc 运动方向
  * @retval 无
  */
static void StepperCounter(uint8_t motor_id, signed char inc)
{
  if (inc == CCW)
  {
    stepPosition[motor_id]--;
  }
  else
  {
    stepPosition[motor_id]++;
  }
}

/**
  * @brief  驱动器使能控制，两路同时生效
  * @param  NewState：ENABLE为恢复输出，DISABLE为禁止输出
  * @retval 无
  */
void MSD_ENA(FunctionalState NewState)
{
  uint8_t i;

  for (i = 0; i < STEPPER_NUM; i++)
  {
    if (NewState)
    {
      MOTOR_EN(i, ON);
      motor_status[i].out_ena = TRUE;
    }
    else
    {
      MOTOR_EN(i, OFF);
      motor_status[i].out_ena = FALSE;
    }
  }

  /* 5. 装载首个比较值并启动对应通道中断 */
  Stepper_UpdateGlobalStatus();

  if (NewState)
  {
    printf("\n\r驱动器恢复运行，两路电机均为保持力矩状态");
  }
  else
  {
    printf("\n\r驱动器禁止输出，两路电机均为脱机状态");
  }
}

StepperCmdResult Stepper_Stop(uint8_t motor_id)
{
  if (!STEPPER_ID_VALID(motor_id))
  {
    return STEPPER_CMD_ID_ERROR;
  }

  /* 立即关闭当前通道输出与中断，保证中途停机不会继续追目标。 */
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_DISABLE);
  __HAL_TIM_DISABLE_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);

  srd[motor_id].run_state = STOP;
  srd[motor_id].step_delay = 0;
  srd[motor_id].accel_count = 0;
  motor_status[motor_id].running = FALSE;
  Stepper_UpdateGlobalStatus();

  if (Stepper_IsAnyRunning() != TRUE)
  {
    __HAL_TIM_DISABLE(&TIM_TimeBaseStructure);
  }

  return STEPPER_CMD_OK;
}

/*! \brief 以给定的步数移动步进电机
 *  \param motor_id 电机编号，0为PE0/PE1/PI5，1为PE4/PI8/PI6
 *  \param step   移动的步数 (正数为顺时针，负数为逆时针).
 *  \param accel  加速度,单位为0.1rad/sec^2
 *  \param decel  减速度,单位为0.1rad/sec^2
 *  \param speed  最大速度,单位为0.1rad/sec
 */
StepperCmdResult stepper_move_T(uint8_t motor_id, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
  /* 1. 输入参数与运行状态校验 */
  unsigned int max_s_lim;
  unsigned int accel_lim;
  int tim_count;
  uint32_t pulses_per_rev;
  float alpha;
  float a_t_x10;
  float a_sq;
  float a_x200;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return STEPPER_CMD_ID_ERROR;
  }

  if (motor_status[motor_id].out_ena != TRUE)
  {
    return STEPPER_CMD_DISABLED;
  }

  if (motor_status[motor_id].running == TRUE)
  {
    return STEPPER_CMD_BUSY;
  }

  if ((step == 0) || (accel == 0) || (decel == 0) || (speed == 0))
  {
    return STEPPER_CMD_PARAM_ERROR;
  }

  /* 2. 按当前电机每圈脉冲数计算梯形加减速系数 */
  pulses_per_rev = stepper_pulses_per_rev[motor_id];
  if (pulses_per_rev == 0)
  {
    return STEPPER_CMD_PARAM_ERROR;
  }

  alpha = (float)(2.0f * 3.14159f / pulses_per_rev);
  a_t_x10 = (float)(10.0f * alpha * T1_FREQ);
  a_sq = (float)(2.0f * 100000.0f * alpha);
  a_x200 = (float)(200.0f * alpha);

  /* 3. 根据步数符号确定方向，并设置方向引脚 */
  if (step < 0)
  {
    srd[motor_id].dir = CCW;
    step = -step;
  }
  else
  {
    srd[motor_id].dir = CW;
  }

  MOTOR_DIR(motor_id, Stepper_GetDirLevel(motor_id, srd[motor_id].dir));

  /* 4. 初始化运行状态机，计算加速段、匀速段和减速段参数 */
  if (step == 1)
  {
    srd[motor_id].accel_count = -1;
    srd[motor_id].run_state = DECEL;
    srd[motor_id].step_delay = 1000;
    motor_status[motor_id].running = TRUE;
  }
  else
  {
    srd[motor_id].min_delay = (int32_t)(a_t_x10 / speed);
    srd[motor_id].step_delay = (int32_t)((T1_FREQ_148 * sqrt(a_sq / accel)) / 10);

    if ((srd[motor_id].min_delay <= 0) || (srd[motor_id].step_delay <= 0))
    {
      return STEPPER_CMD_PARAM_ERROR;
    }

    if ((srd[motor_id].step_delay / 2) > TIM_PERIOD)
    {
      return STEPPER_CMD_PARAM_ERROR;
    }

    max_s_lim = (uint32_t)(speed * speed / (a_x200 * accel / 10));
    if (max_s_lim == 0)
    {
      max_s_lim = 1;
    }

    accel_lim = ((long)step * decel) / (accel + decel);
    if (accel_lim == 0)
    {
      accel_lim = 1;
    }

    if (accel_lim <= max_s_lim)
    {
      srd[motor_id].decel_val = accel_lim - step;
    }
    else
    {
      srd[motor_id].decel_val = -(long)(max_s_lim * accel / decel);
    }

    if (srd[motor_id].decel_val == 0)
    {
      srd[motor_id].decel_val = -1;
    }

    srd[motor_id].decel_start = step + srd[motor_id].decel_val;

    if (srd[motor_id].step_delay <= srd[motor_id].min_delay)
    {
      srd[motor_id].step_delay = srd[motor_id].min_delay;
      srd[motor_id].run_state = RUN;
    }
    else
    {
      srd[motor_id].run_state = ACCEL;
    }

    srd[motor_id].accel_count = 0;
    motor_status[motor_id].running = TRUE;
  }

  /* 5. 装载首个比较值并启动对应通道中断 */
  Stepper_UpdateGlobalStatus();

  tim_count = __HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,
                        stepper_hw[motor_id].pul_channel,
                        tim_count + srd[motor_id].step_delay / 2);

  TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_DISABLE);
  __HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_ENABLE_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_MOE_ENABLE(&TIM_TimeBaseStructure);
  __HAL_TIM_ENABLE(&TIM_TimeBaseStructure);

  return STEPPER_CMD_OK;
}

/**
  * @brief  速度决策，在TIM8比较中断中按通道调用
  * @param  motor_id 电机编号
  * @retval 无
  */
void speed_decision(uint8_t motor_id)
{
  __IO uint32_t tim_count = 0;
  __IO uint32_t tmp = 0;
  uint16_t new_step_delay = 0;
  __IO static uint16_t last_accel_delay[STEPPER_NUM] = {0};
  __IO static uint32_t step_count[STEPPER_NUM] = {0};
  __IO static int32_t rest[STEPPER_NUM] = {0};
  __IO static uint8_t edge_count[STEPPER_NUM] = {0};

  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  /* 1. 检查当前通道的比较中断是否到达 */
  if ((__HAL_TIM_GET_FLAG(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_flag) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it) != RESET))
  {
    __HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);

    /* 2. 重新装载下一次比较值，维持脉冲翻转节奏 */
    tim_count = __HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
    tmp = tim_count + srd[motor_id].step_delay / 2;
    __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_channel, tmp);

    /* 3. 每经过两次翻转，才算输出了一个完整步进脉冲 */
    edge_count[motor_id]++;
    if (edge_count[motor_id] == 2)
    {
      edge_count[motor_id] = 0;

      /* 4. 根据当前状态推进梯形加减速状态机 */
      switch (srd[motor_id].run_state)
      {
        case STOP:
          step_count[motor_id] = 0;
          rest[motor_id] = 0;
          TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_DISABLE);
          __HAL_TIM_DISABLE_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);

          motor_status[motor_id].running = FALSE;
          Stepper_UpdateGlobalStatus();

          if (Stepper_IsOtherRunning(motor_id) != TRUE)
          {
            __HAL_TIM_DISABLE(&TIM_TimeBaseStructure);
          }
          break;

        case ACCEL:
          StepperCounter(motor_id, srd[motor_id].dir);
          TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_ENABLE);
          step_count[motor_id]++;
          srd[motor_id].accel_count++;
          new_step_delay = srd[motor_id].step_delay - (((2 * (long)srd[motor_id].step_delay) + rest[motor_id]) / (4 * srd[motor_id].accel_count + 1));
          rest[motor_id] = ((2 * (long)srd[motor_id].step_delay) + rest[motor_id]) % (4 * srd[motor_id].accel_count + 1);

          if (step_count[motor_id] >= srd[motor_id].decel_start)
          {
            srd[motor_id].accel_count = srd[motor_id].decel_val;
            srd[motor_id].run_state = DECEL;
          }
          else if (new_step_delay <= srd[motor_id].min_delay)
          {
            last_accel_delay[motor_id] = new_step_delay;
            new_step_delay = srd[motor_id].min_delay;
            rest[motor_id] = 0;
            srd[motor_id].run_state = RUN;
          }
          break;

        case RUN:
          StepperCounter(motor_id, srd[motor_id].dir);
          step_count[motor_id]++;
          new_step_delay = srd[motor_id].min_delay;

          if (step_count[motor_id] >= srd[motor_id].decel_start)
          {
            srd[motor_id].accel_count = srd[motor_id].decel_val;
            if (last_accel_delay[motor_id] == 0)
            {
              new_step_delay = srd[motor_id].step_delay;
            }
            else
            {
              new_step_delay = last_accel_delay[motor_id];
            }
            srd[motor_id].run_state = DECEL;
          }
          break;

        case DECEL:
          StepperCounter(motor_id, srd[motor_id].dir);
          step_count[motor_id]++;
          srd[motor_id].accel_count++;
          new_step_delay = srd[motor_id].step_delay - (((2 * (long)srd[motor_id].step_delay) + rest[motor_id]) / (4 * srd[motor_id].accel_count + 1));
          rest[motor_id] = ((2 * (long)srd[motor_id].step_delay) + rest[motor_id]) % (4 * srd[motor_id].accel_count + 1);

          if (srd[motor_id].accel_count >= 0)
          {
            srd[motor_id].run_state = STOP;
          }
          break;
      }

      srd[motor_id].step_delay = new_step_delay;
    }
  }
}
