/**
  ******************************************************************************
  * @file    bsp_stepper_T_speed.c
  * @brief   双路步进电机梯形加减速算法
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_T_speed.h"
#include "./usart/bsp_debug_usart.h"

volatile speedRampData srd[STEPPER_NUM];
volatile int stepPosition[STEPPER_NUM] = {0};
volatile struct GLOBAL_FLAGS status = {FALSE, FALSE, TRUE};
volatile struct GLOBAL_FLAGS motor_status[STEPPER_NUM] = {
  {FALSE, FALSE, TRUE},
  {FALSE, FALSE, TRUE}
};
static uint32_t stepper_pulses_per_rev[STEPPER_NUM] = {
  STEPPER_DEFAULT_PULSES_PER_REV,
  STEPPER_DEFAULT_PULSES_PER_REV
};
static uint32_t stepper_speed_delay_k_q8[STEPPER_NUM] = {0};
static uint32_t stepper_last_accel_delay[STEPPER_NUM] = {0};
static uint32_t stepper_step_count[STEPPER_NUM] = {0};
static int32_t stepper_rest[STEPPER_NUM] = {0};
static uint8_t stepper_edge_count[STEPPER_NUM] = {0};

#define STEPPER_SPEED_Q_SHIFT      16
#define STEPPER_SPEED_Q            (1L << STEPPER_SPEED_Q_SHIFT)
#define STEPPER_SPEED_DELAY_Q      8
#define STEPPER_DYNAMIC_MAX_DELAY  (TIM_PERIOD * 2U)

static void Stepper_ResetIsrState(uint8_t motor_id)
{
  if (!STEPPER_ID_VALID(motor_id))
  {
    return;
  }

  stepper_last_accel_delay[motor_id] = 0;
  stepper_step_count[motor_id] = 0;
  stepper_rest[motor_id] = 0;
  stepper_edge_count[motor_id] = 0;
}

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
  stepper_speed_delay_k_q8[motor_id] =
    (uint32_t)(((10.0f * 2.0f * 3.14159f * (float)T1_FREQ * (float)(1U << STEPPER_SPEED_DELAY_Q)) /
                (float)pulses_per_rev) + 0.5f);
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

uint8_t Stepper_GetRuntimeSnapshot(uint8_t motor_id, StepperRuntimeSnapshot *snapshot)
{
  uint32_t primask;

  if ((!STEPPER_ID_VALID(motor_id)) || (snapshot == 0))
  {
    return FALSE;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  snapshot->position_steps = stepPosition[motor_id];
  snapshot->running = motor_status[motor_id].running;
  snapshot->out_ena = motor_status[motor_id].out_ena;
  snapshot->dir = srd[motor_id].dir;
  snapshot->continuous = srd[motor_id].continuous;
  snapshot->run_state = srd[motor_id].run_state;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return TRUE;
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

static uint32_t Stepper_SpeedToDelay(uint8_t motor_id, uint32_t speed)
{
  uint32_t delay;

  if ((speed == 0) || (stepper_speed_delay_k_q8[motor_id] == 0))
  {
    return 0;
  }

  delay = stepper_speed_delay_k_q8[motor_id] / (speed << STEPPER_SPEED_DELAY_Q);
  if (delay == 0)
  {
    delay = 1;
  }
  return delay;
}

static uint32_t Stepper_SpeedQ16ToDelay(uint8_t motor_id, int32_t speed_q16)
{
  uint32_t abs_speed_q16;
  uint32_t speed_q8;
  uint32_t delay;

  if (speed_q16 < 0)
  {
    abs_speed_q16 = (uint32_t)(-speed_q16);
  }
  else
  {
    abs_speed_q16 = (uint32_t)speed_q16;
  }

  if (abs_speed_q16 == 0)
  {
    return STEPPER_DYNAMIC_MAX_DELAY;
  }

  speed_q8 = (abs_speed_q16 + (1U << (STEPPER_SPEED_Q_SHIFT - STEPPER_SPEED_DELAY_Q - 1U))) >>
             (STEPPER_SPEED_Q_SHIFT - STEPPER_SPEED_DELAY_Q);
  if (speed_q8 == 0)
  {
    return STEPPER_DYNAMIC_MAX_DELAY;
  }

  delay = stepper_speed_delay_k_q8[motor_id] / speed_q8;
  if (delay == 0)
  {
    delay = 1;
  }
  if (delay > STEPPER_DYNAMIC_MAX_DELAY)
  {
    delay = STEPPER_DYNAMIC_MAX_DELAY;
  }
  return delay;
}

static uint32_t Stepper_SpeedDeltaQ16PerStep(uint32_t accel, uint32_t delay)
{
  uint32_t divisor;
  uint32_t delta;

  divisor = (uint32_t)(T1_FREQ >> STEPPER_SPEED_Q_SHIFT);
  if (divisor == 0)
  {
    divisor = 1;
  }

  delta = (uint32_t)(((accel * delay) + divisor - 1U) / divisor);
  if (delta == 0)
  {
    delta = 1;
  }
  return delta;
}

static void Stepper_SetContinuousTarget(uint8_t motor_id, uint8_t dir, uint8_t stop, uint32_t accel, uint32_t decel, uint32_t speed)
{
  int32_t signed_target;

  if (stop == TRUE)
  {
    signed_target = 0;
  }
  else if (dir != 0)
  {
    signed_target = -(int32_t)speed;
  }
  else
  {
    signed_target = (int32_t)speed;
  }

  srd[motor_id].target_accel = accel;
  srd[motor_id].target_decel = decel;
  srd[motor_id].target_speed = speed;
  srd[motor_id].target_speed_q16 = signed_target * STEPPER_SPEED_Q;
  srd[motor_id].target_min_delay = (stop == TRUE) ? STEPPER_DYNAMIC_MAX_DELAY : Stepper_SpeedToDelay(motor_id, speed);
  srd[motor_id].target_dir = (dir != 0) ? CCW : CW;
  srd[motor_id].stop_on_target = stop;
  srd[motor_id].speed_update_pending = FALSE;
}

static uint32_t Stepper_UpdateContinuousPlanner(uint8_t motor_id)
{
  int32_t current;
  int32_t target;
  int32_t delta;
  uint32_t accel;

  current = srd[motor_id].current_speed_q16;
  target = srd[motor_id].target_speed_q16;

  if (current == target)
  {
    if ((target == 0) && (srd[motor_id].stop_on_target == TRUE))
    {
      srd[motor_id].continuous = FALSE;
      srd[motor_id].run_state = STOP;
    }
    else
    {
      srd[motor_id].run_state = RUN;
    }
    return Stepper_SpeedQ16ToDelay(motor_id, current);
  }

  if (((current > 0) && (target < 0)) || ((current < 0) && (target > 0)) ||
      ((target == 0) && (current != 0)))
  {
    accel = srd[motor_id].target_decel;
    if (current > 0)
    {
      delta = (int32_t)Stepper_SpeedDeltaQ16PerStep(accel, srd[motor_id].step_delay);
      current = (current <= delta) ? 0 : (current - delta);
    }
    else
    {
      delta = (int32_t)Stepper_SpeedDeltaQ16PerStep(accel, srd[motor_id].step_delay);
      current = ((-current) <= delta) ? 0 : (current + delta);
    }
  }
  else
  {
    accel = (((current >= 0) && (current < target)) ||
             ((current <= 0) && (current > target))) ?
            srd[motor_id].target_accel : srd[motor_id].target_decel;
    delta = (int32_t)Stepper_SpeedDeltaQ16PerStep(accel, srd[motor_id].step_delay);
    if (current < target)
    {
      current = ((target - current) <= delta) ? target : (current + delta);
    }
    else
    {
      current = ((current - target) <= delta) ? target : (current - delta);
    }
  }

  if ((current == 0) && (target > 0) && (srd[motor_id].dir != CW))
  {
    srd[motor_id].dir = CW;
    MOTOR_DIR(motor_id, Stepper_GetDirLevel(motor_id, srd[motor_id].dir));
  }
  else if ((current == 0) && (target < 0) && (srd[motor_id].dir != CCW))
  {
    srd[motor_id].dir = CCW;
    MOTOR_DIR(motor_id, Stepper_GetDirLevel(motor_id, srd[motor_id].dir));
  }
  else if ((current > 0) && (srd[motor_id].dir != CW))
  {
    srd[motor_id].dir = CW;
    MOTOR_DIR(motor_id, Stepper_GetDirLevel(motor_id, srd[motor_id].dir));
  }
  else if ((current < 0) && (srd[motor_id].dir != CCW))
  {
    srd[motor_id].dir = CCW;
    MOTOR_DIR(motor_id, Stepper_GetDirLevel(motor_id, srd[motor_id].dir));
  }

  srd[motor_id].current_speed_q16 = current;

  if ((current == 0) && (target == 0) && (srd[motor_id].stop_on_target == TRUE))
  {
    srd[motor_id].continuous = FALSE;
    srd[motor_id].run_state = STOP;
    return STEPPER_DYNAMIC_MAX_DELAY;
  }

  srd[motor_id].run_state = (current == target) ? RUN :
                            (((current >= 0) && (current < target)) ||
                             ((current <= 0) && (current > target))) ? SPEED_ACCEL : SPEED_DECEL;
  return Stepper_SpeedQ16ToDelay(motor_id, current);
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
    printf("\n驱动器恢复运行，两路电机均为保持力矩状态");
  }
  else
  {
    printf("\n驱动器禁止输出，两路电机均为脱机状态");
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
  srd[motor_id].continuous = FALSE;
  srd[motor_id].speed_update_pending = FALSE;
  srd[motor_id].stop_on_target = FALSE;
  srd[motor_id].current_speed_q16 = 0;
  srd[motor_id].target_speed_q16 = 0;
  Stepper_ResetIsrState(motor_id);
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
 *  \param step   移动的步数，正数为顺时针，负数为逆时针
 *  \param accel  加速度，单位为0.1rad/sec^2
 *  \param decel  减速度，单位为0.1rad/sec^2
 *  \param speed  最大速度，单位为0.1rad/sec
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
  if (stepper_speed_delay_k_q8[motor_id] == 0)
  {
    Stepper_SetMotorPulsesPerRev(motor_id, pulses_per_rev);
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
  Stepper_ResetIsrState(motor_id);

  /* 4. 初始化运行状态机，计算加速段、匀速段和减速段参数 */
  srd[motor_id].continuous = FALSE;
  srd[motor_id].speed_update_pending = FALSE;
  srd[motor_id].target_dir = srd[motor_id].dir;
  srd[motor_id].stop_on_target = FALSE;
  srd[motor_id].target_min_delay = 0;
  srd[motor_id].target_speed = 0;
  srd[motor_id].target_accel = 0;
  srd[motor_id].target_decel = 0;
  srd[motor_id].current_speed_q16 = 0;
  srd[motor_id].target_speed_q16 = 0;

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
      srd[motor_id].step_delay = TIM_PERIOD * 2U;
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

  TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_ENABLE);
  __HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_ENABLE_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_MOE_ENABLE(&TIM_TimeBaseStructure);
  __HAL_TIM_ENABLE(&TIM_TimeBaseStructure);

  return STEPPER_CMD_OK;
}

StepperCmdResult Stepper_RunContinuous(uint8_t motor_id, uint8_t dir, uint32_t accel, uint32_t speed)
{
  int tim_count;
  uint32_t pulses_per_rev;
  float alpha;
  float a_t_x10;
  float a_sq;

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

  if ((accel == 0) || (speed == 0))
  {
    return STEPPER_CMD_PARAM_ERROR;
  }

  pulses_per_rev = stepper_pulses_per_rev[motor_id];
  if (pulses_per_rev == 0)
  {
    return STEPPER_CMD_PARAM_ERROR;
  }
  if (stepper_speed_delay_k_q8[motor_id] == 0)
  {
    Stepper_SetMotorPulsesPerRev(motor_id, pulses_per_rev);
  }

  alpha = (float)(2.0f * 3.14159f / pulses_per_rev);
  a_t_x10 = (float)(10.0f * alpha * T1_FREQ);
  a_sq = (float)(2.0f * 100000.0f * alpha);

  srd[motor_id].dir = (dir != 0) ? CCW : CW;
  MOTOR_DIR(motor_id, Stepper_GetDirLevel(motor_id, srd[motor_id].dir));
  Stepper_ResetIsrState(motor_id);

  srd[motor_id].min_delay = (int32_t)(a_t_x10 / speed);
  srd[motor_id].step_delay = (int32_t)((T1_FREQ_148 * sqrt(a_sq / accel)) / 10);
  if ((srd[motor_id].min_delay <= 0) || (srd[motor_id].step_delay <= 0))
  {
    return STEPPER_CMD_PARAM_ERROR;
  }

  if ((srd[motor_id].step_delay / 2) > TIM_PERIOD)
  {
    srd[motor_id].step_delay = TIM_PERIOD * 2U;
  }

  srd[motor_id].decel_start = 0xFFFFFFFFU;
  srd[motor_id].decel_val = 0;
  srd[motor_id].accel_count = 0;
  srd[motor_id].continuous = TRUE;
  srd[motor_id].current_speed_q16 = 0;
  Stepper_SetContinuousTarget(motor_id, srd[motor_id].dir, FALSE, accel, accel, speed);
  srd[motor_id].run_state = SPEED_ACCEL;

  motor_status[motor_id].running = TRUE;
  Stepper_UpdateGlobalStatus();

  tim_count = __HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,
                        stepper_hw[motor_id].pul_channel,
                        tim_count + srd[motor_id].step_delay / 2);

  TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_ENABLE);
  __HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_ENABLE_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);
  __HAL_TIM_MOE_ENABLE(&TIM_TimeBaseStructure);
  __HAL_TIM_ENABLE(&TIM_TimeBaseStructure);

  return STEPPER_CMD_OK;
}

StepperCmdResult Stepper_UpdateRunningSpeed(uint8_t motor_id, uint32_t accel, uint32_t decel, uint32_t speed)
{
  if (!STEPPER_ID_VALID(motor_id))
  {
    return STEPPER_CMD_ID_ERROR;
  }

  return Stepper_UpdateContinuousSpeedSigned(motor_id,
                                            srd[motor_id].dir,
                                            FALSE,
                                            accel,
                                            decel,
                                            speed);
}

StepperCmdResult Stepper_UpdateContinuousSpeedSigned(uint8_t motor_id, uint8_t dir, uint8_t stop, uint32_t accel, uint32_t decel, uint32_t speed)
{
  uint32_t primask;

  if (!STEPPER_ID_VALID(motor_id))
  {
    return STEPPER_CMD_ID_ERROR;
  }

  if (motor_status[motor_id].out_ena != TRUE)
  {
    return STEPPER_CMD_DISABLED;
  }

  if ((motor_status[motor_id].running != TRUE) ||
      (srd[motor_id].continuous != TRUE))
  {
    return STEPPER_CMD_BUSY;
  }

  if ((accel == 0) || (decel == 0) || ((speed == 0) && (stop != TRUE)))
  {
    return STEPPER_CMD_PARAM_ERROR;
  }

  if ((stop != TRUE) && (Stepper_SpeedToDelay(motor_id, speed) == 0))
  {
    return STEPPER_CMD_PARAM_ERROR;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  Stepper_SetContinuousTarget(motor_id, dir, stop, accel, decel, speed);
  if (primask == 0U)
  {
    __enable_irq();
  }

  return STEPPER_CMD_OK;
}

/**
  * @brief  速度决策，在TIM8比较中断中按通道调用
  * @param  motor_id 电机编号
  * @retval 无
  */
void speed_decision(uint8_t motor_id)
{
  uint32_t ccr;
  uint32_t new_step_delay;

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
    new_step_delay = srd[motor_id].step_delay;
    ccr = __HAL_TIM_GET_COMPARE(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_channel);
    ccr += srd[motor_id].step_delay / 2;
    __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_channel, ccr);

    /* 3. 每经过两次翻转，才算输出了一个完整步进脉冲 */
    stepper_edge_count[motor_id]++;
    if (stepper_edge_count[motor_id] == 2)
    {
      stepper_edge_count[motor_id] = 0;

      /* 4. 根据当前状态推进梯形加减速状态机 */
      switch (srd[motor_id].run_state)
      {
        case STOP:
          Stepper_ResetIsrState(motor_id);
          TIM_CCxChannelCmd(MOTOR_PUL_TIM, stepper_hw[motor_id].pul_channel, TIM_CCx_DISABLE);
          __HAL_TIM_DISABLE_IT(&TIM_TimeBaseStructure, stepper_hw[motor_id].pul_it);

          motor_status[motor_id].running = FALSE;
          srd[motor_id].continuous = FALSE;
          srd[motor_id].speed_update_pending = FALSE;
          Stepper_UpdateGlobalStatus();

          if (Stepper_IsOtherRunning(motor_id) != TRUE)
          {
            __HAL_TIM_DISABLE(&TIM_TimeBaseStructure);
          }
          return;

        case ACCEL:
          StepperCounter(motor_id, srd[motor_id].dir);
          stepper_step_count[motor_id]++;

          srd[motor_id].accel_count++;
          new_step_delay = srd[motor_id].step_delay - (((2 * (long)srd[motor_id].step_delay) + stepper_rest[motor_id]) / (4 * srd[motor_id].accel_count + 1));
          stepper_rest[motor_id] = ((2 * (long)srd[motor_id].step_delay) + stepper_rest[motor_id]) % (4 * srd[motor_id].accel_count + 1);

          if (stepper_step_count[motor_id] >= srd[motor_id].decel_start)
          {
            srd[motor_id].accel_count = srd[motor_id].decel_val;
            srd[motor_id].run_state = DECEL;
          }
          else if (new_step_delay <= srd[motor_id].min_delay)
          {
            stepper_last_accel_delay[motor_id] = new_step_delay;
            new_step_delay = srd[motor_id].min_delay;
            stepper_rest[motor_id] = 0;
            srd[motor_id].run_state = RUN;
          }
          break;

        case RUN:
          StepperCounter(motor_id, srd[motor_id].dir);
          stepper_step_count[motor_id]++;
          if (srd[motor_id].continuous == TRUE)
          {
            new_step_delay = Stepper_UpdateContinuousPlanner(motor_id);
            break;
          }

          new_step_delay = srd[motor_id].min_delay;

          if (stepper_step_count[motor_id] >= srd[motor_id].decel_start)
          {
            srd[motor_id].accel_count = srd[motor_id].decel_val;
            if (stepper_last_accel_delay[motor_id] == 0)
            {
              new_step_delay = srd[motor_id].step_delay;
            }
            else
            {
              new_step_delay = stepper_last_accel_delay[motor_id];
            }
            srd[motor_id].run_state = DECEL;
          }
          break;

        case DECEL:
          StepperCounter(motor_id, srd[motor_id].dir);
          stepper_step_count[motor_id]++;
          srd[motor_id].accel_count++;
          new_step_delay = srd[motor_id].step_delay - (((2 * (long)srd[motor_id].step_delay) + stepper_rest[motor_id]) / (4 * srd[motor_id].accel_count + 1));
          stepper_rest[motor_id] = ((2 * (long)srd[motor_id].step_delay) + stepper_rest[motor_id]) % (4 * srd[motor_id].accel_count + 1);

          if (srd[motor_id].accel_count >= 0)
          {
            srd[motor_id].run_state = STOP;
          }
          break;

        case SPEED_ACCEL:
          StepperCounter(motor_id, srd[motor_id].dir);
          stepper_step_count[motor_id]++;
          new_step_delay = Stepper_UpdateContinuousPlanner(motor_id);
          break;

        case SPEED_DECEL:
          StepperCounter(motor_id, srd[motor_id].dir);
          stepper_step_count[motor_id]++;
          new_step_delay = Stepper_UpdateContinuousPlanner(motor_id);
          break;
      }

      srd[motor_id].step_delay = new_step_delay;
    }
  }
}
