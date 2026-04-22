#ifndef __BSP_STEPPER_T_SPEED_H
#define	__BSP_STEPPER_T_SPEED_H

#include "stm32f4xx.h"
#include "./stepper/bsp_stepper_init.h"
#include "math.h"

/* 单台步进电机的梯形加减速运行参数 */
typedef struct {
  unsigned char run_state : 3; /* 当前运行状态：STOP/ACCEL/DECEL/RUN */
  unsigned char dir : 1;       /* 当前方向：CW/CCW */
  unsigned int step_delay;     /* 下一次比较事件的延时周期 */
  unsigned int decel_start;    /* 开始进入减速段的步数位置 */
  signed int decel_val;        /* 减速段补偿值 */
  signed int min_delay;        /* 匀速段最小延时，对应最大速度 */
  signed int accel_count;      /* 加减速段计数器 */
} speedRampData;

/* 系统级状态标志 */
struct GLOBAL_FLAGS {
  unsigned char running:1; /* 当前是否存在任意电机正在运行 */
  unsigned char cmd:1;     /* 串口是否接收到待处理命令 */
  unsigned char out_ena:1; /* 驱动器输出是否使能 */
};

typedef enum {
  STEPPER_CMD_OK = 0,
  STEPPER_CMD_BUSY,
  STEPPER_CMD_DISABLED,
  STEPPER_CMD_ID_ERROR,
  STEPPER_CMD_PARAM_ERROR
} StepperCmdResult;

#define FALSE             0
#define TRUE              1
#define CW                0 // 顺时针
#define CCW               1 // 逆时针

#define STOP              0 // 停止状态
#define ACCEL             1 // 加速状态
#define DECEL             2 // 减速状态
#define RUN               3 // 匀速状态

/*频率相关参数*/
#define TIM_PRESCALER      31
#define T1_FREQ           (SystemCoreClock/(TIM_PRESCALER+1))
#define T1_FREQ_148       ((float)((T1_FREQ*0.676)/10))

/* 默认值仅用于上电初始化，实际运行参数由各设备单独下发 */
#define STEPPER_DEFAULT_PULSES_PER_REV  6400U

extern speedRampData srd[STEPPER_NUM];
extern int stepPosition[STEPPER_NUM];
extern struct GLOBAL_FLAGS status;
extern struct GLOBAL_FLAGS motor_status[STEPPER_NUM];

void speed_decision(uint8_t motor_id);
void MSD_ENA(FunctionalState NewState);
void Stepper_SetMotorPulsesPerRev(uint8_t motor_id, uint32_t pulses_per_rev);
StepperCmdResult stepper_move_T(uint8_t motor_id, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
uint8_t Stepper_IsAnyRunning(void);

#endif
