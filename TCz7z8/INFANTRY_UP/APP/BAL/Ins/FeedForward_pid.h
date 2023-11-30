#ifndef __FEEDFORWARD_PID_H
#define __FEEDFORWARD_PID_H
#include "stm32f4xx.h"

#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

typedef struct
{
	float	k1;//直接前馈
	float k2;//一阶差分前馈
	float set;//现在的目标值
	float last_set;//上次的目标值
	float out;//前馈计算的输出值
	float max_out;//最大前馈输出
}Feedforward_pid_t;

void Feedforward_pid_init(Feedforward_pid_t *FF,float k1,float k2,float max_out);
float Feedforward_pid_calc(Feedforward_pid_t *FF); 
extern Feedforward_pid_t FF_yaw;
#endif