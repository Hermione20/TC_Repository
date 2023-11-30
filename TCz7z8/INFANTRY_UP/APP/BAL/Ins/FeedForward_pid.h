#ifndef __FEEDFORWARD_PID_H
#define __FEEDFORWARD_PID_H
#include "stm32f4xx.h"

#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

typedef struct
{
	float	k1;//ֱ��ǰ��
	float k2;//һ�ײ��ǰ��
	float set;//���ڵ�Ŀ��ֵ
	float last_set;//�ϴε�Ŀ��ֵ
	float out;//ǰ����������ֵ
	float max_out;//���ǰ�����
}Feedforward_pid_t;

void Feedforward_pid_init(Feedforward_pid_t *FF,float k1,float k2,float max_out);
float Feedforward_pid_calc(Feedforward_pid_t *FF); 
extern Feedforward_pid_t FF_yaw;
#endif