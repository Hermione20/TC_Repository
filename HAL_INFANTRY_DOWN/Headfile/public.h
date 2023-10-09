#ifndef __PUBLIC_H
#define __PUBLIC_H


/**************ST HEAD*********************/
//#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "main.h"


//#include "stm32f4xx_it.h"
#include <stdio.h>

/*************MATH HEAD********************/
#include <math.h>
#include <arm_math.h>

#define u8   uint8_t
#define u16  uint16_t
#define u32  uint32_t 
#define s32  int32_t  
#define s16  int16_t


//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//定义 角度(度)转换到 弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

/**************TOOL HEAD*******************/
#include <string.h>
#include <stdarg.h>


/*************Algorithm********************/
#include "oldpid.h"





/**************senior**********************/
#include "CanBus.h"
#include "CH100.h"
#include "JUDGE.h"
#include "DJI_MOTOR.h"
#include "HI220.h"
#include "HT430.h"
//#include "Auto_shoot.h"
#include "LK_TECH.h"
#include "REMOTE.h"
#include "PM01.h"
#include "can_chassis_transmit.h"


#include "senior.h"
/***************TASK*********************/

#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"


/**************RTOS**********************/


#endif
