#ifndef __MAIN_H__
#define __MAIN_H__
#include "common.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_spi.h"
#include "SuperviseTask.h"
#include "ad.h"
#include "gun.h"
#include "common.h"
#include "can1.h"
#include "can2.h"
#include "delay.h"
#include "timer.h"
#include "usart1.h"
#include "usart3.h"
#include "uart4.h"
#include "led.h"
#include "oled.h"
#include "bsp_flash.h"
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include <string.h>
#include <stdarg.h>
#include "ramp.h"
#include "CanBusTask.h"
#include "pid.h"
#include "pid_regulator.h"      
#include "laser.h"
#include "bsp.h"
#include "encoder.h"
#include "ControlTask.h"
#include "LostCounter.h"
#include "SuperviseTask.h"
#include "RemoteTask.h"
#include "IOTask.h"
#include "protocal.h"
#include "FIFO.h"
#include "Judge.h"
#include "config.h"
#include "output2vs.h"
#include "oled.h"
#include "Key_scan.h"
#include "iwdg.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "icm20948.h"
#include "icm20948_driver.h"
#include "spi.h"
#include "adc.h"
#include "HI220.h"
#include "kalman_filter.h"
#include "client.h"
#include "auto_angle_task.h"
#include "ramp_second.h"
#include "distance_measure.h"
#include "protobuf-c.h"
#include "Attack.pb-c.h"
#include "Signal.pb-c.h"
#include "TurretCommand.pb-c.h"
#include "Auto_Shoot_Control_Task.h" 

#endif 
