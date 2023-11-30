
#ifndef __LED_H
#define __LED_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/*----------------------------------------------------------------------------*/


/* Port_Definition -----------------------------------------------------------*/
#define LED_GREEN_PORT							GPIOC
#define LED_GREEN_PIN								GPIO_PIN_1
#define LED_GREEN_GPIO_CLK_ENABLE()	do{__HAL_RCC_GPIOC_CLK_ENABLE();}while(0)

#define LED_RED_PORT                GPIOC
#define LED_RED_PIN									GPIO_PIN_2
#define LED_RED_GPIO_CLK_ENABLE()		do{__HAL_RCC_GPIOC_CLK_ENABLE();}while(0)
/*----------------------------------------------------------------------------*/

/* LED_Port ------------------------------------------------------------------*/
#define LED_RED(x)                  do{x?\
																			 HAL_GPIO_WritePin(LED_RED_PORT,LED_RED_PIN,GPIO_PIN_SET):\
																			 HAL_GPIO_WritePin(LED_RED_PORT,LED_RED_PIN,GPIO_PIN_RESET);}while(0)
#define LED_GREEN(x)                do{x?\
																			 HAL_GPIO_WritePin(LED_GREEN_PORT,LED_GREEN_PIN,GPIO_PIN_SET):\
																			 HAL_GPIO_WritePin(LED_GREEN_PORT,LED_GREEN_PIN,GPIO_PIN_RESET);}while(0)		

#define LED_RED_TOGGLE()            do{HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);}while(0)     
#define LED_GREEN_TOGGLE()          do{HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);}while(0)													 
/*----------------------------------------------------------------------------*/


/* Function_Definition -------------------------------------------------------*/
void LED_GPIO_init(void);
/*----------------------------------------------------------------------------*/																			 
																			 
#endif
