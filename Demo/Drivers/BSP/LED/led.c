#include "led.h"

void LED_GPIO_init()
{
	GPIO_InitTypeDef gpio;
	
	LED_RED_GPIO_CLK_ENABLE();
	LED_GREEN_GPIO_CLK_ENABLE();
	
	gpio.Pin=LED_RED_PIN;
	gpio.Mode=GPIO_MODE_OUTPUT_PP;
	gpio.Pull=GPIO_PULLUP;
	gpio.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LED_RED_PORT,&gpio);
	
	gpio.Pin=LED_GREEN_PIN;
	HAL_GPIO_Init(LED_GREEN_PORT,&gpio);
	
	LED_RED(0);
	LED_GREEN(0);
}
