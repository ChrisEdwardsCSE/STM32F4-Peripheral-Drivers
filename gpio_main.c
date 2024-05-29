/*
 * main.c
 *
 *  Created on: May 11, 2024
 *      Author: ched
 */

#include "main.h"
#include "stm32f446xx_gpio_driver.h"

int main(void) {
	GPIO_TypeDef *ptrGPIOA = GPIOA;
	gpio_conf_struct green_led = {0};
	
	// Enable the clock for GPIOA & GPIOC peripheral
	RCC_GPIOA_CLK_ENABLE();
	RCC_GPIOC_CLK_ENABLE();

	// Configure Green LED pin
	green_led.pin = 5;
	green_led.mode = GPIO_PIN_O_MODE;
	green_led.pull = GPIO_PIN_NO_PULL;
	green_led.speed = GPIO_PIN_SPEED_MED;
	green_led.o_type = GPIO_PIN_OTYPE_PP;
	myhal_gpio_init(ptrGPIOA, &green_led); // actually configures GPIO pin using the struct created


	// 
	GPIO_TypeDef *ptrGPIOC = GPIOC;
	gpio_conf_struct b1_button = {0};
	
	// Configure B1 button pin
	b1_button.pin = 13;
	b1_button.mode = GPIO_PIN_I_MODE;
	b1_button.pull = GPIO_PIN_NO_PULL;
	myhal_gpio_init(ptrGPIOC, &b1_button);

	

	// configures interrupt with EXTI->R/FTSR
	myhal_gpio_conf_interrupt(13, INT_FALLING_EDGE);
	for (int i = 0; i < 50000; i++);

	// enables interrupt with EXTI->IMR
	myhal_gpio_enable_interrupt(13, EXTI15_10_IRQn);

	while (1) {

	}
	return 0;
}

/* Interrupt Handler for EXTI13 */
void EXTI15_10_IRQHandler(void)
{
	myhal_gpio_clear_interrupt(13);

	myhal_gpio_write_pin(GPIOA, 5, 1);
	for (int i = 0; i < 500000; i++);
	myhal_gpio_write_pin(GPIOA, 5, 0);
	for (int i = 0; i < 500000; i++);
}


