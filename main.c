/*
 * main.c
 *
 *  Created on: May 11, 2024
 *      Author: ched
 */

#include "main.h"
#include "stm32f446xx_gpio_driver.h"

// interrupt handler for EXTI13 (the one with green LED)
void EXTI15_10_IRQHandler(void)
{
	myhal_gpio_clear_interrupt(13);

	myhal_gpio_write_pin(GPIOA, 5, 1);
	for (int i = 0; i < 500000; i++);
	myhal_gpio_write_pin(GPIOA, 5, 0);
	for (int i = 0; i < 500000; i++);
}

int main(void) {

	/* **** Blink green LED for a split second*/
	GPIO_TypeDef *x = GPIOA;
	gpio_conf_struct green_led = {0};
	// enable the clock for GPIOA peripheral
	RCC_GPIOA_CLK_ENABLE();

	// configure LED with all necessary configurations
	green_led.pin = 5;
	green_led.mode = GPIO_PIN_O_MODE;
	green_led.pull = GPIO_PIN_NO_PULL;
	green_led.speed = GPIO_PIN_SPEED_MED;
	green_led.o_type = GPIO_PIN_OTYPE_PP;
	myhal_gpio_init(x, &green_led); // actually configures GPIO pin using the struct created

//	toggle LED real quick
//	myhal_gpio_write_pin(GPIOA, 5, 1);
//
//	for (int i = 0; i < 50000; i++);
//
//	myhal_gpio_write_pin(GPIOA, 5, 0);

	// try to do interrupt with B1, didn't really work. PR goes high when IMR does
	// and so interrupt triggered immediately without pressing button
	GPIO_TypeDef *y = GPIOC;
	gpio_conf_struct b1_button = {0};
	// configure GPIO pin
	b1_button.pin = 13;
	b1_button.mode = GPIO_PIN_I_MODE;
	b1_button.pull = GPIO_PIN_NO_PULL;
	myhal_gpio_init(y, &b1_button);

	// enable clock for B1 interrupt
	RCC_GPIOC_CLK_ENABLE();

	// configures interrupt with EXTI->R/FTSR
	myhal_gpio_conf_interrupt(13, INT_FALLING_EDGE);
	for (int i = 0; i < 50000; i++);

	// enables interrupt with EXTI->IMR
	myhal_gpio_enable_interrupt(13, EXTI15_10_IRQn);

	while (1) {

	}
	return 0;

}


