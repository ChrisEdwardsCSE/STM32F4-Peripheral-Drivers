/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: May 11, 2024
 *      Author: ched
 */

#include "stm32f446xx_gpio_driver.h"

/*********** Helper functions ***********/
/**
 * Configures mode of pin : input, output, alt, or analog mode
 * @param *GPIOx : GPIO Port Base Address
 * @param pin_no : pin number
 * @param mode	 : the mode
 */
static void myhal_gpio_conf_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t mode) {
	GPIOx->MODER |= (mode << (2 * pin));
	// b/c MODER register is 32 bits, 2 bits for the mode conf of each pin per port. so multiply pin
	// by 2 to get to the rigth place and input mode there (mode is 2 bits) defined in .h file
}

/**
 * Configures output type of pin
 * @param o_type : output type (0: push pull. 1: open drain)
 */
static void myhal_gpio_conf_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t o_type) {
	if (pin > 15) {
		// fail
	} else {
		GPIOx->OTYPER |= (o_type << pin);
	}
}

/**
 * Configure speed of pin
 * @param speed	: speed value (low, medium, high, very high)
 */
static void myhal_gpio_conf_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t speed) {
	GPIOx->OSPEEDR |= (speed << (2 * pin));
}

/**
 * @Configure pull up or pull down resistors
 * @param pupd : pull up or pull down
 */
void myhal_gpio_conf_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t pupd) {
	GPIOx->PUPDR |= (pupd << (2 * pin));
}

/**
 * Configure alternate function for given pin
 * @param altfunc : the alternate function
 */
void myhal_gpio_conf_pin_af(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t altfunc) {
	if (pin < 8) {
		GPIOx->AFR[0] |= (altfunc << (pin * 4));
	} else {
		GPIOx->AFR[1] |= (altfunc << ((pin % 8) * 4) );
	}
}

/**
 * Read pin value
 */
uint8_t myhal_gpio_read_pin(GPIO_TypeDef *GPIOx, uint16_t pin) {
	uint8_t value = (GPIOx->IDR >> pin) & (uint8_t)1;
	return value;
}

/**
 * Write pin value
 */
void myhal_gpio_write_pin(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t val)
{
	if (val)
		(GPIOx->ODR) = (GPIOx->ODR) | (1 << pin);
	else
		GPIOx->ODR &= ~(1 << pin);
}

/**
 * Initialize GPIO pin with given values
 */
void myhal_gpio_init(GPIO_TypeDef *GPIOx, gpio_conf_struct *gpio_conf)
{
	myhal_gpio_conf_pin_mode(GPIOx, gpio_conf->pin, gpio_conf->mode);
	myhal_gpio_conf_pin_otype(GPIOx, gpio_conf->pin, gpio_conf->o_type);
	myhal_gpio_conf_pin_speed(GPIOx, gpio_conf->pin, gpio_conf->speed);
	myhal_gpio_conf_pin_pupd(GPIOx, gpio_conf->pin, gpio_conf->pull);
	myhal_gpio_conf_pin_af(GPIOx, gpio_conf->pin, gpio_conf->altfunc);
}

/*********** INTERRUPTS ***********/

/**
 * Configures interrupt for pin
 * @param edge_sel : edge selection value
 */
void myhal_gpio_conf_interrupt(uint16_t pin, int_edge_sel_t edge_sel) {
	if (edge_sel == INT_RISING_EDGE)
	{
		EXTI->RTSR |= (1 << pin);
	}
	else if (edge_sel == INT_FALLING_EDGE)
	{
		EXTI->FTSR |= (1 << pin);
	}
	else if (edge_sel == INT_RISING_FALLING_EDGE)
	{
		EXTI->RTSR |= (1 << pin);
		EXTI->FTSR |= (1 << pin);
	}
}

/**
 * Enables interrupt for pin and IRQ number
 * @param irq_no : irq number on NVIC to be enabled
 */
void myhal_gpio_enable_interrupt(uint16_t pin, IRQn_Type irq_no)
{
	/* FIX - setting IMR bit sometimes sets PR bit immediately causing interrupt */
	EXTI->IMR |= (1 << pin); // enables pin on EXTI line
	NVIC_EnableIRQ(irq_no); // enables interrupt reception for pin on NVIC
}

/**
 * Clear the interrupt pending bit on pin if set
 */
void myhal_gpio_clear_interrupt(uint16_t pin) {
	if (EXTI->PR & (1 << pin))
	{
		EXTI->PR = 0x2000U;
	}
}


