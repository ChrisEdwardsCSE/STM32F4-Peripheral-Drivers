/*
 * gpio-driver.c
 * GPIO Driver for STM32F4 series.
 * 
 *  Created on: May 11, 2024
 *      Author: Christopher Edwards
 */

#include "gpio-driver.h"

/*********** Helper functions ***********/
/**
 * Configures mode of pin : input, output, alt, or analog mode
 * @param *GPIOx - GPIO Port Base Address
 * @param pin_no - Pin number
 * @param mode	 - Mode of pin
 */
static void __GPIO_Conf_Mode(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t mode) {
	// MODER 32 bits, 2 bits for mode of each pin per port
	GPIOx->MODER |= (mode << (2 * pin));
}

/**
 * Configures output type of pin
 * @param o_type - Output type (0: push pull. 1: open drain)
 */
static void __GPIO_Conf_OType(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t o_type) {
	if (pin > 15) {
		// fail
	} else {
		GPIOx->OTYPER |= (o_type << pin);
	}
}

/**
 * Configure speed of pin
 * 
 * @param speed	- Speed (low, medium, high, very high)
 */
static void __GPIO_Conf_Speed(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t speed) {
	GPIOx->OSPEEDR |= (speed << (2 * pin));
}

/**
 * @Configure pull up or pull down resistors
 * 
 * @param pupd - pull up or pull down
 */
void GPIO_Conf_PUPD(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t pupd) {
	GPIOx->PUPDR |= (pupd << (2 * pin));
}

/**
 * Configure alternate function for given pin
 * 
 * @param altfunc - the alternate function
 */
void GPIO_Conf_AltFunc(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t altfunc) {
	if (pin < 8) {
		GPIOx->AFR[0] |= (altfunc << (pin * 4));
	} else {
		GPIOx->AFR[1] |= (altfunc << ((pin % 8) * 4) );
	}
}

/**
 * Read pin value
 * 
 * @return - value of the pin, 1 or 0
 */
uint8_t GPIO_Read_Pin(GPIO_TypeDef *GPIOx, uint16_t pin) {
	uint8_t value = (GPIOx->IDR >> pin) & (uint8_t)1;
	return value;
}

/**
 * Write pin value
 * 
 * @param val - Value to write to pin
 */
void GPIO_Write_Pin(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t val)
{
	if (val)
		(GPIOx->ODR) = (GPIOx->ODR) | (1 << pin);
	else
		GPIOx->ODR &= ~(1 << pin);
}

/**
 * Initialize GPIO pin with given values
 * 
 * @param gpio_conf - GPIO initialization struct
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, gpio_conf_struct *gpio_conf)
{
	GPIO_Conf_Mode(GPIOx, gpio_conf->pin, gpio_conf->mode);
	GPIO_Conf_OType(GPIOx, gpio_conf->pin, gpio_conf->o_type);
	GPIO_Conf_Speed(GPIOx, gpio_conf->pin, gpio_conf->speed);
	GPIO_Conf_PUPD(GPIOx, gpio_conf->pin, gpio_conf->pull);
	GPIO_Conf_AltFunc(GPIOx, gpio_conf->pin, gpio_conf->altfunc);
}

/*********** INTERRUPTS ***********/

/**
 * Configures interrupt for pin
 * @param edge_sel - edge selection value
 */
void GPIO_Conf_Int(uint16_t pin, int_edge_sel_t edge_sel) {
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
 * @param irq_no - irq number on NVIC to be enabled
 */
void GPIO_Int_Enable(uint16_t pin, IRQn_Type irq_no)
{
	/* FIX - setting IMR bit sometimes sets PR bit immediately causing interrupt */
	EXTI->IMR |= (1 << pin); // enables pin on EXTI line
	NVIC_EnableIRQ(irq_no); // enables interrupt reception for pin on NVIC
}

/**
 * Clear the interrupt pending bit on pin if set
 */
void GPIO_Int_Clear(uint16_t pin) {
	if (EXTI->PR & (1 << pin))
	{
		EXTI->PR = 0x2000U;
	}
}
