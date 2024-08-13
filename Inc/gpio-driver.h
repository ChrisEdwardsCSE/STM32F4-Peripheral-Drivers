/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: May 11, 2024
 *      Author: Christopher Edwards
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/* Define Pin Modes */
#define GPIO_PIN_I_MODE 	( (uint32_t) 0x00)
#define GPIO_PIN_O_MODE 	( (uint32_t) 0x01)
#define GPIO_PIN_AF_MODE 	( (uint32_t) 0x02)

#define GPIO_PIN_OTYPE_PP 	( (uint32_t) 0x00)
#define GPIO_PIN_OTYPE_OD 	( (uint32_t) 0x01)

#define GPIO_PIN_SPEED_LOW 			( (uint32_t) 0x00)
#define GPIO_PIN_SPEED_MED 			( (uint32_t) 0x01)
#define GPIO_PIN_SPEED_HIGH 		( (uint32_t) 0x02)
#define GPIO_PIN_SPEED_VERY_HIGH	( (uint32_t) 0x03)

#define GPIO_PIN_NO_PULL 	( (uint32_t) 0x00)
#define GPIO_PIN_PULL_DOWN 	( (uint32_t) 0x01)
#define GPIO_PIN_PULL_UP 	( (uint32_t) 0x11)

/* GPIO Clock Enables */
#define RCC_GPIOA_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<0))
#define RCC_GPIOB_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<1))
#define RCC_GPIOC_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<2))
#define RCC_GPIOD_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<3))
#define RCC_GPIOE_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<4))
#define RCC_GPIOF_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<5))
#define RCC_GPIOG_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<6))
#define RCC_GPIOH_CLK_ENABLE()	(RCC->AHB1ENR |= (1<<7))

/* GPIO Initialization Configuration Struct */
typedef struct {
	uint32_t pin; 		// GPIO pins of port
	uint32_t mode; 		// Mode of pins
	uint32_t o_type; 	// output type of pins
	uint32_t pull;		// pull-up or pull-down
	uint32_t speed;		// speed of pins
	uint32_t altfunc;	// alternate function
} gpio_conf_struct;

/* Interrupt Edge Configuration */
typedef enum {
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE // either edge
} int_edge_sel_t;

/*********** Helper functions ***********/

/**
 * @Configure pull up or pull down resistors
 * 
 * @param pupd - pull up or pull down
 */
void GPIO_Conf_PUPD(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t pupd);


/**
 * Configure alternate function for given pin
 * 
 * @param altfunc - the alternate function
 */
void GPIO_Conf_AltFunc(GPIO_TypeDef *GPIOx, uint16_t pin, uint32_t altfunc);

/**
 * Read pin value
 * 
 * @return - value of the pin, 1 or 0
 */
uint8_t GPIO_Read_Pin(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
 * Write pin value
 * 
 * @param val - Value to write to pin
 */
void GPIO_Write_Pin(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t val);

/**
 * Initialize GPIO pin with given values
 * 
 * @param gpio_conf - GPIO initialization struct
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, gpio_conf_struct *gpio_conf);

/**
 * Configures interrupt for pin
 * @param edge_sel - edge selection value
 */
void GPIO_Conf_Int(uint16_t pin, int_edge_sel_t edge_sel);

/**
 * Enables interrupt for pin and IRQ number
 * @param irq_no - irq number on NVIC to be enabled
 */
void GPIO_Int_Enable(uint16_t pin, IRQn_Type irq_no);

/**
 * Clear the interrupt pending bit on pin if set
 */
void GPIO_Int_Clear(uint16_t pin);

#endif /* INC_GPIO_DRIVER_H_ */