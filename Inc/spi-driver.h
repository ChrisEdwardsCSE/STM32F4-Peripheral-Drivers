/*
 * spi-driver.h
 *
 *  Created on: Jul 25, 2024
 *      Author: ched
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f4xx_hal.h"
///*
// * spi-driver.h
// *
// *  Created on: Jul 23, 2024
// *      Author: ched
// */
//#include "stm32f446xx.h"
//
//#ifndef STM32F4XX_HAL_DRIVER_INC_SPI_DRIVER_H_
//#define STM32F4XX_HAL_DRIVER_INC_SPI_DRIVER_H_


/* SPI_CR1 Bit definitions */
#define SPI_REG_CR1_BIDIMODE		( (uint32_t) 1 << 15)
#define SPI_ENABLE_2_LINE_UNI_DIR	0
#define SPI_ENABLE_1_LINE_BIDI		1

#define SPI_REG_CR1_DFF				( (uint32_t) 1 << 11)
#define SPI_8BIT_DF_ENABLE			0
#define SPI_16BIT_DF_ENABLE			1

#define SPI_REG_CR1_SSM				( (uint32_t) 1 << 9)
#define SPI_SSM_ENABLE				1
#define SPI_SSM_DISABLE				0

#define SPI_REG_CR1_SSI				( (uint32_t) 1 << 8)

#define SPI_CR1_LSB_FIRST			( (uint32_t) 1 << 7)
#define SPI_TX_MSB_FIRST			0
#define SPI_TX_LSB_FIRST			1

#define SPI_REG_CR1_SPE				( (uint32_t ) 1 << 6)

#define SPI_REG_CR1_BR_PCLK_DIV_2	( (uint32_t) 0 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_4	( (uint32_t) 1 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_8	( (uint32_t) 2 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_16	( (uint32_t) 3 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_32	( (uint32_t) 4 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_64	( (uint32_t) 5 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_128	( (uint32_t) 6 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_256	( (uint32_t) 7 << 3)

#define SPI_REG_CR1_MSTR			( (uint32_t) 1 << 2)
#define SPI_MASTER_MODE_SEL			1
#define SPI_SLAVE_MODE_SEL			0

#define SPI_REG_CR1_CPOL			( (uint32_t) 1 << 1)

#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

#define SPI_REG_CR1_CPHA			( (uint32_t) 1 << 0)
#define SPI_FIRST_CLOCK_TRANS		0
#define SPI_SECOND_CLOCK_TRANS		1

/* SPI_CR2 Bit definitions */
#define SPI_REG_CR2_TXEIE_ENABLE	( (uint32_t) 1 << 7)
#define SPI_REG_CR2_RXNEIE_ENABLE	( (uint32_t) 1 << 6)
#define SPI_REG_CR2_ERRIE_ENABLE	( (uint32_t) 1 << 5)

#define SPI_REG_CR2_FRAME_FORMAT	( (uint32_t) 1 << 4)
#define SPI_MOTOROLA_MODE			0
#define SPI_TI_MODE					1

#define SPI_REG_CR2_SSOE			( (uint32_t) 1 << 2)

/* SPI_SR Bit definitions */
#define SPI_REG_SR_FRE_FLAG			( (uint32_t) 1 << 8)
#define SPI_REG_SR_BUSY_FLAG		( (uint32_t) 1 << 7)
#define SPI_REG_SR_TXE_FLAG			( (uint32_t) 1 << 1)
#define SPI_REG_SR_RXNE_FLAG		( (uint32_t) 1 << 0)

#define SPI_1 						SPI1
#define SPI_2 						SPI2
#define SPI_3 						SPI3

#define SPI_IS_BUSY 				1
#define SPI_IS_NOT_BUSY				0

#define MYHAL_RCC_SPI1_CLK_ENABLE()	(RCC->APB2ENR |= (1 << 12))
#define MYHAL_RCC_SPI2_CLK_ENABLE()	(RCC->APB1ENR |= (1 << 14))
#define MYHAL_RCC_SPI3_CLK_ENABLE()	(RCC->APB1ENR |= (1 << 15))
#define MYHAL_RCC_SPI4_CLK_ENABLE() (RCC->APB2ENR |= (1 << 13))


/* SPI State Struct */
typedef enum
{
	MYHAL_SPI_STATE_RESET		= 0x00,	// not initialized or disabled
	MYHAL_SPI_STATE_READY		= 0x01,	// initialized and ready (ONLY state where it's good)
	MYHAL_SPI_STATE_BUSY		= 0x02, // busy
	MYHAL_SPI_STATE_BUSY_TX		= 0x12,	// TX busy
	MYHAL_SPI_STATE_BUSY_RX		= 0x22, // RX busy
	MYHAL_SPI_STATE_BUSY_TX_RX	= 0x32, // both busy
	MYHAL_SPI_STATE_ERROR		= 0x03	// error
} myhal_spi_state_t;

/* SPI Configuration Struct */
typedef struct
{
	uint32_t Mode;			// master or slave mode
	uint32_t Direction;		// bidi or 2 line uni di
	uint32_t DataSize;		// 8 bit or 16 bit data transmission
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
	uint32_t NSS;			// which slave management (HW or SW)
	uint32_t Prescaler;
	uint32_t FirstBit;		// data transfer start from MSB or LSB
} spi_init_t;

/* SPI handler struct */
typedef struct __spi_handler_t
{
	SPI_TypeDef		*Instance;		// SPI peripheral base address -  the actual peripheral
	spi_init_t		Init;			// SPI configuration params
	uint8_t			*pTxBuffPtr;	// pointer to TX buffer; a global, in-system buffer
	uint16_t		TxXferSz;		// TX transfer size
	uint16_t		TxXferCount;	// TX transfer count
	uint8_t			*pRxBuffPtr;	// pointer to RX buffer
	uint16_t		RxXferSz;		// RX transfer size
	uint16_t		RxXferCount;	// RX transfer counter
	myhal_spi_state_t	State;		// state of peripheral, must be ready
} spi_handler_t;



void init_gpio(void);

void init_clocks(void);

uint8_t spi_tx(spi_handler_t *spi_handler, uint8_t tx_data);
uint8_t spi_rx(spi_handler_t *spi_handler);
/**
 * Initialize SPI device
 * @param	spi_handler - base address of SPI peripheral
 */
void myhal_spi_init (spi_handler_t *spi_handler);

/**
 * Master TX
 *
 * @param	buf : poitner to TX buf
 * @param	len : length of TX data
 */
void myhal_spi_master_tx (spi_handler_t *spi_handler, uint8_t *tx_buf, uint32_t len);

/**
 * Slave TX
 *
 *
 *
 */
void myhal_spi_slave_tx (spi_handler_t *spi_handler, uint8_t *tx_buf, uint32_t len);

/**
 * Slave RX
 *
 * @para rcv_buf : pointer to RX buf
 *
 */
void myhal_spi_slave_rx (spi_handler_t *spi_handler, uint8_t *rx_buf, uint32_t len);

/**
 * Master RX
 *
 *
 *
 */
void myhal_spi_master_rx (spi_handler_t *spi_handler, uint8_t *rx_buf, uint32_t len);

/**
 * SPI IRQ Handler
 * @param	hspi : pointer to spi handler_t struc which contains config info for SPI
 */
void myhal_i2c_spi_irq_handler(spi_handler_t *hspi);

/**
 * Handles TXE interrupt, only gets called when TXEIE = 1, TXE = 1
 * Calls from within master or slave TX, so TX buffer address already in pTXBuffPtr
 */
void myhal_spi_handle_tx_int(spi_handler_t *hspi);

/**
 * Handles RXNE interrupt, only gets called when RXNEIE=1 AND RXNE=1
 * RXNE means there's stuff to be read in RX_buf, we have RXBuf in pRXBuffPtr, so just access
 * that way.
 */
void myhal_spi_handle_rx_int(spi_handler_t *hspi);


#endif /* INC_SPI_DRIVER_H_ */
