/*
 * stm32f407xx.h
 *
 *  Created on: Nov 14, 2023
 *      Author: meisam
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/*
 * The BASEADDR address of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x20001C00U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx and APBx bus Peripheral BASEADDR address
 */

#define PERIPH_BASEADDR		0x40000000U
#define APB1PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR	0x40010000U
#define AHB1PERIPH_BASEADDR	0x40020000U
#define AHB2PERIPH_BASEADDR	0x50000000U

/*
 * BASEADDR addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define 	I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define 	I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define 	I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)

#define 	SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define 	SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)

#define 	USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define 	USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define 	UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define 	UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)

// Peripheral register definition structures

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_RegDef_t;

/*
* Peripheral definistions
*/

#define GPAIOA		((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPAIOB		((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPAIOC		((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPAIOD		((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPAIOE		((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPAIOF		((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPAIOG		((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPAIOH		((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPAIOI		((GPIO_RegDef_t *) GPIOI_BASEADDR)


#endif /* INC_STM32F407XX_H_ */
