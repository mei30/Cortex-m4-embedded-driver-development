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
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

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

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLL_CFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RST;
	uint32_t RESERVED;
	uint32_t RESERVED;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED;
	uint32_t RESERVED;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;	
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED;
	uint32_t RESERVED;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED;
	uint32_t RESERVED;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2CFGR;
	volatile uint32_t PLLSAICFGR; 
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

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

#define RCC			((RCC_RegDef_t *) RCC_BASEADDR)


/* Clock enable macros for GPIOx peripherals */
#define GPIOA_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLOCK_EN()		(RCC->AHB1ENR |= (1 << 8))

/* Clock enable macros for I2Cx peripherals */
#define I2C1_PCLOCK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLOCK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLOCK_EN()		(RCC->APB1ENR |= (1 << 23))

/* Clock enable macros for SPIx peripherals */
#define SPI1_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 14))
#define SPI3_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 15))
#define SPI4_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 13))

/* Clock enable macros for USARTx peripherals */
#define USART1_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLOCK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 18))
#define UART4_PCLOCK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLOCK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 5))

/* Clock enable macros for SYSCFG peripherals */
#define SYSCFG_PCLOCK_EN()		(RCC->APB2ENR |= (1 << 14))

/* Clock disable macros for GPIOX peripherals */
#define GPIOA_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLOCK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/* GPIOx Reset Peripherals */
#define GPIOA_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/* Generap purpose Macros*/
#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#endif /* INC_STM32F407XX_H_ */
