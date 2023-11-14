/*
 * stm32f407xx.h
 *
 *  Created on: Nov 14, 2023
 *      Author: meisam
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * The base address of Flash and SRAM memories
 */

#define FLASH_BASEADDRESS		0x08000000U
#define SRAM1_BASEADDRESS		0x20000000U
#define SRAM2_BASEADDRESS		0x20001C00U
#define ROM_BASEADDRESS			0x1FFF0000U
#define SRAM					SRAM1_BASEADDRESS

#endif /* INC_STM32F407XX_H_ */
