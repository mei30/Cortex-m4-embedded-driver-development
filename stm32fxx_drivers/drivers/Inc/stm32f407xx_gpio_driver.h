/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 16, 2023
 *      Author: meisam
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct
{
    GPIO_RegDef_t* pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
* Peripheral clock control
*/
void GPIO_PreClockControl(void);

void GPIO_Init(void);
void GPIO_DeInit(void);
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);
void GPIO_IRQControl(void);
void GPIO_IRQHandling(void);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
