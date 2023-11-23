/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 16, 2023
 *      Author: meisam
 */

#include "stm32f407xx_gpio_driver.h"

void GPIO_PreClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLOCK_EN();
        }

        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLOCK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLOCK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLOCK_DI();
        }

        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLOCK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLOCK_DI();
        }
    }
}

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
    uint32_t temp = 0;

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (
                2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (
                2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;
        temp = 0;
    }
    else
    {
    	  // Configuring Edge Detection
    	  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
    	  {
    		  EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		  EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	  }
    	  else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
    	  {
    		  EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		  EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	  }
    	  else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
    	  {
    		  EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		  EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	  }

    	  // Configuring the GPIO port selection in SYSCFG_EXTICR
    	  int temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
    	  int temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

    	  uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

    	  SYSCFG_PCLOCK_EN();

    	  SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

    	  // Configure delivering EXTI interrupt line in IMR
		  EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    }

    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (
            2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (
            2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER |= temp;

    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (
            2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (
            2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (
             pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(1 << (
            pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALFN)
    {
        int temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        int temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (
                pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }

    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
    uint8_t value;

    value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);

    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
    uint16_t value;

    value = pGPIOx->IDR;

    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t Value)
{
    pGPIOx->ODR |= Value << pinNumber;
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
    pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IRQITControl(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
            *NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (31 < IRQNumber && IRQNumber < 64)
		{
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (64 <= IRQNumber && IRQNumber < 96)
		{

            *NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (31 < IRQNumber && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        }
        else if (64 <= IRQNumber && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
	}
}

void GPIO_IRQPRIControl(uint8_t IRQNumber, uint8_t IRQPriority)
{
    const uint8_t iprx = IRQNumber / 4;
    const uint8_t iprx_section = IRQNumber % 4;

    uint8_t amount_shift = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NIVIC_PRI_BASE_ADDR + iprx) |= (IRQPriority << amount_shift);
}

void GPIO_IRQHandling(uint8_t pinNumber)
{
    if (EXTI->PR & (1 << pinNumber))
    {
        EXTI->PR |= (1 << pinNumber);
    }
}
