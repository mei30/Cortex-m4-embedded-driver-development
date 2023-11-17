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
        if (pGPIOx == GPAIOA)
        {
            GPIOA_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOB)
        {
            GPIOB_PCLOCK_EN();
        }

        else if (pGPIOx == GPAIOC)
        {
            GPIOC_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOD)
        {
            GPIOD_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOE)
        {
            GPIOE_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOF)
        {
            GPIOF_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOG)
        {
            GPIOG_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOH)
        {
            GPIOH_PCLOCK_EN();
        }
        else if (pGPIOx == GPAIOI)
        {
            GPIOI_PCLOCK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPAIOA)
        {
            GPIOA_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOB)
        {
            GPIOB_PCLOCK_DI();
        }

        else if (pGPIOx == GPAIOC)
        {
            GPIOC_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOD)
        {
            GPIOD_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOE)
        {
            GPIOE_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOF)
        {
            GPIOF_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOG)
        {
            GPIOG_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOH)
        {
            GPIOH_PCLOCK_DI();
        }
        else if (pGPIOx == GPAIOI)
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
    if (pGPIOx == GPAIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPAIOB)
    {
        GPIOB_REG_RESET();
    }

    else if (pGPIOx == GPAIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPAIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPAIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPAIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPAIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPAIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPAIOI)
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

void GPIO_IRQControl(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

void GPIO_IRQHandling(uint8_t pinNumber)
{

}