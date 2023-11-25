/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 16, 2023
 *      Author: meisam
 */

#include "stm32f407xx_spi_driver.h"

void SPI_PreClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLOCK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLOCK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLOCK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLOCK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLOCK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLOCK_DI();
        }

    }
}

void SPI_Init(SPI_Handle_t pSPIHandle)
{
	SPI_PreClockControl(pSPIHandle.pSPIx, ENABLE);

    uint32_t tempreg = 0;

    tempreg |= (pSPIHandle.SPIConfig.SPI_DeviceMode << 2);

    if (pSPIHandle.SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        tempreg &= ~(1 << 15);
    } else if (pSPIHandle.SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        tempreg |= (1 << 15);
    } else if (pSPIHandle.SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        tempreg &= ~(1 << 15);
        tempreg |= (1 << 10);
    }

    tempreg |= (pSPIHandle.SPIConfig.SPI_SclkSpeed << 3);
    tempreg |= (pSPIHandle.SPIConfig.SPI_DFF << 11);
    tempreg |= (pSPIHandle.SPIConfig.SPI_CPOL << 1);
    tempreg |= (pSPIHandle.SPIConfig.SPI_CPHE << 0);
    tempreg |= (pSPIHandle.SPIConfig.SPI_SSM << 9);

    pSPIHandle.pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t* pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI1)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI1)
    {
        SPI3_REG_RESET();
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flagname)
{
    if (pSPIx->SR & flagname)
        return FLAG_SET;
    return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len)
{
    while (len > 0)
    {
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        if (pSPIx->CR1 & (1 << 11))
        {
            pSPIx->DR = *((uint16_t *)pTxBuffer);
            len -= 2;
            (uint16_t *)pTxBuffer++;
        } else
        {
            pSPIx->DR = *pTxBuffer;
            len--;
            pTxBuffer++;
        }
        
    }
    
}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len)
{

}

void SPI_IRQITControl(uint8_t IRQNumber, uint8_t EnorDi)
{

}

void SPI_IRQPRIControl(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t* pSPIHandle)
{

}

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << 6);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << 6);
    }
}

void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << 8);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << 8);
    }
}
