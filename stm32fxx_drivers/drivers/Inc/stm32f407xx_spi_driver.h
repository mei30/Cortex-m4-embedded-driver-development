/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 16, 2023
 *      Author: meisam
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stdint.h>

#include "stm32f407xx.h"

typedef struct 
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig; 
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHE;
    uint8_t SPI_SSM;
} SPI_ConfigDef_t;

#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE  0

#define SPI_BUS_CONFIG_FD                   1
#define SPI_BUS_CONFIG_HD                   2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY       3

#define SPI_SCLK_SPEED_DIV2         0 
#define SPI_SCLK_SPEED_DIV4         1        
#define SPI_SCLK_SPEED_DIV8         2         
#define SPI_SCLK_SPEED_DIV16        3        
#define SPI_SCLK_SPEED_DIV32        4        
#define SPI_SCLK_SPEED_DIV64        5       
#define SPI_SCLK_SPEED_DIV128       6        
#define SPI_SCLK_SPEED_DIV256       7        

#define SPI_DFF_8BITS       0
#define SPI_DFF_16BITS      1

#define SPI_CPOL_HIGHT      1
#define SPI_CPOL_LOW        0

#define SPI_CPHA_HIGHT      1
#define SPI_CPHA_LOW        0

#define SPI_SSM_EN    1
#define SPI_SSM_DI    0

#define SPI_TXE_FLAG        (1 << 1)
#define SPI_RXNE_FLAG       (1 << 0)
#define SPI_BUSY_FLAG       (1 << 7)

#define SPI_READY            0
#define SPI_BUSY_IN_RX       1
#define SPI_BUSY_IN_TX       2

typedef struct
{
    SPI_RegDef_t* pSPIx;
    SPI_ConfigDef_t SPIConfig;
    uint8_t* pTxBuffer;
    uint8_t* pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;
} SPI_Handle_t;

#define SPI_EVENT_TX_CMPLT      1
#define SPI_EVENT_RX_CMPLT      2
#define SPI_EVENT_OVR_ERR       3

void SPI_PreClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t len);

uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pRxBuffer, uint32_t len);

void SPI_IRQITControl(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPRIControl(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx);
void SPI_CloseTransmision(SPI_Handle_t* pSPIHandle);
void SPI_CloseReception(SPI_Handle_t* pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, uint8_t AppEv);

// TODO: In order to master mode work properly in Hardware mode add SSE API


#include "stm32f407xx.h"

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
