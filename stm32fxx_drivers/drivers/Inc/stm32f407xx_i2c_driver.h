/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Nov 16, 2023
 *      Author: meisam
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
    uint32_t I2C_SCLSpeed;
    uint32_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct
{
    I2C_RegDef_t* pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;

#define I2C_SCL_SPEED_SM          100000
#define I2C_SCL_SPEED_FM4K        400000
#define I2C_SCL_SPEED_FM2K        200000

#define I2C_ACK_ENABLE      0
#define I2C_ACK_DISABLE     1

#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

#define I2C_TXE_FLAG        (1 << 7)
#define I2C_RXNE_FLAG       (1 << 6)
#define I2C_SB_FLAG         (1 << 0)
#define I2C_ADDR_FLAG       (1 << 1)
#define I2C_BTF_FLAG       (1 << 2)

void I2C_PreClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);

void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx);

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint8_t len, uint8_t SlaveAddr);

void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t AppEv);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)

#include "stm32f407xx.h"

#endif