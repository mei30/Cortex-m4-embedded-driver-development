#include "stm32f407xx_i2c_driver.h"


uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[8] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteSlaveAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteSlaveAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t* pI2Cx);

void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
    pI2Cx->CR1 |= (1 << 8);
}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
    pI2Cx->CR1 |= (1 << 9);
}

static void I2C_ExecuteSlaveAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1);
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteSlaveAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr |= 1 << 0;
    pI2Cx->DR = SlaveAddr;
}

void I2C_ClearAddrFlag(I2C_RegDef_t* pI2Cx)
{
    uint32_t dummyread = pI2Cx->SR1;
    dummyread = pI2Cx->SR2;
    (void)dummyread;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 0);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname)
{
    if (pI2Cx->SR1 & flagname)
        return FLAG_SET;
    return FLAG_RESET;
}


void I2C_PreClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
            I2C1_PCLOCK_EN();
        else if (pI2Cx == I2C2)
            I2C2_PCLOCK_EN();
        else if (pI2Cx == I2C3)
            I2C3_PCLOCK_EN();
    } else
    {
        if (pI2Cx == I2C1)
            I2C1_PCLOCK_DI();
        else if (pI2Cx == I2C2)
            I2C2_PCLOCK_DI();
        else if (pI2Cx == I2C3)
            I2C3_PCLOCK_DI();
    }
}

uint32_t Rcc_GetPLLOutputClock()
{
    // TODO: calculate PLL clock later
    return 0;
}

uint32_t RCC_PCLK1Value(void)
{
    uint32_t pclk1, SystemClk;

    uint8_t clksrc, ahbp, temp, apb1p;

    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if (clksrc == 0)
    {
        SystemClk = 16000000;
    }
    else if (clksrc == 1)
    {
        SystemClk = 8000000;
    }
    else if (clksrc == 2)
    {
        SystemClk = Rcc_GetPLLOutputClock();
    }

    temp = ((RCC->CFGR >> 4) & 0xf);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else {
        ahbp = AHB_PreScaler[temp - 8];
    }

    temp = ((RCC->CFGR >> 10) & 0x7);

    if (temp < 4)
    {
        apb1p = 1;
    }
    else {
        apb1p = APB1_PreScaler[temp - 4];
    }

    pclk1 = (SystemClk) / apb1p * ahbp;

    return pclk1;
}

void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	I2C_PreClockControl(pI2CHandle->pI2Cx, ENABLE);

    uint32_t tempreg = 0;

    tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    tempreg = 0;
    tempreg |= RCC_PCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    uint16_t ccr_value = 0;

    tempreg = 0;
    
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        ccr_value = RCC_PCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        tempreg |= (ccr_value & 0xFFF);
    } else {
        tempreg |= (1 << 15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = RCC_PCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        } else {
            ccr_value = RCC_PCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }

        tempreg |= (ccr_value & 0xFFF);
    }

    pI2CHandle->pI2Cx->CCR = tempreg;

    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        tempreg = (Rcc_GetPLLOutputClock() / 1000000) + 1;
    } else {

        tempreg = ((Rcc_GetPLLOutputClock() * 300) / 1000000000) + 1;
    }

    pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

    
}

void I2C_DeInit(I2C_RegDef_t* pI2Cx)
{
    if (pI2Cx == I2C1)
        I2C1_REG_RESET();
    else if (pI2Cx == I2C2)
        I2C2_REG_RESET();
    else if (pI2Cx == I2C3)
        I2C3_REG_RESET();
}

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint8_t len, uint8_t SlaveAddr)
{
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // checking SB flag
    while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

    I2C_ExecuteSlaveAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_DeviceAddress);

    // Checking addr flag
    while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

    I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

    while (len > 0)
    {
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));

        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        ++pTxBuffer;
        --len;
    }

    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));

    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));

    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)   );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG) );

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << 10);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << 10);
	}
}
