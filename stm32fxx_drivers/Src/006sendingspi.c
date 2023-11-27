/*
 * PB14--> SPI2_MISO
 * PB15--> SPI2_MOSI
 * PB13--> SPI2_SCLK
 * PB12--> SPI2_NSS
 * ALT function mode: 5
 */

#include <string.h>

#include "stm32f407xx.h"

void SPI_GPIOInit()
{
	GPIO_Handle_t SPIPins;

	memset(&SPIPins, 0, sizeof(GPIO_Handle_t));

		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
		SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALFN;
		SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		// SCLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 3;
		GPIO_Init(&SPIPins);

		// NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
		GPIO_Init(&SPIPins);

		// MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 5;
		GPIO_Init(&SPIPins);

		// MISO
//		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
//		GPIO_Init(&SPIPins);
}

void SPI2_Init()
{
	SPI_Handle_t SPIHandle;

	//memset(&SPIHandle, 0, sizeof(SPI_Handle_t));

	SPIHandle.pSPIx = SPI3;
	SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPIHandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGHT;
	SPIHandle.SPIConfig.SPI_CPHE = SPI_CPHA_HIGHT;
	SPIHandle.SPIConfig.SPI_SSM = SPI_SSM_EN;


	SPI_Init(&SPIHandle);


}

int main()
{
	char data[] = "Hello World";

	SPI_GPIOInit();
	GPIOB->MODER &= ~(0x3 << (2 * 4));

	SPI2_Init();

	// NOTE: In SSM mode set SSI to hight to prevent from MODEF error
	SPI_SSIConfig(SPI3, ENABLE);

	// NOTE: this bit really enables SPI, for configuring SPI properly we must disable first or our configurations won't set
	SPI_PeripheralControl(SPI3, ENABLE);

	SPI_SendData(SPI3, (uint8_t *)data, strlen(data));

	SPI_PeripheralControl(SPI3, ENABLE);

	GPIO_DeInit(GPIOB);
	GPIO_PreClockControl(GPIOB, DISABLE);

	while(1);

	return 0;
}
