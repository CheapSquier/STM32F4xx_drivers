/*
 * 006sti_testing.c
 *
 *  Created on: Mar 31, 2021
 *      Author: wadeb
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "STM32F407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

void SPI2_GPIO_Config(void);

int main(void) {

	/*
	 * Hello world (master only)
	 * 1. Create the SPI handle for SPI2
	 * 3. Setup SPI2 pins, all are on Alt Func 5
	 * 	a. SPI2-SCK: 	PB13
	 * 	b. SPI2-MISO: 	PB14
	 * 	c. SPI2-MOSI:	PB15
	 * 	d. SPI2-NSS:	PB12 (skip, we'll assume SW SSM)
	 * 2. SPI_Init(the SPI handle)
	 * 3. Store the "Hello world" string at a ptr in free memory space
	 * 4. SPI_Send(the SPI handle, pointer to the string, Len"
	 */

	SPI_Handle_t SPI2_handle;

	SPI2_GPIO_Config(); // This function does GPIO CLK EN as well

	SPI2_CLK_EN();

	SPI2_handle.pSPIx = SPI2_HW;
	SPI2_handle.SPI_DeviceConfig.SPI_MstrSlvSel = SPI_MODE_MASTER;
	SPI2_handle.SPI_DeviceConfig.SPI_BusConfig = SPI_BUS_DUPLEX_FULL;
	SPI2_handle.SPI_DeviceConfig.SPI_CPOL = SPI_CPOL_NORMAL_ACTH;
	SPI2_handle.SPI_DeviceConfig.SPI_CPHA = SPI_CPHA_1ST_EDGE;
	SPI2_handle.SPI_DeviceConfig.SPI_ClkSpeed = SPI_PCLK_SPD_DIV2;
	SPI2_handle.SPI_DeviceConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_handle.SPI_DeviceConfig.SPI_SSM = SPI_SSM_SW;
	SPI2_handle.SPI_DeviceConfig.SPI_FRF = SPI_FRF_MOTO; // ignored if SSM = SPI_SSM_SW = 1
	// Doing these for initialization to defaults
	SPI2_handle.SPI_DeviceConfig.SPI_TXEIE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_RXNEIE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_ERRIE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_SSOE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_TXDMAEN = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_RXDMAEN = 0;

	SPI_Init(&SPI2_handle);
	SPI_Control(SPI2_handle.pSPIx, enable);

	char stringBuffer[] = "Hello World!";

	SPI_Send(SPI2_handle.pSPIx, (uint8_t*)stringBuffer, sizeof(stringBuffer));
	// either code should work
	//SPI_Send(SPI2_HW, (uint8_t*)stringBuffer, strlen("Hello World!"));

	while(1);  // Infinite loop

	SPI_Control(SPI2_handle.pSPIx, disable);

	return 0;
}
void SPI2_GPIO_Config(void) {

	GPIO_Handle_t GPIOB_SPI_handle;

	GPIOB_CLK_EN();

	GPIOB_SPI_handle.pGPIOx = GPIOB_HW;

	//All the SPI2 pins will be setup for Alt Func, I/O, same config, then
	//we set the pin number and initialize 1 pin at a time. The same registers
	//are programmed multiple times, but with different bit fields each time.
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE;
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PSH_PULL;
	// SPI2 SCK
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&GPIOB_SPI_handle);
	// SPI2 MISO
	//GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&GPIOB_SPI_handle);
	// SPI2 MOSI
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIOB_SPI_handle);
	// SPI2 NSS
	//GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&GPIOB_SPI_handle);

}
