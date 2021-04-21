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
int  PushUserButton_Continue(void);
void delay(void);
void small_delay(void);
uint8_t CheckACKorNACK(SPI_Reg_t*);

#define COMMAND_LED_CTRL	0x50 // D# pin, On|Off, no return
#define COMMAND_SENSOR_READ 0x51 // A# pin, returns 1 byte result representing an 8-bit voltage value
#define COMMAND_LED_READ	0x52 // D# pin, returns 1 byte result representing LED On|Off
#define COMMAND_PRINT		0x53 // Len, Message, no return
#define COMMAND_ID_READ		0x54 // None, returns 10 bytes of Arduino board ID string
#define ACK					0xF5
#define NACK				0xA5

void Cmd_LED_CTRL(SPI_Reg_t *SPI_HW_ptr, uint8_t pinNum, uint8_t OnOrOff) {

	uint8_t cmd = COMMAND_LED_CTRL, cmdResult;
	uint8_t dummyRead = 0xff;

	SPI_Send(SPI_HW_ptr, &cmd, 1); // Send a cmd, then send 2 parameters (1 byte each)
	SPI_Receive(SPI_HW_ptr, &dummyRead, 1);
	cmdResult = CheckACKorNACK(SPI_HW_ptr);
	SPI_Send(SPI_HW_ptr, &pinNum, 1);
	SPI_Send(SPI_HW_ptr, &OnOrOff, 1);
}
 uint8_t Cmd_SENSOR_READ(SPI_Reg_t *SPI_HW_ptr, uint8_t ApinNum) {

	uint8_t cmd = COMMAND_SENSOR_READ, cmdResult;
	uint8_t *pSensorValue = 0;

	SPI_Send(SPI_HW_ptr, &cmd, 1);
	cmdResult = CheckACKorNACK(SPI_HW_ptr);
	SPI_Send(SPI_HW_ptr, &ApinNum, 1);
	SPI_Receive(SPI_HW_ptr, pSensorValue, 1);

	 return *pSensorValue;
}
 uint8_t Cmd_LED_READ(SPI_Reg_t *SPI_HW_ptr, uint8_t DpinNum) {

	 uint8_t cmd = COMMAND_LED_READ, cmdResult;
	 uint8_t *pLEDValue = malloc(1), dummyByte = 0xff;

	 SPI_Send(SPI_HW_ptr, &cmd, 1);
	 cmdResult = CheckACKorNACK(SPI_HW_ptr);
	 SPI_Send(SPI_HW_ptr, &DpinNum, 1);
	 SPI_Receive(SPI_HW_ptr, pLEDValue, 1); // This is a dummy
	 SPI_Send(SPI_HW_ptr, &dummyByte, 1);
	 SPI_Receive(SPI_HW_ptr, pLEDValue, 1);

	 return *pLEDValue;
}
 void Cmd_PRINT(SPI_Reg_t *SPI_HW_ptr, uint8_t len, char *message) {

	 uint8_t cmd = COMMAND_PRINT, cmdResult;

	 SPI_Send(SPI_HW_ptr, &cmd, 1);
	 cmdResult = CheckACKorNACK(SPI_HW_ptr);
	 SPI_Send(SPI_HW_ptr, &len, 1);
	 SPI_Send(SPI_HW_ptr, (uint8_t*)message, sizeof(*message));
 }
 char* Cmd_ID_READ(SPI_Reg_t *SPI_HW_ptr) {

	 uint8_t cmd = COMMAND_ID_READ, cmdResult;
	 char* boardID = malloc(10);

	 SPI_Send(SPI_HW_ptr, &cmd, 1);
	 cmdResult = CheckACKorNACK(SPI_HW_ptr);
	 SPI_Receive(SPI_HW_ptr, (uint8_t*)boardID, 10);

	 return boardID;
 }
 uint8_t CheckACKorNACK(SPI_Reg_t *SPI_HW_ptr) {

	 uint8_t RxBuffer[1] = {0xff}, dummyRead;
	 uint8_t dummyByte = 0xff;

	 SPI_Receive(SPI_HW_ptr, &dummyRead, 1);
	 SPI_Send(SPI_HW_ptr, &dummyByte, 1);
	SPI_Receive(SPI_HW_ptr, RxBuffer, 1); // Read for ACK or NACK and ignore
	if (RxBuffer[0] == ACK) {
		// Do something based on the command?
		return ACK;
	} else if (RxBuffer[0] == NACK) {
		// However we display an error message
		return NACK;
	}
	return NACK; // If we got garbage, return NACK
 }

int main(void) {

	/*
	 * STM32 will receive data from Arduino with SPI communication
	 * STM32 is Master, Arduino Slave.
	 * Data flows both directions? (or slave to master only?)
	 * DFF = 0 (8-bit), HW SS management, SCLK = 2MHz (pclk = 16MHz)
	 * Master will send # of bytes to be transferred before actual data.
	 * 1. Create the SPI handle for SPI2
	 * 3. Setup SPI2 pins, all are on Alt Func 5
	 * 	a. SPI2-SCK: 	PB13
	 * 	b. SPI2-MISO: 	PB14
	 * 	c. SPI2-MOSI:	PB15
	 * 	d. SPI2-NSS:	PB12
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
	SPI2_handle.SPI_DeviceConfig.SPI_ClkSpeed = SPI_PCLK_SPD_DIV16;
	SPI2_handle.SPI_DeviceConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_handle.SPI_DeviceConfig.SPI_SSM = SPI_SSM_HW;
	SPI2_handle.SPI_DeviceConfig.SPI_SSOE = 1;
	SPI2_handle.SPI_DeviceConfig.SPI_FRF = SPI_FRF_MOTO; // ignored if SSM = SPI_SSM_SW = 1
	// Doing these for initialization to defaults
	SPI2_handle.SPI_DeviceConfig.SPI_TXEIE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_RXNEIE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_ERRIE = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_TXDMAEN = 0;
	SPI2_handle.SPI_DeviceConfig.SPI_RXDMAEN = 0;

	SPI_Init(&SPI2_handle);

	uint8_t RxBuffer[256], LEDValue = 99;


	while (1) {
		PushUserButton_Continue();
		delay();
		SPI_Control(SPI2_handle.pSPIx, enable);

		/*
		 * Manual, step by step code
		 */
		uint8_t cmd, dummyRead = 0xff, dummyWrite = 0xff;
		uint8_t ackORnack, args[2], LEDValue, SensorValue;
/**/
		cmd = COMMAND_LED_CTRL; //======================================================
		args[0] = 9; // Pin #
		args[1] = 1; // On
		SPI_Send(SPI2_handle.pSPIx, &cmd, 1);					// 1. >> Send Command
		SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);			// 2. << Receive dummy
		small_delay();
		SPI_Send(SPI2_handle.pSPIx, &dummyWrite, 1);			// 3. >> Send dummy
		SPI_Receive(SPI2_handle.pSPIx, &ackORnack, 1);			// 4. << Receive Ack/Nack
		if (ackORnack == ACK) {
			SPI_Send(SPI2_handle.pSPIx, args, 2);				// 5. >> Send 1st and 2nd arg bytes
			SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);
			//SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);
			//printf("COMMAND_LED_CTRL Executed\n");
			//SPI_Send(SPI2_handle.pSPIx, &dummyWrite, 1);
		}

		//while (SPI_GetFlagStatus(SPI2_handle.pSPIx, SPI_SR_BSY_FLG)) {}
		//SPI_Control(SPI2_handle.pSPIx, disable);
		//PushUserButton_Continue();
		//delay();
		//SPI_Control(SPI2_handle.pSPIx, enable); //------------------------------------
/*
		cmd = COMMAND_SENSOR_READ; //======================================================
		args[0] = 0; //Analog Pin #
		SPI_Send(SPI2_handle.pSPIx, &cmd, 1);					// 7. >> Send Command
		SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);			// 8. << Receive dummy
		small_delay();  // Needed for the analog read
		SPI_Send(SPI2_handle.pSPIx, &dummyWrite, 1);			// 9. >> Send dummy
		SPI_Receive(SPI2_handle.pSPIx, &ackORnack, 1);			//10. << Receive Ack/Nack
		if (ackORnack == ACK) {
			SPI_Send(SPI2_handle.pSPIx, &args[0], 1);			//11. >> Send 1st arg byte
			SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);		//12. << Receive dummy
			small_delay();
			SPI_Send(SPI2_handle.pSPIx, &dummyWrite, 1);		//13. >> Send dummy
			SPI_Receive(SPI2_handle.pSPIx, &SensorValue, 1);		//14. << Receive LED value
		}

		//while (SPI_GetFlagStatus(SPI2_handle.pSPIx, SPI_SR_BSY_FLG)) {}
		//SPI_Control(SPI2_handle.pSPIx, disable);
		//PushUserButton_Continue();
		//delay();
		//SPI_Control(SPI2_handle.pSPIx, enable); //------------------------------------
*/

/*
		cmd = COMMAND_LED_READ; //======================================================
		args[0] = 9; //Pin #
		SPI_Send(SPI2_handle.pSPIx, &cmd, 1);					// 7. >> Send Command
		//SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);			// 8. << Receive dummy
		small_delay();
		SPI_Send(SPI2_handle.pSPIx, &dummyWrite, 1);			// 9. >> Send dummy
		SPI_Receive(SPI2_handle.pSPIx, &ackORnack, 1);			//10. << Receive Ack/Nack
		if (ackORnack == ACK) {
			SPI_Send(SPI2_handle.pSPIx, &args[0], 1);			//11. >> Send 1st arg byte
			SPI_Receive(SPI2_handle.pSPIx, &dummyRead, 1);		//12. << Receive dummy
			small_delay();
			SPI_Send(SPI2_handle.pSPIx, &dummyWrite, 1);		//13. >> Send dummy
			SPI_Receive(SPI2_handle.pSPIx, &LEDValue, 1);		//14. << Receive LED value
		}
		//Cmd_LED_CTRL(SPI2_handle.pSPIx, 9, 0);
		//LEDValue = Cmd_LED_READ(SPI2_handle.pSPIx, 9);
		//Cmd_PRINT(SPI2_handle.pSPIx, sizeof("Say something"), "Say something");
*/

		/*
		// Send the length of data (in bytes) to the slave
		uint8_t *lenBuffer;
		SPI_Receive(SPI2_handle.pSPIx, lenBuffer, 1);
		uint8_t *pRxBuffer;
		pRxBuffer = malloc(*lenBuffer);
		SPI_Receive(SPI2_handle.pSPIx, pRxBuffer, *lenBuffer);
*/
		// While SPI2 is busy, loop, else disable SPI2
		while (SPI_GetFlagStatus(SPI2_handle.pSPIx, SPI_SR_BSY_FLG)) {}

		SPI_Control(SPI2_handle.pSPIx, disable);
		while (0);

	}

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
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GPIOB_SPI_handle);
	// SPI2 MOSI
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIOB_SPI_handle);
	// SPI2 NSS
	GPIOB_SPI_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GPIOB_SPI_handle);

}
int PushUserButton_Continue(void)
{
	// toggle_mode = 0, LED on when the button is pushed, off when it's not
	// toggle_mode = 1, LED toggles when the button is pushed and released

	// Loop until the blue user button is pushed (connected to PA0). PA0 will see VDD when the
	// button is pushed. So if we're set to pull-up

	// With Vdd on the button, open collector will turn the light on when the driver is activated.
	// With push-pull, when the driver is on, the LED will be off.

	GPIO_Handle_t GPIOA_handle;
	GPIOA_handle.pGPIOx = GPIOA_HW;

	GPIOA_CLK_EN();

	GPIOA_handle.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIOA_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOA_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE; // since ext pull-down

	GPIO_Init(&GPIOA_handle);

	while(1) {
		// Look for the button push, return when we see it.
		if (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 0)== 1) {
			GPIOA_CLK_DIS();

			return 0;
		}
	}

	return 1; //We shouldn't get here
}
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}
void small_delay(void)
{
	for(uint32_t i = 0 ; i < 100 ; i ++);
}
