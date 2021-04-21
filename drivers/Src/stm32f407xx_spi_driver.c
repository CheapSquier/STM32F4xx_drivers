/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Mar 26, 2021
 *      Author: wadeb
 */
#include "stm32f407xx_spi_driver.h"
/***********************************************************************
 * 					APIs supported by this SPI driver
 *
 */
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes a SPI port, given a declared SPI Handle struct,
 * 						which consists of a SPI Register struct and a SPI Device struct, both defined in
 * 						the main STM32 .h file
 *
 * @param[in]         - SPI Handle struct
 *
 * @return            -  none
 *
 * @Note              -  Things we're not setting: LSBFIRST, CRC (any CRC control)
 */

void SPI_Init(SPI_Handle_t *pSPIHandle) {
	// Master or Slave  -------------------------------------------------
	if (pSPIHandle->SPI_DeviceConfig.SPI_MstrSlvSel == SPI_MODE_MASTER) {
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_MSTR); // Set the MSTR bit
	} else {
		// Setup for slave
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_MSTR); // Clear the MSTR bit
	}
	// Bus configuration/Data transmission  --------------------------------
	if (pSPIHandle->SPI_DeviceConfig.SPI_BusConfig == SPI_BUS_DUPLEX_FULL) {
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_RXONLY); // Sets full duplex (RXONLY = 0)
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_BIDIMODE); // Sets 2-line uni communication
	} else if (pSPIHandle->SPI_DeviceConfig.SPI_BusConfig == SPI_BUS_DUPLEX_HALF) {
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_RXONLY); // Sets full duplex (RXONLY = 0)
		pSPIHandle->pSPIx->CR1 |=  (0x1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_DeviceConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY) {
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR1 |=  (0x1 << SPI_CR1_RXONLY); // Output disabled (RXONLY = 1)
	}
	// Data Frame Format  -------------------------------------------------------
	if (pSPIHandle->SPI_DeviceConfig.SPI_DFF == SPI_DFF_8BIT) {
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_DFF); // Clear the DFF bit
	} else {
		// Setup for 16 bit
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_DFF); // Set the DFF bit
	}

	// SCK Polarity   -----------------------------------------------------------
	if (pSPIHandle->SPI_DeviceConfig.SPI_CPOL == SPI_CPOL_NORMAL_ACTH) {
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_CPOL); // Clear the CPOL bit
	} else {
		// Set CPOL inverted
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_CPOL); // Set the CPOL bit
	}

	// SCK Phase (1st or 2nd edge data capture)  --------------------------------
	if (pSPIHandle->SPI_DeviceConfig.SPI_CPHA == SPI_CPHA_1ST_EDGE) {
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_CPHA); // Clear the CPHA bit
	} else {
		// Set CPHA for 2nd edge
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_CPHA); // Set the CPHA bit
	}

	// Slave Select Management (SW or HW)  -----SSM = 1 is SW, 0 is HW ----------
	if (pSPIHandle->SPI_DeviceConfig.SPI_SSM == SPI_SSM_SW) {
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_SSM); // Set the SSM bit
		if (pSPIHandle->SPI_DeviceConfig.SPI_FRF == SPI_FRF_MOTO) {
			pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_SSI); // Clear the SSI bit
		}
	} else {
		// Setup for HW management of SS
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_SSM); // Clear the SSM bit
	}

	// Set the bus speed (clear then set) ---------------------------------------
	pSPIHandle->pSPIx->CR1 &= ~(0x7 << SPI_CR1_BR2_0); // Clear the Baud Rate bits
	// SPI_ClkSpeed should be set with the @SPI_PCLK_SPD macro, so the same bit values can
	// be used to set the BR[2:0] bits
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_DeviceConfig.SPI_ClkSpeed << SPI_CR1_BR2_0);

	// SPI peripheral enable ----- NO, better to wait until it's all configured, then enable ------
	//

	// 1. Next time, set all macros based on their default and actual values rather than
	//    using values like 8 or 16 (0 or 1 would be better)
	// 2. Store all the config values into a temp register first, then do a single write to
	//    that temp register.

	uint32_t tempReg = 0x00U;

	if (pSPIHandle->SPI_DeviceConfig.SPI_TXEIE == SPI_TXINT_EN) {
		tempReg |= (1 << SPI_CR2_TXEIE);
	}
	if (pSPIHandle->SPI_DeviceConfig.SPI_RXNEIE  == SPI_RXINT_EN) {
		tempReg |= (1 << SPI_CR2_RXNEIE);
	}
	if (pSPIHandle->SPI_DeviceConfig.SPI_ERRIE == SPI_ERRINT_EN) {
		tempReg |= (1 << SPI_CR2_EREIE);
	}
	if (pSPIHandle->SPI_DeviceConfig.SPI_FRF == SPI_FRF_TI) {
		tempReg |= (1 << SPI_CR2_FRF);
	}
	if (pSPIHandle->SPI_DeviceConfig.SPI_SSOE == SPI_SSOE_EN) {
		tempReg |= (1 << SPI_CR2_SSOE);
	}
	if (pSPIHandle->SPI_DeviceConfig.SPI_TXDMAEN == SPI_TXDMA_EN) {
		tempReg |= (1 << SPI_CR2_TXDMAEN);
	}
	if (pSPIHandle->SPI_DeviceConfig.SPI_RXDMAEN == SPI_TXDMA_EN) {
		tempReg |= (1 << SPI_CR2_RXDMAEN);
	}
	pSPIHandle->pSPIx->CR2 = tempReg;


}

/*********************************************************************
 * @fn      		  - SPI_Control
 *
 * @brief             - Enables or disables the given SPI port
 *
 * @param[in]         - SPI register pointer
 * @param[in]         - Enable or Disable (typedef'd)
 *
 * @return            -  none
 *
 * @Note              -  T
 */
void SPI_Control(SPI_Reg_t *pSPIx, EnableDisable_e EnaOrDis) {

	if (EnaOrDis == enable) {
		pSPIx->CR1 |= (0x1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(0x1 << SPI_CR1_SPE);
	}
}
void SPI_DeInit(SPI_Handle_t *pSPIHandle){

	if (pSPIHandle->pSPIx == SPI1_HW) {
		SPI1_REG_RESET();
	} else if (pSPIHandle->pSPIx == SPI2_HW) {
		SPI2_REG_RESET();
	} else if (pSPIHandle->pSPIx == SPI3_HW) {
		SPI3_REG_RESET();
	}
}
uint8_t SPI_GetFlagStatus(SPI_Reg_t *pSPIx, uint32_t flagName){

	if (pSPIx->SR & flagName) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}
/*********************************************************************
 * @fn      		  - SPI_Send
 *
 * @brief             - Could be 8 or 16 bits of data to send, so pass that as a pointer and a
 * 						Len
 *
 * @param[in]         - SPI Handle Struct
 * @param[in]         - Pointer that will reference the Tx/Rx buffer
 * @param[in]         - Data frame length
 *
 * @return            -  none
 *
 * @Note              -  Blocking call
 */
// Why does Len need to be uint32_t? Standard practice?
void SPI_Send(SPI_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while (Len > 0) {
		// Loop until TX buffer is empty (TXE == 1 means empty)
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLG) == FLAG_RESET);

		// write data.
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else {
			//8 bit
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}



void SPI_Receive(SPI_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

	while (Len > 0) {
		// If RX buffer is empty, don't read, just loop
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE_FLG) == FLAG_RESET);

		// Read data
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit
			*(uint16_t*)pRxBuffer = pSPIx->DR;
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;
		} else {
			//8 bit
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}

// IRQ Config and ISR Handling
/*********************************************************************
 * @fn      		  - SPI_IRQConfig
 *
 * @brief             - Sets up an IRQ with a priority value and leaves it in an enabled or
 * 						disabled state.
 *
 * @param[in]         - What number we want to assign this IRQ.
 * @param[in]		  - ENABLE or DISABLE macro (or 0 or 1)
 *
 * @return            -  none
 *
 * @Note              -  Could have IRQ grouping in IRQConfig
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnaOrDis);

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - Sets IRQ# Priority value
 *
 * @param[in]         - IRQ Number
 * @param[in]         - IRQ Priority value (0-15)
 *
 * @return            -  none
 *
 * @Note              -  Even though there are 8 bits per IRQ# in
 * 						 the register, only the UPPER 4 bits are used
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Specifies what to do with IRQs for this pin
 *
 * @param[in]         - SPI Handle for SPI1, SPI2, SPI3
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
