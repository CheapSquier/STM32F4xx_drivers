/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Mar 26, 2021
 *      Author: wadeb
 */

#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_

#include "STM32F407xx.h"

/*
 * Configuration structure for SPI pins
 */
typedef struct
{
	uint8_t	SPI_MstrSlvSel;			/*!<Possible values from @SPI_DEVICE_MODES>*/
	uint8_t	SPI_BusConfig;			/*!<Possible values from @SPI_BUS_MODES>*/
	uint8_t	SPI_ClkSpeed;			/*!<Possible values from @SPI_OUTPUT_TYPE>*/
	uint8_t	SPI_CPHA;				/*!<Possible values from @SPI_SPI_SPI_CPHA>*/
	uint8_t	SPI_CPOL;				/*!<Possible values from @SPI_SPI_SPI_CPOL>*/
	uint8_t	SPI_DFF;				/*!<Possible values from @SPI_DFF>*/
	uint8_t SPI_SSM;				/*!<Possible values from @SPI_SW_SLAVE_MGMT>*/
	uint8_t	SPI_TXEIE;				/*!<Possible values from @SPI_TX_INTERRUPT>*/
	uint8_t	SPI_RXNEIE;				/*!<Possible values from @SPI_RX_INTERRUPT>*/
	uint8_t	SPI_ERRIE;				/*!<Possible values from @SPI_ERR_INTERRUPT>*/
	uint8_t	SPI_FRF;				/*!<Possible values from @SPI_FRAME_FMT>*/
	uint8_t	SPI_SSOE;				/*!<Possible values from @SPI_SS_OUTPUT_EN>*/
	uint8_t	SPI_TXDMAEN;			/*!<Possible values from @SPI_TX_DMA_EN>*/
	uint8_t	SPI_RXDMAEN;			/*!<Possible values from @SPI_RX_DMA_EN>*/
}SPI_DeviceConfig_t;
/*
 * Handle structure for a SPI pin
 */
typedef struct
{
	SPI_Reg_t *pSPIx; /*!< Holds base addr of this pin's SPI port >*/
	SPI_DeviceConfig_t SPI_DeviceConfig;
}SPI_Handle_t;

/*
 * @SPI_DEVICE_MODES
 * SPI Modes, master or slave
 */
#define SPI_MODE_MASTER 			0
#define SPI_MODE_SLAVE	 			1
/*
 * @SPI_BUS_MODES
 * SPI Bus mode, Full/Half Duplex, Simplex
 */
//#define SPI_BUS_SIMPLEX_TXONLY	TX/RX is from Master perspective.
//									If TX, set BIDIMODE = 0 and drop
//									the MISO line (not needed)
//									If RX, set RXONLY = 1
#define SPI_BUS_SIMPLEX_RXONLY		1
#define SPI_BUS_DUPLEX_HALF			2
#define SPI_BUS_DUPLEX_FULL			3
/*
 * @SPI_DFF
 * SPI 8 or 16 bit data
 */
#define SPI_DFF_8BIT				8
#define SPI_DFF_16BIT				16
/*
 * @SPI_CPOL
 * SPI Normal or Inverted polarity (active high or active low, idle at low or idle at high)
 */
#define SPI_CPOL_NORMAL_ACTH		0
#define SPI_CPOL_INVERSE_ACTL		1
/*
 * @SPI_CPHA
 * SPI data latches on 1st or 2nd edge of clock
 */
#define SPI_CPHA_1ST_EDGE			1
#define SPI_CPHA_2ND_EDGE			2
/*
 * @SPI_PCLK_SPD
 * SPI PCLK divider value
 */
#define SPI_PCLK_SPD_DIV2			0
#define SPI_PCLK_SPD_DIV4			1
#define SPI_PCLK_SPD_DIV8			2
#define SPI_PCLK_SPD_DIV16			3
#define SPI_PCLK_SPD_DIV32			4
#define SPI_PCLK_SPD_DIV64			5
#define SPI_PCLK_SPD_DIV128			6
#define SPI_PCLK_SPD_DIV256			7
/*
* @SPI_SW_SLAVE_MGMT
* SPI Normal or Inverted polarity (active high or active low, idle at low or idle at high)
*/
#define SPI_SSM_HW		0
#define SPI_SSM_SW		1
/*
 * @SPI_TX_INTERRUPT
 */
#define SPI_TXINT_MASKED	0
#define SPI_TXINT_EN		1
/*
 * @SPI_RX_INTERRUPT
 */
#define SPI_RXINT_MASKED	0
#define SPI_RXINT_EN		1
/*
 * @SPI_ERR_INTERRUPT
 */
#define SPI_ERRINT_MASKED	0
#define SPI_ERRINT_EN		1
/*
 * @SPI_FRAME_FMT
 */
#define SPI_FRF_MOTO	0
#define SPI_FRF_TI		1
/*
 * @SPI_SS_OUTPUT_EN
 */
#define SPI_SSOE_DISABLE	0
#define SPI_SSOE_EN			1
/*
 * @SPI_TX_DMA_EN
 */
#define SPI_TXDMA_DISABLE	0
#define SPI_TXDMA_EN		1
/*
 * @SPI_RX_DMA_EN
 */
#define SPI_RXDMA_DISABLE	0
#define SPI_RXDMA_EN		1

/*
 * SPI related flag definitions
 */
/*
#define SPI_SR_FRE_ERR		8
#define 		7
#define SPI_SR_OVR_FLG		6
#define SPI_SR_MODF_FLT		5
#define SPI_SR_CRCERR_FLG	4
#define SPI_SR_UDR_FLG		3
#define SPI_SR_CHSIDE		2 */
#define SPI_SR_TXE_FLG		(1 << SPI_SR_TXE)
#define SPI_SR_RXNE_FLG		(1 << SPI_SR_RXNE)
#define SPI_SR_BSY_FLG		(1 << SPI_SR_BSY)

/***********************************************************************
 * 					APIs supported by this SPI driver
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Control(SPI_Reg_t *pSPIx, EnableDisable_e EnaOrDis);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);
uint8_t SPI_GetFlagStatus(SPI_Reg_t *pSPIx, uint32_t flagName);
// Why does Len need to be uint32_t? Standard practice?
void SPI_Send(SPI_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_Receive(SPI_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// IRQ Config and ISR Handling
// Note: Could have IRQ grouping in IRQConfig
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnaOrDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
//SPI1, SPI2, or SPI3
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif /* STM32F407XX_SPI_DRIVER_H_ */
