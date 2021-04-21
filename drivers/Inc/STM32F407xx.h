/*
 * STM32F407xx.h
 *
 *  Created on: Mar 4, 2021
 *      Author: wadeb
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stdint.h>

/*********************************************************************************
 * ARM Cortex M4 Processor Specific Register Addresses
 */
/* NVIC Interrupt Set (NVIC_ISE# */
#define NVIC_ISER0			((volatile uint32_t*) 0xE000E100U)
#define NVIC_ISER1			((volatile uint32_t*) 0xE000E104U)
#define NVIC_ISER2			((volatile uint32_t*) 0xE000E108U)
/* NVIC Interrupt Clear (NVIC_ICE# */
#define NVIC_ICER0			((volatile uint32_t*) 0xE000E180U)
#define NVIC_ICER1			((volatile uint32_t*) 0xE000E184U)
#define NVIC_ICER2			((volatile uint32_t*) 0xE000E188U)
/* NVIC Interrupt Priority (NVIC_IPR# */
#define NVIC_IPR_BASEADDR	((volatile uint32_t*) 0xE000E400U)
#define NUM_PR_BITS_USED	4
/* Memory Base Addresses */
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U // 112K, main SRAM
#define SRAM2_BASEADDR		0x2001C000U // 16K
#define ROM					0x1FFF0000U // System Memory
#define SRAM				SRAM1_BASEaddr

/* Bus Domain Base Addresses */
#define RCC_BASEADDR		0x40023800U	// Reset, Clock Control for multiple busses

#define GPIOK_BASEADDR		0x40022800U // AHB1
#define GPIOJ_BASEADDR		0x40022400U // AHB1
#define GPIOI_BASEADDR		0x40022000U // AHB1
#define GPIOH_BASEADDR		0x40021C00U // AHB1
#define GPIOG_BASEADDR		0x40021800U // AHB1
#define GPIOF_BASEADDR		0x40021400U // AHB1
#define GPIOE_BASEADDR		0x40021000U // AHB1
#define GPIOD_BASEADDR		0x40020C00U // AHB1
#define GPIOC_BASEADDR		0x40020800U // AHB1
#define GPIOB_BASEADDR		0x40020400U // AHB1
#define GPIOA_BASEADDR		0x40020000U // AHB1

#define CAN2_BASEADDR		0x40006800U // 		APB1
#define CAN1_BASEADDR		0x40006400U // 		APB1
#define I2C3_BASEADDR		0x40005C00U // 		APB1
#define I2C2_BASEADDR		0x40005800U // 		APB1
#define I2C1_BASEADDR		0x40005400U // 		APB1
#define UART5_BASEADDR		0x40005000U // 		APB1
#define UART4_BASEADDR		0x40004C00U // 		APB1
#define UART8_BASEADDR		0x40007C00U // 		APB1
#define UART7_BASEADDR		0x40007800U // 		APB1
#define USART3_BASEADDR		0x40004800U // 		APB1
#define USART2_BASEADDR		0x40004400U // 		APB1
#define USART6_BASEADDR		0x40011400U // 			APB2
#define USART1_BASEADDR		0x40011000U // 			APB2

#define EXTI_BASEADDR		0x40013C00U // 			APB2
#define SYSCFG_BASEADDR		0x40013800U // 			APB2
#define SPI1_BASEADDR		0x40013000U // 			APB2
#define I2S3ext_BASEADDR	0x40004000U	// 		APB1
#define SPI2_BASEADDR	    0x40003800U // 		APB1
#define SPI3_BASEADDR   	0x40003C00U // 		APB1

#define I2S3		    	(SPI3) 		// 		APB1
#define I2S2		    	(SPI2) 		// 		APB1
#define I2S2ext_BASEADDR	0x40003400U // 			APB2
#define I2S3ext_BASEADDR	0x40004000U	// 		APB1
#define SDIO_BASEADDR		0x40012C00U // 			APB2

#define DAC_BASEADDR		0x40007400U // 		APB1
#define PWR_BASEADDR		0x40007000U // 		APB1
#define ADC_BASEADDR		0x40012000U // APB2

//AHB Bus is higher speed, APB is lower speed

/************************    Peripheral Register Definitions    *************************/

typedef struct {
	volatile uint32_t	MODER;			// 0x00 IN, OUT, Alt. Func., Analog
	volatile uint32_t	OTYPER;			// 0x04 PushPull Output, Open-Drain Output
	volatile uint32_t	OSPEEDR;		// 0x08 Low, Med, High, Very High output speed
	volatile uint32_t	PUPDR;			// 0x0c PUPD off, Pull-Up (PU), Pull-Down (PD)
	volatile uint32_t	IDR;			// 0x10 Input Data register (read only). Data clred on read
	volatile uint32_t	ODR;			// 0x14 Output Data register, rw by word
	volatile uint32_t	BSRR;			// 0x18 Atomic bit output data state. write-only
										//      bits 0-15: 0=no action, 1=set bit/pin
										//	    bits 16-31: 0=no action, 1=reset bit/pin
	volatile uint32_t	LCKR;			// 0x1c Locks GPIO registers. Needs a specific WR sequence
	volatile uint32_t	AFRL;			// 0x20 Alt. Function control for pins 0-7
	volatile uint32_t	AFRH;			// 0x24 Alt. Function control for pins 8-15
} GPIO_Reg_t;

typedef struct {
	volatile uint32_t	CR1;		// 0x00 Setup BIDI/Tx/Rx modes, CRC enable, DFF, Master/Slave, enable, Baud rate, CPOL, CPHA
	volatile uint32_t	CR2;		// 0x04 Tx/Rx buffer empty int, err int, Moto/TI Frame fmt, SSOE, Tx/Rx DMA enable
	volatile uint32_t	SR;			// 0x08 Frame fmt err, busy flg, over/underrun flgs, mode fault, CRC err, ch side, Tx/Rx empty flgs
	volatile uint32_t	DR;			// 0x0c Data register for either Tx or Rx data. Same register does both MSByte=0x0 in 8-bit DFF
	volatile uint32_t	CRCPR;		// 0x10 CRC polynomial
	volatile uint32_t	RXCRCR;		// 0x14 CRC value
	volatile uint32_t	TXCRCR;		// 0x18 CRC value
	volatile uint32_t	I2SCFGR;	// 0x1c I2S vs SPI mode, I2S enable & cfg, PCM, I2S standard, CKPOL, Data Len, 16/32 bit audio
	volatile uint32_t	I2SPR;		// 0x20 I2S Master Clk en, prescaler control
} SPI_Reg_t;

						// Remember, for a struct definition to maintain the correct
						// addressing, we need to allocate space to the Reserved registers
						// as well as the used registers.
typedef struct {
	volatile uint32_t	CR;			// 0x00
	volatile uint32_t	PLLCFGR;			// 0x04
	volatile uint32_t	CFGR;		// 0x08
	volatile uint32_t	CIR;			// 0x0c
	volatile uint32_t	AHB1RSTR;			// 0x10
	volatile uint32_t	AHB2RSTR;			// 0x14
	volatile uint32_t	AHB3RSTR;			// 0x18
	uint32_t 			RESERVED0;			// 0x1c Reserved
	volatile uint32_t	APB1RSTR;			// 0x20
	volatile uint32_t	APB2RSTR;			// 0x24
	uint32_t 			RESERVED1[2];			// 0x28, 0x2c Reserved
	volatile uint32_t	AHB1ENR;		// 0x30
	volatile uint32_t	AHB2ENR;		// 0x34
	volatile uint32_t	AHB3ENR;		// 0x38
	uint32_t 			RESERVED3;			// 0x3c Reserved
	volatile uint32_t	APB1ENR;		// 0x40
	volatile uint32_t	APB2ENR;		// 0x44
	uint32_t 			RESERVED4[2];			// 0x48, 0x4c Reserved
	volatile uint32_t	AHB1LPENR;		// 0x50
	volatile uint32_t	AHB2LPENR;		// 0x54
	volatile uint32_t	AHB3LPENR;		// 0x58
	uint32_t 			RESERVED5;			// 0x5c Reserved
	volatile uint32_t	APB1LPENR;		// 0x60
	volatile uint32_t	APB2LPENR;		// 0x64
	uint32_t 			RESERVED6[2];			// 0x68 Reserved
	// Reserved 		----;			// 0x6c Reserved
	volatile uint32_t	BDCR;			// 0x70
	volatile uint32_t	CSR;			// 0x74
	uint32_t 			RESERVED7;			// 0x78, 7c Reserved
	volatile uint32_t	SSCGR;			// 0x70
	volatile uint32_t	PLLI2SCFGR;			// 0x74
} RCC_Reg_t;

typedef struct {
	volatile uint32_t	MEMRMP;			// 0x00
	volatile uint32_t	PMC;			// 0x04
	volatile uint32_t	EXTICR[4];		// 0x08-0x14
			 uint32_t	RESERVED1[2];	// 0x18
	volatile uint32_t	CMPCR;			// 0x20 Skipped 64 bits? Should have been 18
} SYSCFG_Reg_t;

typedef struct {
	volatile uint32_t	IMR;		// 0x00 (offset from EXTI_BASEADDR on APB2)
	volatile uint32_t	EMR;		// 0x04
	volatile uint32_t	RTSR;		// 0x08
	volatile uint32_t	FTSR;		// 0x0c
	volatile uint32_t	SWIER;		// 0x10
	volatile uint32_t	PR;		// 0x14
} EXTI_Reg_t;

#define GPIOA_HW			((GPIO_Reg_t*) GPIOA_BASEADDR)
#define GPIOB_HW			((GPIO_Reg_t*) GPIOB_BASEADDR)
#define GPIOC_HW			((GPIO_Reg_t*) GPIOC_BASEADDR)
#define GPIOD_HW			((GPIO_Reg_t*) GPIOD_BASEADDR)
#define GPIOE_HW			((GPIO_Reg_t*) GPIOE_BASEADDR)
#define GPIOF_HW			((GPIO_Reg_t*) GPIOF_BASEADDR)
#define GPIOG_HW			((GPIO_Reg_t*) GPIOG_BASEADDR)
#define GPIOH_HW			((GPIO_Reg_t*) GPIOH_BASEADDR)
#define GPIOI_HW			((GPIO_Reg_t*) GPIOI_BASEADDR)

#define SPI1_HW				((SPI_Reg_t*)  SPI1_BASEADDR)
#define SPI2_HW				((SPI_Reg_t*)  SPI2_BASEADDR)
#define SPI3_HW				((SPI_Reg_t*)  SPI3_BASEADDR)

#define RCC_HW				((RCC_Reg_t*) RCC_BASEADDR)
#define SYSCFG_HW			((SYSCFG_Reg_t*) SYSCFG_BASEADDR)
#define EXTI_HW				((EXTI_Reg_t*) EXTI_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()		(RCC_HW->AHB1ENR |= (1 << 8))
/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 23))
/*
 * Clock Enable Macros for SPI peripherals
 */
#define SPI1_CLK_EN()			(RCC_HW->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 15))
/*
 * Clock Enable Macros for UART/USART peripherals
 */
#define USART1_CLK_EN()			(RCC_HW->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 19))
#define UART5_CLK_EN()			(RCC_HW->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()			(RCC_HW->APB2ENR |= (1 << 5))
/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_CLK_EN()			(RCC_HW->APB2ENR |= (1 << 14))
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DIS()		(RCC_HW->AHB1ENR &= ~(1 << 8))
/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPI peripherals
 */
#define SPI1_CLK_DIS()			(RCC_HW->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 15))
/*
 * Clock Disable Macros for UART/USART peripherals
 */
#define USART1_CLK_DIS()			(RCC_HW->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DIS()			(RCC_HW->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DIS()			(RCC_HW->APB2ENR &= ~(1 << 5))
/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_CLK_DIS()			(RCC_HW->APB2ENR &= ~(1 << 14))
/*
 * Reset macros for the GPIO Registers
 * Set the bit corresponding to the GPIO port to 1 to reset, than back to 0 for normal ops
 */
#define GPIOA_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 0)); (RCC_HW->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 1)); (RCC_HW->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 2)); (RCC_HW->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 3)); (RCC_HW->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 4)); (RCC_HW->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 5)); (RCC_HW->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 6)); (RCC_HW->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 7)); (RCC_HW->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 8)); (RCC_HW->AHB1RSTR &= ~(1 << 8)); } while(0)

#define SPI1_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 0)); (RCC_HW->AHB1RSTR &= ~(1 << 0)); } while(0)
#define SPI2_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 1)); (RCC_HW->AHB1RSTR &= ~(1 << 1)); } while(0)
#define SPI3_REG_RESET() do{ (RCC_HW->AHB1RSTR |= (1 << 2)); (RCC_HW->AHB1RSTR &= ~(1 << 2)); } while(0)

#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15
/*
 * Interrupt Registers
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
/*
 *  returns port code for given GPIOx base address
 *  This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA_HW)?0:\
										(x == GPIOB_HW)?1:\
										(x == GPIOC_HW)?2:\
										(x == GPIOD_HW)?3:\
								        (x == GPIOE_HW)?4:\
								        (x == GPIOF_HW)?5:\
								        (x == GPIOG_HW)?6:\
								        (x == GPIOH_HW)?7: \
								        (x == GPIOI_HW)?8:0)
/*
 * Bit definition macros
 */
//GPIO
//SPI
#define SPI_CR1_BIDIMODE	15
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_DFF			11
#define SPI_CR1_RXONLY		10
#define SPI_CR1_SSM			9
#define SPI_CR1_SSI			8
#define SPI_CR1_SPE			6
#define SPI_CR1_BR2_0		3
#define SPI_CR1_MSTR		2
#define SPI_CR1_CPOL		1
#define SPI_CR1_CPHA		0

#define SPI_CR2_TXEIE		7
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_EREIE		5
#define SPI_CR2_FRF			4
#define SPI_CR2_SSOE		2
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_RXDMAEN		0

#define SPI_SR_FRE		8
#define SPI_SR_BSY		7
#define SPI_SR_OVR		6
#define SPI_SR_MODF		5
#define SPI_SR_CRCERR	4
#define SPI_SR_UDR		3
#define SPI_SR_CHSIDE	2
#define SPI_SR_TXE		1
#define SPI_SR_RXNE		0
/*
 * Generic macros and enums
 */
typedef enum
{
	disable = 0,
	enable = 1
}EnableDisable_e;
#define ENABLE 			1
#define DISABLE 		0
#define	SET 			1
#define RESET 			0
#define	GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_SET 		SET
#define FLAG_RESET 		RESET

#endif /* STM32F407XX_H_ */
