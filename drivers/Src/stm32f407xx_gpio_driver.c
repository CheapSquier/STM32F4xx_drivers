/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Mar 9, 2021
 *      Author: wadeb
 */
#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes a GPIO port, given a declared GPIO Handle struct,
 * 						which consists of a GPIO Register struct and a GPIO Pin struct, both defined in
 * 						the main STM32 .h file
 *
 * @param[in]         - GPIO Handle struct
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t	mode, pin;

	mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// Given a GPIO pin #:
	// 1. Configure the Mode
	if (mode <= GPIO_MODE_ANALOG) {
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pin*2); // clear the bits so the bitwise OR properly sets
		pGPIOHandle->pGPIOx->MODER |= (mode << pin*2);
	} else {
		// This is for interrupts.
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FEDGE ) {
			//1. configure the FTSR
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pin*2); // Sets to input mode (0)
			EXTI_HW->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI_HW->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_REDGE ) {
			//1. configure the RTSR
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pin*2);  // Sets to input mode (0)
			EXTI_HW->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI_HW->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFEDGE ) {
			//1. configure the FTSR and RTSR
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pin*2);  // Sets to input mode (0)
			EXTI_HW->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI_HW->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// 2. configure the GPIO port selection in the SYSCFG_EXTICR[1-4]
			/*
			 * a. Pick with EXTICR# (1-4) based on which EXTI# which depends on
			 *    the PinNumber
			 * b. The 4 bits will be selected also based on EXTI#/PinNumber
			 * c. The value of the 4 bits depends on the port
			 */
		SYSCFG_CLK_EN();
		uint8_t extiNum, bitShift, value;

		extiNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		bitShift = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4;
		value = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_HW->EXTICR[extiNum] &= ~(0xf << bitShift); //clear bits
		SYSCFG_HW->EXTICR[extiNum] |= (value << bitShift); //set bits

		// 3. Enable EXTI interrupt delivery using the IMR
		EXTI_HW->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		// 4. Enable the NVIC in the core (do this in GPIO_IRQConfig function)

	}
	// 2. Configure the speed (Only needed for Output and some Alt Func modes????)
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pin*2); // clear both bits to initialize
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << pin*2);

	// 3. Configure Pull-up/Pull-down
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pin*2); // clear both bits to initialize
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPinPuPdCtrl << pin*2);

	// 4. Configure Pin Output Type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pin); // clear the bit to initialize
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType << pin);

	// 5. Configure Alternate Function, if needed
	if (mode == GPIO_MODE_ALTFN) {
		//Configure the alt function registers
		if (pin < 8) {
			pGPIOHandle->pGPIOx->AFRL &= ~(0xf << pin*4); // 1st, clear the bits, 2nd, set the bits
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << pin*4);
		} else {
			pGPIOHandle->pGPIOx->AFRH &= ~(0xf << (pin-8)*4); // Same, but have to offset for upper 8 pins
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (pin-8)*4);
		}

	}

}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Undoes what the GPIO_Init() function did.
 *
 * @param[in]         - GPIO Handle struct        -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle)
{
	if (pGPIOHandle->pGPIOx == GPIOA_HW) {
		GPIOA_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOB_HW) {
		GPIOB_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOC_HW) {
		GPIOC_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOD_HW) {
		GPIOD_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOE_HW) {
		GPIOE_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOF_HW) {
		GPIOF_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOG_HW) {
		GPIOG_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOH_HW) {
		GPIOH_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOI_HW) {
		GPIOI_REG_RESET();
	}
}

// Clock Control
/*********************************************************************
 * @fn      		  - GPIO_ClkControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ClkControl(GPIO_Reg_t *pGPIOx, uint8_t EnaOrDis)
{
	if (EnaOrDis == ENABLE) {
		if (pGPIOx == GPIOA_HW) {
			GPIOA_CLK_EN();
		} else if (pGPIOx == GPIOB_HW) {
			GPIOB_CLK_EN();
		} else if (pGPIOx == GPIOC_HW) {
			GPIOC_CLK_EN();
		} else if (pGPIOx == GPIOD_HW) {
			GPIOD_CLK_EN();
		} else if (pGPIOx == GPIOE_HW) {
			GPIOE_CLK_EN();
		} else if (pGPIOx == GPIOF_HW) {
			GPIOF_CLK_EN();
		} else if (pGPIOx == GPIOG_HW) {
			GPIOG_CLK_EN();
		} else if (pGPIOx == GPIOH_HW) {
			GPIOH_CLK_EN();
		} else if (pGPIOx == GPIOI_HW) {
			GPIOI_CLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA_HW) {
			GPIOA_CLK_DIS();
		} else if (pGPIOx == GPIOB_HW) {
			GPIOB_CLK_DIS();
		} else if (pGPIOx == GPIOC_HW) {
			GPIOC_CLK_DIS();
		} else if (pGPIOx == GPIOD_HW) {
			GPIOD_CLK_DIS();
		} else if (pGPIOx == GPIOE_HW) {
			GPIOE_CLK_DIS();
		} else if (pGPIOx == GPIOF_HW) {
			GPIOF_CLK_DIS();
		} else if (pGPIOx == GPIOG_HW) {
			GPIOG_CLK_DIS();
		} else if (pGPIOx == GPIOH_HW) {
			GPIOH_CLK_DIS();
		} else if (pGPIOx == GPIOI_HW) {
			GPIOI_CLK_DIS();
		}
	}
}

// Read/Write/Toggle
/*********************************************************************
 * @fn      		  - GPIO_ReadInputPin
 *
 * @brief             - Reads the specified pin from the specified port. Since it's only reading
 * 						one pin, it only returns a 0 or 1.
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number, 0-15
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

uint8_t  GPIO_ReadInputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
/*********************************************************************
 * @fn      		  - GPIO_ReadInputPort
 *
 * @brief             - Reads all 16 pins on the specified port, returning a 16 bit data word.
 *
 * @param[in]         - Base address of the GPIO peripheral
 *
 * @return            -  16 bit data word
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadInputPort(GPIO_Reg_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR & 0x0000FFFF);
	return value;
}
/*********************************************************************
 * @fn      		  - GPIO_WriteOutputPin
 *
 * @brief             - Writes the specified pin of the specified port. Since it's only writing
 * 						one pin, it only requires a 0 or 1 as in input data value.
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number, 0-15
 * @param[in]         - Data value to write, 0 or 1
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void     GPIO_WriteOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	// Error check that value is either 0 or 1??
	if (value == SET) {
		pGPIOx->ODR |= (1U << PinNumber); // Set the bit
	} else {
		pGPIOx->ODR &= ~(1U << PinNumber); // Reset the bit
	}
	return;
}
/*********************************************************************
 * @fn      		  - GPIO_WriteOutputPort
 *
 * @brief             - Writes all 16 pins on the specified port. A 16 bit data word is required
 * 						as input data.
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - 16 bit data word
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void     GPIO_WriteOutputPort(GPIO_Reg_t *pGPIOx, uint16_t data)
{
	pGPIOx->ODR = (uint32_t)data;
	return;
}
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Simply inverts the output state of the specified pin on the specified port.
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number, 0-15
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void     GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (0x1U << PinNumber);

	return;
}

// IRQ Config and ISR Handling
/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Sets up an IRQ with a priority value and leaves it in an enabled or
 * 						disabled state.
 *
 * @param[in]         - What number we want to assign this IRQ.
 * @param[in]		  - ENABLE or DISABLE macro (or 0 or 1)
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnaOrDis) {

	if (EnaOrDis == ENABLE) {
		// Set NVIC_ISER0-NVIC_ISER7. Which register depends on which IRQNumber
		// Which ISER# depends on which IRQ Number. The STM32F405 has 82 IRQ lines
		if (IRQNumber < 32) { // ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber >31 && IRQNumber < 64) { // ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >63 && IRQNumber < 96) { // ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	} else {
		// Set NVIC_ICER0-NVIC_ICER7
		if (IRQNumber < 32) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber >31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >63 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}

}
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

	// Now take care of setting the priority

	uint8_t IPR_regNum = IRQNumber / 4;
	uint8_t shiftAmount = (IRQNumber % 4)*8 + (8 - NUM_PR_BITS_USED); // +4 to get to the upper 4 bits

	// Ok, because the baseaddr we're working with is a uint32_t and a pointer, incrementing
	// or multiplying operates in increments of uint32_t bytes (i.e. 4). So we have to compensate
	// for that when we offset to a new address.
	*(NVIC_IPR_BASEADDR + IPR_regNum*4/sizeof(NVIC_IPR_BASEADDR)) &= ~(0xff << shiftAmount);
	*(NVIC_IPR_BASEADDR + IPR_regNum*4/sizeof(NVIC_IPR_BASEADDR)) |= (IRQPriority << shiftAmount);

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Specifies what to do with IRQs for this pin
 *
 * @param[in]         - Pin number, 0-15
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

	// The code is setup such that the weak IRQHandler defined in the project startup file
	// will call this handler. This function requires PinNumber, but the weak definition does
	// not, so you'll be hardcoding the pin# into the call from that handler to this one.
	// Note that the startup takes care of putting the weak defined IRQHandler at the
	// correct address.

	// Check if pending is set. If yes, clear it.
	if (EXTI_HW->PR & (1 << PinNumber)) {
		// clear the pending bit by writing a 1 to it.
		EXTI_HW->PR |= (1 << PinNumber);
	}
}
