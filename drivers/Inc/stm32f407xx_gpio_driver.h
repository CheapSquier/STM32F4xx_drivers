/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Mar 9, 2021
 *      Author: wadeb
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

#include "STM32F407xx.h"

/*
 * Configuration structure for GPIO pins
 */
typedef struct
{
	uint8_t	GPIO_PinNumber;			/*!<>*/
	uint8_t	GPIO_PinMode;			/*!<Possible values from @GPIO_PIN_MODES>*/
	uint8_t	GPIO_PinSpeed;			/*!<Possible values from @GPIO_OUTPUT_TYPE>*/
	uint8_t	GPIO_PinPinPuPdCtrl;	/*!<Possible values from @GPIO_OUTPUT_SPEED>*/
	uint8_t	GPIO_PinOutType;		/*!<Possible values from @GPIO_PUPD_TYPE>*/
	uint8_t	GPIO_PinAltFuncMode;	/*!<Possible values from @GPIO_ALT_FUNC_MODES>*/
}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_Reg_t *pGPIOx; /*!< Holds base addr of this pin's GPIO port >*/
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
// Exhaustively define macros for PIN0 0, PIN1 1.... that seems like overkill
/*
 * @GPIO_PIN_MODES
 * GPIO Modes and Interrupt Types
 */
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT 			1
#define GPIO_MODE_ALTFN 		2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_INT_FEDGE		4
#define GPIO_MODE_INT_REDGE		5
#define GPIO_MODE_INT_RFEDGE	6
/*
 * @GPIO_OUTPUT_TYPE
 * GPIO Output Type
 */
#define GPIO_OUT_PSH_PULL		0
#define GPIO_OUT_OPEN_DRAIN		1
/*
 * @GPIO_OUTPUT_SPEED
 * GPIO Output Speed
 */
#define GPIO_OUT_SPD_LOW		0
#define GPIO_OUT_SPD_MED		1
#define GPIO_OUT_SPD_HIGH		2
#define GPIO_OUT_SPD_VHIGH		3
/*
 * @GPIO_PUPD_TYPE
 * GPIO Pull-Up/Pull-Down
 */
#define GPIO_PUPD_NONE			0
#define GPIO_PUPD_PU			1
#define GPIO_PUPD_PD			2

/*
 * @GPIO_ALT_FUNC_MODES
 * Different alt functions
 */
/***********************************************************************
 * 					APIs supported by this GPIO driver
 *
 */
// Init/DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle);
/* Instructor: void GPIO_DeInit(GPIO_Reg_t *pGPIOx);
 * Me: Why can't we just use the Reg ptr already in our GPIO Handle?
 *     void GPIO_DeInit(GPIO_Reg_t *pGPIOHandle);
 * The latter just means the user will select the pGPIOx variable from the handle
 * themselves, which is probably more clear. But, I think I prefer the idea
 * of init and DeInit taking the same parameter type. Seems less confusing*/

// Clock Control
/* Fancy version:
 * void GPIO_ClkControl(GPIO_Reg_t *pGPIOx, EnableDisable_t EnaOrDis);
 * Simpler version below should just get ENABLE or DISABLE macro
 */
void GPIO_ClkControl(GPIO_Reg_t *pGPIOx, uint8_t EnaOrDis);

// Read/Write/Toggle
uint8_t  GPIO_ReadInputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_Reg_t *pGPIOx);
void     GPIO_WriteOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void     GPIO_WriteOutputPort(GPIO_Reg_t *pGPIOx, uint16_t data);
void     GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber);

// IRQ Config and ISR Handling
// Note: Could have IRQ grouping in IRQConfig
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnaOrDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F407XX_GPIO_DRIVER_H_ */
