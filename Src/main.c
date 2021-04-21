/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "STM32F407xx.h"
#include "stm32f407xx_gpio_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
int LED_and_Button(uint8_t, uint8_t);
int extLED_and_Button(uint8_t, uint8_t);
int LED_toggle(uint8_t);
void delay(uint32_t);
int pushButtonInterrupt(void);

int main(void) {
	//extLED_and_Button(GPIO_OUT_PSH_PULL, 1);
	//LED_and_Button(GPIO_OUT_PSH_PULL, 1);
	//LED_toggle(GPIO_OUT_OPEN_DRAIN);
	pushButtonInterrupt();

	/* Loop forever */
	for(;;);
}
int LED_and_Button(uint8_t output_type, uint8_t toggle_mode)
{
	// toggle_mode = 0, LED on when the button is pushed, off when it's not
	// toggle_mode = 1, LED toggles when the button is pushed and released

	// Turn on an the blue LED (LED6, connected to PD15with a 680ohm resistor) when
	// the blue user button is pushed (connected to PA0). PA0 will see VDD when the
	// button is pushed. So if we're set to pull-up

	// With Vdd on the button, open collector will turn the light on when the driver is activated.
	// With push-pull, when the driver is on, the LED will be off.

	GPIO_Handle_t GPIOA_handle, GPIOD_handle;
	GPIOA_handle.pGPIOx = GPIOA_HW;
	GPIOD_handle.pGPIOx = GPIOD_HW;

	GPIOA_CLK_EN();
	GPIOD_CLK_EN();

	GPIOA_handle.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIOA_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOA_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE; // since ext pull-down

	GPIOD_handle.GPIO_PinConfig.GPIO_PinNumber = 2;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	//output_type should be GPIO_OUT_PSH_PULL or GPIO_OUT_OPEN_DRAIN
	GPIOD_handle.GPIO_PinConfig.GPIO_PinOutType = output_type;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GPIOA_handle);
	GPIO_Init(&GPIOD_handle);

	if (toggle_mode) {
		while(1) {
			if (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 0)== 1) {
				while (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 0)== 1) {
					// Maybe this double read acts like a debounce so no delay is needed?
					continue;
				}
				GPIO_ToggleOutputPin(GPIOD_handle.pGPIOx, 15);
			}
		}
	} else {
		while(1) {
			// if GPIOA/PA0 is low, set GPIOD/PD15 to low
			if (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 0)== 0) {
				GPIO_WriteOutputPin(GPIOD_handle.pGPIOx, 15, 0);
			} else {
				GPIO_WriteOutputPin(GPIOD_handle.pGPIOx, 15, 1);
			}
		}
	}


    /* Loop forever */
	for(;;);
}

int extLED_and_Button(uint8_t output_type, uint8_t toggle_mode)
{
	// toggle_mode = 0, LED on when the button is pushed, off when it's not
	// toggle_mode = 1, LED toggles when the button is pushed and released

	// Turn on an the blue LED (LED6, connected to PD15with a 680ohm resistor) when
	// the blue user button is pushed (connected to PA0). PA0 will see VDD when the
	// button is pushed. So if we're set to pull-up

	// With Vdd on the button, open collector will turn the light on when the driver is activated.
	// With push-pull, when the driver is on, the LED will be off.

	GPIO_Handle_t GPIOA_handle, GPIOD_handle;
	GPIOA_handle.pGPIOx = GPIOA_HW;
	GPIOD_handle.pGPIOx = GPIOD_HW;

	GPIOA_CLK_EN();
	GPIOD_CLK_EN();

	GPIOA_handle.GPIO_PinConfig.GPIO_PinNumber = 1; //GPIOA, pin 1 (PA1)
	GPIOA_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOA_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE; // since ext pull-down

	GPIOD_handle.GPIO_PinConfig.GPIO_PinNumber = 2;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	//output_type should be GPIO_OUT_PSH_PULL or GPIO_OUT_OPEN_DRAIN
	GPIOD_handle.GPIO_PinConfig.GPIO_PinOutType = output_type;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GPIOA_handle);
	GPIO_Init(&GPIOD_handle);

	if (toggle_mode) {
		while(1) {
			if (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 1)== 1) {
				while (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 1)== 1) {
					// Maybe this double read acts like a debounce so no delay is needed?
					continue;
				}
				GPIO_ToggleOutputPin(GPIOD_handle.pGPIOx, 2);
			}
		}
	} else {
		while(1) {
			// if GPIOA/PA0 is low, set GPIOD/PD15 to low
			if (GPIO_ReadInputPin(GPIOA_handle.pGPIOx, 1)== 0) {
				GPIO_WriteOutputPin(GPIOD_handle.pGPIOx, 2, 0);
			} else {
				GPIO_WriteOutputPin(GPIOD_handle.pGPIOx, 2, 1);
			}
		}
	}


    /* Loop forever */
	for(;;);
}

int LED_toggle(uint8_t output_type) {

	GPIO_Handle_t GPIOD_handle;
	GPIOD_handle.pGPIOx = GPIOD_HW;

	GPIOD_handle.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinOutType = output_type;
	GPIOD_handle.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE;

	GPIOD_CLK_EN();
	GPIO_Init(&GPIOD_handle);

	while(1) {
		GPIO_ToggleOutputPin(GPIOD_handle.pGPIOx, 15);
		delay(2000000U);
	}
}

void delay(uint32_t dly_value) {
	int i;
	for(i=0; i < dly_value; i++) {
	}
	return;
}
