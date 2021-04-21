/*
 * 005button_interrupt.c
 *
 *  Created on: Mar 23, 2021
 *      Author: wadeb
 */

#include "STM32F407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>
void delay(uint32_t);

int pushButtonInterrupt(void) {

	GPIO_Handle_t GPIOD_btn, GPIOD_onbrdLED;
	memset(&GPIOD_btn, 0, sizeof(GPIO_Handle_t));
	memset(&GPIOD_onbrdLED, 0, sizeof(GPIO_Handle_t));
	GPIOD_btn.pGPIOx = GPIOD_HW;
	GPIOD_onbrdLED.pGPIOx = GPIOD_HW;

	GPIOD_CLK_EN();
	//Setup the LED
	GPIOD_onbrdLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIOD_onbrdLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOD_onbrdLED.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PSH_PULL;
	GPIOD_onbrdLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPD_LOW;
	GPIOD_onbrdLED.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE;
	//Setup the button
	GPIOD_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIOD_btn.GPIO_PinConfig.GPIO_PinPinPuPdCtrl = GPIO_PUPD_NONE;
	GPIOD_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FEDGE;

	//Setup IRQ handling
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15); // 15 is lowest priority
	//Init the GPIOs
	GPIO_Init(&GPIOD_onbrdLED);
	GPIO_Init(&GPIOD_btn);


	return 1;
}

void EXTI9_5_IRQHandler(void) {

	// Debouncing might be needed here. Delay and/or use multiple reads
	delay(250000U);
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOD_HW, GPIO_PIN_15);
}
