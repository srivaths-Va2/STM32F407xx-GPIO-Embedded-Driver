/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**********************************************************************************************************************
 *
 * 						Example code to demonstrate the application of the Driver
 * 						The code intends to blink the LEDs at PD12 and PD13
 *
 * ********************************************************************************************************************/

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_gpio_driver.c"

// Defining the handle for the GPIO pin

GPIO_Handle_t hgpio1;			// GPIO Handler for GPIO pin PD12
GPIO_Handle_t hgpio2;			// GPIO Handler for GPIO pin PD13

void delay()				// software delay function
{
	uint8_t i = 0;
	for(i = 0; i < 100; i++);
}


int main(void)
{
	// Initializing the GPIO handler for pin PD12
	hgpio1.pGPIOx = GPIOD;
	hgpio1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	hgpio1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	hgpio1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	hgpio1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_OUTPUT_PP;
	hgpio1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	hgpio1.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_ALTFUN_0;

	// Initializing the GPIO handler for pin PD13
	hgpio2.pGPIOx = GPIOD;
	hgpio2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	hgpio2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	hgpio2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	hgpio2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_OUTPUT_PP;
	hgpio2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	hgpio2.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_ALTFUN_0;

	// Enabling the peripheral clock of GPIOD port
	GPIO_PericlockControl(hgpio1.pGPIOx, ENABLE);

	// Initializing GPIO pin PD12 with the parameters passed
	GPIO_Init(&hgpio1);

	// Initializing GPIO pin PD12 with the parameters passed
	GPIO_Init(&hgpio2);

	while(1)
	{
		// Toggling pin PD12
		GPIO_WritePin(hgpio1.pGPIOx, hgpio1.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
		delay();
		GPIO_WritePin(hgpio1.pGPIOx, hgpio1.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
		delay();

		// Toggling pin PD13
		GPIO_WritePin(hgpio2.pGPIOx, hgpio2.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
		delay();
		GPIO_WritePin(hgpio2.pGPIOx, hgpio2.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
		delay();
	}
}
