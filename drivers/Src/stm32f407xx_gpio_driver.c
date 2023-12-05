#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"


/*********************************************************************************************************************
 * @fn - 			GPIO_PericlockControl
 *
 * @brief - 		Used to enable or disable the peripheral clock for the given GPIO port
 *
 * param[1] - 		Base address of GPIOx
 * param[2] - 		ENABLE or DISABLE
 *
 * *******************************************************************************************************************/
void GPIO_PericlockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}

		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}

	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}
	}
}

/*********************************************************************************************************************
 * @fn - 			GPIO_Init
 *
 * @brief - 		Used to initialize the GPIO port with passed parameters
 *
 * param[1] - 		pointer to initialised GPIO Handler
 *
 * *******************************************************************************************************************/

uint8_t get_portcode(GPIO_RegDef_t* pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		return (uint8_t)0;
	}
	else if(pGPIOx == GPIOB)
	{
		return (uint8_t)1;
	}
	else if(pGPIOx == GPIOC)
	{
		return (uint8_t)2;
	}
	else if(pGPIOx == GPIOD)
	{
		return (uint8_t)3;
	}
	else if(pGPIOx == GPIOE)
	{
		return (uint8_t)4;
	}
	else if(pGPIOx == GPIOF)
	{
		return (uint8_t)5;
	}
	else if(pGPIOx == GPIOG)
	{
		return (uint8_t)6;
	}
	else if(pGPIOx == GPIOH)
	{
		return (uint8_t)7;
	}
	else if(pGPIOx == GPIOI)
	{
		return (uint8_t)8;
	}
	return 0;
}

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;
	//1. Configure the GPIO pin mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}

	else		// Function block to program pin for interrupts
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Set the bit on FTSR to enable falling edge interrupt event
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Reset the bit on RTSR to disable rising edge interrupt event
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Set the bit on RTSR to enable falling edge interrupt event
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Reset the bit on FTSR to disable rising edge interrupt event
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RTFT)
		{
			// 1. Configure both the RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Set the bit on FTSR to enable falling edge interrupt event
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// Set the bit on RTSR to enable rising edge interrupt event
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR register

		uint8_t syscfg_exticr_regtoaccess = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);				// syscfg_exticr_regtoaccess == 0, access EXTICR1, syscfg_exticr_regtoaccess == 1, access EXTICR2, syscfg_exticr_regtoaccess == 3, access EXTICR3 else access EXTICR4
		uint8_t syscfg_exticr_offset = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);					// syscfg_exticr_offset used to decide left shift offset based on pin number

		uint8_t portcode =  get_portcode(pGPIOHandle->pGPIOx);												// getting the port-code for GPIOx

		SYSCFG_PCLK_EN();																					// Enabling SYSCFG clock

		if(syscfg_exticr_regtoaccess == 0)
		{
			SYSCFG->EXTICR1 |= (portcode << 4*(syscfg_exticr_offset));
		}

		if(syscfg_exticr_regtoaccess == 1)
		{
			SYSCFG->EXTICR2 |= (portcode << 4*(syscfg_exticr_offset));
		}

		if(syscfg_exticr_regtoaccess == 2)
		{
			SYSCFG->EXTICR3 |= (portcode << 4*(syscfg_exticr_offset));
		}

		if(syscfg_exticr_regtoaccess == 3)
		{
			SYSCFG->EXTICR4 |= (portcode << 4*(syscfg_exticr_offset));
		}
		// 3. Enable the EXTI delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure the GPIO pin speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure the pull-up and pull-down control

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;


	// 4. Configure the output-type of the pin

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure the alternate function type of the pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFRL &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->AFRL |= temp;
		temp = 0;
	}

	else
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFRH &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->AFRH |= temp;
		temp = 0;

	}
}


/*********************************************************************************************************************
 * @fn - 			GPIO_DeInit
 *
 * @brief - 		Used to de-initialize the GPIO port
 *
 * param[1] - 		Base address of GPIO port
 *
 * *******************************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_RESET();

	else if(pGPIOx == GPIOB)
		GPIOB_RESET();

	else if(pGPIOx == GPIOA)
		GPIOC_RESET();

	else if(pGPIOx == GPIOB)
		GPIOD_RESET();

	else if(pGPIOx == GPIOA)
		GPIOE_RESET();

	else if(pGPIOx == GPIOB)
		GPIOF_RESET();

	else if(pGPIOx == GPIOA)
		GPIOG_RESET();

	else if(pGPIOx == GPIOB)
		GPIOH_RESET();

	else if(pGPIOx == GPIOA)
		GPIOI_RESET();
}


/*********************************************************************************************************************
 * @fn - 			GPIO_ReadPin
 *
 * @brief - 		Used to Read from a GPIO pin
 *
 * param[1] - 		base address of the GPIO port
 * param[2] -		Pin number
 *
 * *******************************************************************************************************************/
uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t val = 0;

	val = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return val;
}


/*********************************************************************************************************************
 * @fn - 			GPIO_ReadPort
 *
 * @brief - 		Used to read entire GPIO ports
 *
 * param[1] - 		base address of GPIOx
 *
 * *******************************************************************************************************************/
uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t val = 0;

	val = (uint16_t)pGPIOx->IDR;
	return val;
}


/*********************************************************************************************************************
 * @fn - 			GPIO_WritePin
 *
 * @brief - 		Used to write to a GPIO pin
 *
 * param[1] - 		base address of GPIOx
 * param[2] -		Pin number
 * param[3] -		Value to be written to a GPIO pin
 *
 * *******************************************************************************************************************/
void GPIO_WritePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else if(Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*********************************************************************************************************************
 * @fn - 			GPIO_WritePort
 *
 * @brief - 		Used to write to a GPIO port
 *
 * param[1] - 		base address of GPIOx
 * param[2] -		Value to be written to a GPIO port
 *
 * *******************************************************************************************************************/
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************************************************************
 * @fn - 			GPIO_IRQInterruptConfig
 *
 * @brief - 		Used to configure the interrupt on a GPIO pin
 *
 * param[1] - 		IRQ-Number of interrupt
 * param[2] -		ENABLE or DISABLE
 *
 * *******************************************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		}

		else if((IRQNumber > 32) && (IRQNumber <= 63))
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
		}

		else if((IRQNumber > 64) && (IRQNumber <= 95))
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
			*NVIC_ICER2 &= ~(1 << (IRQNumber % 64));
		}
	}

	else if(EnorDi == DISABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
			*NVIC_ISER0 &= ~(1 << IRQNumber);
		}

		else if((IRQNumber > 32) && (IRQNumber <= 63))
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			*NVIC_ISER1 &= ~(1 << (IRQNumber % 32));
		}

		else if((IRQNumber > 64) && (IRQNumber <= 95))
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
			*NVIC_ISER2 &= ~(1 << (IRQNumber % 64));
		}
	}
}


/*********************************************************************************************************************
 * @fn - 			GPIO_IRQPriorityConfig
 *
 * @brief - 		Used to configure the priority of the interrupt on a GPIO pin
 *
 * param[1] - 		IRQ-Number of interrupt
 * param[2] -		Priority value
 *
 * *******************************************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority)
{
	uint8_t int_priority_reg = (IRQNumber / 4);					// variable used to decide which register to access in NVIC priority bank
	uint8_t int_reg_section = (IRQNumber % 4);					// variable used to access the corresponding section of IRQ number

	*(NVIC_PRIORITY_BASE_ADDR + (4 * int_priority_reg)) |= (Priority << 8 * (int_reg_section));
}


/*********************************************************************************************************************
 * @fn - 			GPIO_IRQHandling
 *
 * @brief - 		Used to Service the ISR by setting EXTI-Pending Register
 *
 * param[1] - 		Pin number at which interrupt is serviced
 *
 * *******************************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		// clear the PR to ensure the interrupt is serviced
		EXTI->PR |= (1 << PinNumber);
	}
}
