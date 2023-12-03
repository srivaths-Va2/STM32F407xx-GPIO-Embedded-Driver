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
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;
	//1. Configure the GPIO pin mode

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->MODER |= temp;
	temp = 0;

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

