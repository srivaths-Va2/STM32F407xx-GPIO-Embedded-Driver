#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

/*
 * C structure for GPIO-peripheral configuration
 * */

typedef struct
{
	uint8_t GPIO_PinNumber;								/* GPIO Pin Number */
	uint8_t GPIO_PinMode;								/* GPIO Pin Mode */
	uint8_t GPIO_PinSpeed;								/* GPIO Pin Speed */
	uint8_t GPIO_PinPuPdControl;						/* GPIO Pull-up and Pull-down resistor configuration */
	uint8_t GPIO_PinOPType;								/* GPIO Pin o/p type */
	uint8_t GPIO_PinAltFunMode;							/* GPIO Pin Alternate function mode settings */
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t* pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * GPIO Pin Number macros
 * */

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
 * GPIO possibilities of pin mode
 * */

#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFUN		2
#define GPIO_MODE_ANALOG		3

/*
 * GPIO possibilities of pin speed
 * */

#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_HIGH				2
#define GPIO_SPEED_VERYHIGH			3

/*
 * GPIO possibilities of pull-up and pull-down
 * */

#define GPIO_NOPUPD					0
#define GPIO_PULLUP					1
#define GPIO_PULLDN					2
#define GPIO_PUPDRES				3

/*
 * GPIO possibilities of pull-up and pull-down
 * */

#define GPIO_OUTPUT_PP				0
#define GPIO_OUTPUT_OD				1

/*
 * GPIO possibilities of Alternate-Functionality
 * */

#define GPIO_PIN_ALTFUN_0			0
#define GPIO_PIN_ALTFUN_1			1
#define GPIO_PIN_ALTFUN_2			2
#define GPIO_PIN_ALTFUN_3			3
#define GPIO_PIN_ALTFUN_4			4
#define GPIO_PIN_ALTFUN_5			5
#define GPIO_PIN_ALTFUN_6			6
#define GPIO_PIN_ALTFUN_7			7
#define GPIO_PIN_ALTFUN_8			8
#define GPIO_PIN_ALTFUN_9			9
#define GPIO_PIN_ALTFUN_10			10
#define GPIO_PIN_ALTFUN_11			11
#define GPIO_PIN_ALTFUN_12			12
#define GPIO_PIN_ALTFUN_13			13
#define GPIO_PIN_ALTFUN_14			14
#define GPIO_PIN_ALTFUN_15			15


/************************************************************************************************
 *
 * 											APIs SUPPORTED BY GPIO PERIPHERALS
 *
 * **********************************************************************************************/

void GPIO_PericlockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t Value);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
