#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile


/************************************************************************************************************************************************************************
 *
 * 																PROCESSOR SPECIFIC REGISTERS
 * 																		ARM CORTEX M4
 *
 *
 ********************************************************************************************************************************************************************** */

/*
 * NVIC Interrupt Set Enable Registers base-Addresses
 * */
#define NVIC_ISER0				((uint32_t*)0xE000E100)
#define NVIC_ISER1				((uint32_t*)0xE000E104)
#define NVIC_ISER2				((uint32_t*)0xE000E108)
#define NVIC_ISER3				((uint32_t*)0xE000E10C)

/*
 * NVIC Interrupt Clear Enable Registers base-Addresses
 * */
#define NVIC_ICER0				((uint32_t*)0XE000E180)
#define NVIC_ICER1				((uint32_t*)0XE000E184)
#define NVIC_ICER2				((uint32_t*)0XE000E188)
#define NVIC_ICER3				((uint32_t*)0XE000E18C)

/*
 * NVIC Interrupt Priority bank base-Addresses
 * */
#define NVIC_PRIORITY_BASE_ADDR			((uint32_t*)0xE000E400)


/************************************************************************************************************************************************************************
 *
 * 																STM32F407xx SPECIFIC REGISTERS
 * 																	ST-Microelectronics
 *
 *
 ********************************************************************************************************************************************************************** */

/*
 * Base address of Flash and SRAM memories
 * */

#define FLASH_BASE_ADDR		0x08000000
#define SRAM1_BASE_ADDR		0x20000000
#define SRAM2_BASE_ADDR		0x2001C000
#define ROM_BASE_ADDR		0x1FFF0000
#define SRAM				SRAM1_BASE_ADDR

/*
 * Base addresses of Peripheral Buses (Add all bus definitions later)
 * */

#define PERIPH_BUS_BASE_ADDR		0x40000000
#define AHB1_BUS_BASE_ADDR			(PERIPH_BUS_BASE_ADDR + 0x00020000)
#define APB2_BUS_BASE_ADDR			(PERIPH_BUS_BASE_ADDR + 0x00010000)

/*
 * Base Address of peripherals hanging on AHB1 bus (Add GPIOD base address for now)
 * */

#define GPIOA_BASE_ADDR				(AHB1_BUS_BASE_ADDR)
#define GPIOB_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00000400)
#define GPIOC_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00000800)
#define GPIOD_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00000C00)
#define GPIOE_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00001000)
#define GPIOF_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00001400)
#define GPIOG_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00001800)
#define GPIOH_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00001C00)
#define GPIOI_BASE_ADDR				(AHB1_BUS_BASE_ADDR + 0x00002000)


#define RCC_BASE_ADDR               (AHB1_BUS_BASE_ADDR + 0x00003800)

#define EXTI_BASE_ADDR				(APB2_BUS_BASE_ADDR + 0x00003C00)

#define SYSCFG_BASE_ADDR			(APB2_BUS_BASE_ADDR + 0x00003800)

/*
 *
 * Defining the C structures for the peripherals
 *
 * */

// C structure for the GPIO peripheral
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
}GPIO_RegDef_t;

// C structure for the RCC peripheral
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2;
	__vo uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t RESERVED6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED8;
	__vo uint32_t RESERVED9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED10;
	__vo uint32_t RESERVED11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

// C structure for EXTI peripheral
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR1;
	__vo uint32_t EXTICR2;
	__vo uint32_t EXTICR3;
	__vo uint32_t EXTICR4;
	uint32_t CMPCR;
}SYSCFG_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses type-casted to xxx_RegDef_t)
 * */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*
 * Clock enable macros for peripherals
 * */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))


/*
 * Clock disable macros for peripherals (only GPIOD clock enable for now)
 * */

#define GPIOA_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 8))

#define SYSCFG_PCLK_DIS()			(RCC->APB2ENR &= ~(1 << 14))

/*
 * Peripheral Reset macros (only GPIOD peripheral)
 * */
#define GPIOA_RESET()				do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); }while(0)
#define GPIOB_RESET()				do{ RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); }while(0)
#define GPIOC_RESET()				do{ RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); }while(0)
#define GPIOD_RESET()				do{ RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); }while(0)
#define GPIOE_RESET()				do{ RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); }while(0)
#define GPIOF_RESET()				do{ RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); }while(0)
#define GPIOG_RESET()				do{ RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); }while(0)
#define GPIOH_RESET()				do{ RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); }while(0)
#define GPIOI_RESET()				do{ RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); }while(0)


/*
 * Some general macros
 * */

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


#endif /* INC_STM3F407XX_H_ */
