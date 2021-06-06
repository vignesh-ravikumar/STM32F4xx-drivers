/*
 * stm32f446re.h
 *
 *  Created on: Oct 30, 2020
 *      Author: 91883
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_

#include<stdint.h>
#include<stddef.h>
/* ARM cortex M4 processor NVIC register architecture*/

#define NVIC_ISER0				(volatile uint32_t*)0xE000E100
#define NVIC_ISER1				(volatile uint32_t*)0xE000E104
#define NVIC_ISER2				(volatile uint32_t*)0xE000E108
#define NVIC_ISER3				(volatile uint32_t*)0xE000E10C

#define NVIC_ICER0				(volatile uint32_t*)0XE000E180
#define NVIC_ICER1				(volatile uint32_t*)0XE000E184
#define NVIC_ICER2				(volatile uint32_t*)0XE000E188
#define NVIC_ICER3				(volatile uint32_t*)0XE000E18C

//IRQ priority config register base address
#define NVIC_PR_BASE_ADDR				(volatile uint32_t*)0xE000E400
#define NO_OF_PR_BITS_IMPLEMENTED		4

//base address of flash and sram

#define FLASH_BASADDR  			0X08000000U
#define SRAM1_BASRADDR 			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define SRAM 					SRAM1_BASEADDR
#define ROM_BASEADDR			0x1FFF0000U 			//System memory

//base address of various bus domains

#define PERIPH_BASE				0x40000000U
#define AHB3PERIPH_BASEADDR		0xA0000000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define APB2PERIPH_BASEADDR		0x40010000U
#define APB1PERIPH_BASEADDR		0x40000000U

//base address of peripherals hanging on AHB1 bus

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

//base address of peripherals hanging on APB1 bus

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

//base address of peripherals hanging on APB2 bus

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

//peripheral definition macros

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART3					((USART_RegDef_t*)USART3_BASEADDR)
#define UART4					((USART_RegDef_t*)UART4_BASEADDR)
#define UART5					((USART_RegDef_t*)UART5_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)

//structuring peripheral registers GPIO
typedef struct{
	volatile uint32_t MODER;			//GPIO port mode register address offset 0x04
	volatile uint32_t OTYPER;			//GPIO port output type register address offset 0x08
	volatile uint32_t OSPEEDER;			//GPIO port output speed register
	volatile uint32_t PUPDR;			//GPIO port pull-up/pull-down register
	volatile uint32_t IDR;				//GPIO port input data register
	volatile uint32_t ODR;				//GPIO port output data register
	volatile uint32_t BSRR;				//GPIO port bit set/reset register
	volatile uint32_t LCKR;				//GPIO port configuration lock register
	volatile uint32_t AFR[2];			//GPIO alternate function low and high registers
}GPIO_RegDef_t;

//structuring peripheral registers RCC
typedef struct{
	volatile uint32_t CR;				//RCC clock control register
	volatile uint32_t PLLCFGR;			//RCC PLL configuration register
	volatile uint32_t CFGR;				//RCC clock configuration register
	volatile uint32_t CIR;				//RCC clock interrupt register
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;			//RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;			//RCC AHB3 peripheral clock enable register
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
}RCC_RegDef_t;

//structuring peripheral registers EXTI
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

//structuring peripheral register SYSCFG
typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;

//structuring peripheral register SPI
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

//structuring peripheral registers for I2C
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;

typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;

//clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1<<7))

//clock enable macros for I2Cx peripherals
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))

//clock enable macros for SPIx peripherals
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1<<13))

//clock enable macros for USARTx peripherals
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()			    (RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1<<5))

//clock enable macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))

//clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<7))

//clock disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<23))

//clock disable macros for SPIx peripherals
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1<<13))

//clock disable macros for USARTx peripherals
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()			    (RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1<<5))

//clock disable macros for SYSCFG peripheral
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1<<14))

//GPIO port reset macros
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 0));  (RCC->AHB1RSTR &= ~(0x1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 1));  (RCC->AHB1RSTR &= ~(0x1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 2));  (RCC->AHB1RSTR &= ~(0x1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 3));  (RCC->AHB1RSTR &= ~(0x1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 4));  (RCC->AHB1RSTR &= ~(0x1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 5));  (RCC->AHB1RSTR &= ~(0x1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 6));  (RCC->AHB1RSTR &= ~(0x1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 7));  (RCC->AHB1RSTR &= ~(0x1 << 7)); } while(0)

//SPI port reset macros
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (0x1 << 12)); (RCC->APB1RSTR &= ~(0x01 << 12)); } while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 14)); (RCC->APB1RSTR &= ~(0x01 << 14)); } while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 15)); (RCC->APB1RSTR &= ~(0x01 << 15)); } while(0)
#define SPI4_REG_RESET()			do{ (RCC->APB2RSTR |= (0x1 << 13)); (RCC->APB1RSTR &= ~(0x01 << 13)); } while(0)

//I2C port reset macroS
#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 21)); (RCC->APB1RSTR &= ~(0x01 << 21)); } while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 22)); (RCC->APB1RSTR &= ~(0x01 << 22)); } while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 23)); (RCC->APB1RSTR &= ~(0x01 << 23)); } while(0)

//USART port reset macros
#define USART1_REG_RESET()			do{ (RCC->APB2RSTR |= (0x1 << 4)); (RCC->APB2RSTR &= ~(0x01 << 4)); }while(0)
#define USART2_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 17)); (RCC->APB1RSTR &= ~(0x01 << 17)); }while(0)
#define USART3_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 18)); (RCC->APB1RSTR &= ~(0x01 << 18)); }while(0)
#define UART4_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 19)); (RCC->APB1RSTR &= ~(0x01 << 19)); }while(0)
#define UART5_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 20)); (RCC->APB1RSTR &= ~(0x01 << 20)); }while(0)
#define USART6_REG_RESET()			do{ (RCC->APB2RSTR |= (0x1 << 5)); (RCC->APB2RSTR &= ~(0x01 << 5)); }while(0)

//return irq (interrupt request) number for exti interrupt
#define IRQ_NO_EXTIO			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

// irq (interrupt request) number for SPI interrupt
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4				84

// irq (interrupt request) number for I2C interrupt
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

//some generic macros
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_spi_driver.h"
#include "stm32f446re_i2c_driver.h"
#include "stm32f446re_usart_driver.h"
#include "stm32f446re_rcc_driver.h"

#endif /* INC_STM32F446RE_H_ */
