/*
 * stm32f446xx.h
 */

#include <stdint.h>
#define __vo volatile

#ifndef INC_STM32F466XX_H_
#define INC_STM32F466XX_H_

#define NVIC_ISER_BASEADDR				((__vo uint32_t *)0xE000E100)
#define NVIC_ICER_BASEADDR				((__vo uint32_t *)0xE000E180)
#define NVIC_IPR_BASEADDR				((__vo uint32_t *)0xE000E400)
#define NO_PR_BITS_IMPL					4

#define FLASH_BASEADDR					0x08000000U	/* base address of flash main memory */
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U

#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U
#define AHB3PERIPH_BASEADDR				0xA0001000U

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_BASEADDR					(APB2PERIPH_BASEADDR + 0x2000)
#define ADC2_BASEADDR					(APB2PERIPH_BASEADDR + 0x2100)
#define ADC3_BASEADDR					(APB2PERIPH_BASEADDR + 0x2200)
#define ADCC_BASEADDR					(APB2PERIPH_BASEADDR + 0x2300)
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)

#define TIM1_BASEADDR					APB2PERIPH_BASEADDR
#define TIM2_BASEADDR					APB1PERIPH_BASEADDR
#define TIM3_BASEADDR					(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR					(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR					(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR					(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR					(APB1PERIPH_BASEADDR + 0x1400)
#define TIM8_BASEADDR					(APB2PERIPH_BASEADDR + 0x0400)
#define TIM9_BASEADDR					(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR					(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR					(APB2PERIPH_BASEADDR + 0x4800)
#define TIM12_BASEADDR					(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR					(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR					(APB1PERIPH_BASEADDR + 0x2000)

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t ERM;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED0[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED1[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct
{
	__vo uint32_t SR;			/*!< USART Status register,             		Address offset: 0x00 */
	__vo uint32_t DR;			/*!< USART Data register,              			Address offset: 0x04 */
	__vo uint32_t BRR;			/*!< USART Baud rate register,              	Address offset: 0x08 */
	__vo uint32_t CR1;			/*!< USART control register 1,              	Address offset: 0x0C */
	__vo uint32_t CR2;			/*!< USART control register 2,              	Address offset: 0x10 */
	__vo uint32_t CR3;			/*!< USART control register 3,             		Address offset: 0x14 */
	__vo uint32_t GTPR;			/*!< USART Guard time and prescaler register,   Address offset: 0x18 */
}USART_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
	__vo uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
	__vo uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
	__vo uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
	__vo uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
	__vo uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
	__vo uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
	__vo uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
	__vo uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
	__vo uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
	__vo uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
	__vo uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
	__vo uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
	__vo uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
	__vo uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
	__vo uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
	__vo uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
	__vo uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
	__vo uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
	__vo uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
	__vo uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
}TIM_RegDef_t;

typedef struct
{
	__vo uint32_t SR;			/*!< ADC status register, 				 Address offset: 0x00 */
	__vo uint32_t CR1;			/*!< ADC control register 1, 			 Address offset: 0x04 */
	__vo uint32_t CR2;			/*!< ADC control register 2, 			 Address offset: 0x08 */
	__vo uint32_t SMPR[2];		/*!< ADC sample time registers, 		 Address offset: 0x0C */
	__vo uint32_t JOFR[4];		/*!< ADC status register, 				 Address offset: 0x14 */
	__vo uint32_t HTR;			/*!< ADC status register, 				 Address offset: 0x24 */
	__vo uint32_t LTR;			/*!< ADC status register, 				 Address offset: 0x28 */
	__vo uint32_t SQR[3];		/*!< ADC regular sequence registers, 	 Address offset: 0x2C */
	__vo uint32_t JSQR;			/*!< ADC status register, 				 Address offset: 0x38 */
	__vo uint32_t JDR[4];		/*!< ADC status register, 				 Address offset: 0x3C */
	__vo uint32_t DR;			/*!< ADC status register, 				 Address offset: 0x4C */
}ADC_RegDef_t;

typedef struct
{
	__vo uint32_t CSR;
	__vo uint32_t CCR;
	__vo uint32_t CDR;
}ADC_Common_RegDef_t;

#define GPIOA							((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF							((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG							((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH							((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC								((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI							((EXTI_RegDef_t *)(EXTI_BASEADDR))
#define SYSCFG							((SYSCFG_RegDef_t *)(SYSCFG_BASEADDR))
#define SPI1							((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4							((SPI_RegDef_t *)SPI4_BASEADDR)

#define USART1							((USART_RegDef_t *)USART1_BASEADDR)
#define USART2							((USART_RegDef_t *)USART2_BASEADDR)
#define USART3							((USART_RegDef_t *)USART3_BASEADDR)
#define UART4							((USART_RegDef_t *)UART4_BASEADDR)
#define UART5							((USART_RegDef_t *)UART5_BASEADDR)
#define USART6							((USART_RegDef_t *)USART6_BASEADDR)

#define TIM1							((TIM_RegDef_t *) TIM1_BASEADDR)
#define TIM2							((TIM_RegDef_t *) TIM2_BASEADDR)
#define TIM3							((TIM_RegDef_t *) TIM3_BASEADDR)
#define TIM4							((TIM_RegDef_t *) TIM4_BASEADDR)
#define TIM5							((TIM_RegDef_t *) TIM5_BASEADDR)
#define TIM6							((TIM_RegDef_t *) TIM6_BASEADDR)
#define TIM7							((TIM_RegDef_t *) TIM7_BASEADDR)
#define TIM8							((TIM_RegDef_t *) TIM8_BASEADDR)
#define TIM9							((TIM_RegDef_t *) TIM9_BASEADDR)
#define TIM10							((TIM_RegDef_t *) TIM10_BASEADDR)
#define TIM11							((TIM_RegDef_t *) TIM11_BASEADDR)
#define TIM12							((TIM_RegDef_t *) TIM12_BASEADDR)
#define TIM13							((TIM_RegDef_t *) TIM13_BASEADDR)
#define TIM14							((TIM_RegDef_t *) TIM14_BASEADDR)

#define ADC1							((ADC_RegDef_t *) ADC1_BASEADDR)
#define ADC2							((ADC_RegDef_t *) ADC2_BASEADDR)
#define ADC3							((ADC_RegDef_t *) ADC3_BASEADDR)
#define ADCC							((ADC_Common_RegDef_t *) ADCC_BASEADDR)

#define GPIOA_PCLK_EN()					(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()					(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()					(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |= (1 << 7))

#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1 << 15))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1 << 20))
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()					(RCC->APB1ENR |= (1 << 23))

#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))
#define ADC1_PCLK_EN()					(RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN()					(RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN()					(RCC->APB2ENR |= (1 << 10))
#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN()					(RCC->APB2ENR |= (1 << 13))
#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))

#define TIM1_PCLK_EN()					(RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN()					(RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()					(RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()					(RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()					(RCC->APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN()					(RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN()					(RCC->APB1ENR |= (1 << 5))
#define TIM8_PCLK_EN()					(RCC->APB2ENR |= (1 << 1))
#define TIM9_PCLK_EN()					(RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN()					(RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN()					(RCC->APB2ENR |= (1 << 18))
#define TIM12_PCLK_EN()					(RCC->APB1ENR |= (1 << 6))
#define TIM13_PCLK_EN()					(RCC->APB1ENR |= (1 << 7))
#define TIM14_PCLK_EN()					(RCC->APB1ENR |= (1 << 8))

#define GPIOA_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 7))

#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 15))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 20))
#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 23))

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))
#define ADC1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 10))
#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 13))
#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))

#define TIM1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 0))
#define TIM2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 3))
#define TIM6_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 5))
#define TIM8_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 1))
#define TIM9_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 16))
#define TIM10_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 17))
#define TIM11_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 18))
#define TIM12_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 6))
#define TIM13_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 7))
#define TIM14_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 8))

#define GPIOA_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()				do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

#define SPI1_REG_RESET()				do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI4_REG_RESET()				do {(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));} while(0)
#define SPI2_REG_RESET()				do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()				do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while(0)

#define USART1_REG_RESET()				do {(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while(0)
#define USART6_REG_RESET()				do {(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while(0)
#define USART2_REG_RESET()				do {(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));} while(0)
#define USART3_REG_RESET()				do {(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));} while(0)
#define UART4_REG_RESET()				do {(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));} while(0)
#define UART5_REG_RESET()				do {(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));} while(0)

#define GPIO_BASEADDR_TO_CODE(x)	  ( (x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
										(x == GPIOE) ? 4 : \
										(x == GPIOF) ? 5 : \
										(x == GPIOG) ? 6 : \
										(x == GPIOH) ? 7 :0 )

#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40

#define IRQ_NO_SPI1
#define IRQ_NO_SPI2
#define IRQ_NO_SPI3
#define IRQ_NO_SPI4

#define IRQ_NO_TIM6						54

#define IRQ_NO_ADC						18

#define ENABLE 							1
#define DISABLE 						0
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET
#define BTN_PRESSED						DISABLE
#define FLAG_SET						SET
#define FLAG_RESET						RESET

/* Bit position definitions of SPI peripheral registers */

#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRCNEXT					12
#define SPI_CR1_CRCEN					13
#define SPI_CR1_BIDIOE					14
#define SPI_CR1_BIDIMODE				15

#define SPI_CR2_RXDMAEN					0
#define SPI_CR2_TXDMAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_ERRIE					4
#define SPI_CR2_RXNEIE					5
#define SPI_CR2_TXEIE					6

#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7
#define SPI_SR_FRE						8

#define RCC_CR_HSION					1
#define RCC_CR_HSIOFF					0

#define NVIC_IRQ_CONFIG(IRQ_NO, ENorDI)				ENorDI ? (*((NVIC_ISER_BASEADDR) + (uint8_t)((IRQ_NO) / 32)) |= (1 << ((IRQ_NO) % 32))) : (*((NVIC_ICER_BASEADDR) + (uint8_t)(IRQ_NO / 32)) |= (1 << ((IRQ_NO) % 32)))
#define NVIC_PRIORITY_CONFIG(IRQ_NO, IRQ_PR)		do { (*((NVIC_IPR_BASEADDR) + (uint8_t)(8*((IRQ_NO) / 4))) &= ~(0xFF << (((IRQ_NO) % 4)  + (8 - NO_PR_BITS_IMPL)))); \
														(*((NVIC_IPR_BASEADDR) + (uint8_t)(8*((IRQ_NO) / 4))) |= ((IRQ_PR) << (((IRQ_NO) % 4) + (8 - NO_PR_BITS_IMPL)))); } while(0)

#endif /* INC_STM32F446XX_H_ */
