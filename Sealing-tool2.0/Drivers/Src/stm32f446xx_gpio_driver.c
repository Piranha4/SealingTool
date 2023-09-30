/*
 * stm32f446xx_gpio_driver.c
 *
 */

#include "stm32f446xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
	}
	else
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// enable peripheral clock for GPIO

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// 1. configure specific FTSR or RTSR or both
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the gpio port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (4 * temp2));

		// 3. Enable the EXTI interrupt delivery using Interrupt Mask Register
		EXTI->IMR |= (1 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	// 2. Configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;
	// 3. configure pu pd
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	// 4. configure output type reg
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	// 5. configure alternate function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->OTYPER &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH)
		GPIOH_REG_RESET();
}

/*GPIO read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x1;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// write 0 to the output data register
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/* GPIO interrupt configuration and handling */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;
	if (ENorDI == ENABLE)
		*(NVIC_ISER_BASEADDR + temp1) |= (1 << temp2);
	else
		*(NVIC_ICER_BASEADDR + temp1) |= (1 << temp2);
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t sa = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL); // shift amount
	*(NVIC_IPR_BASEADDR + iprx) &= ~(0xff << sa);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << sa);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
/* Configuration structure */
