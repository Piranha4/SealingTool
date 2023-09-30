#include "stm32f446xx_adc_driver.h"

void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
	{
		if (pADCx == ADC1)
			ADC1_PCLK_EN();
		else if (pADCx == ADC2)
			ADC2_PCLK_EN();
		else if (pADCx == ADC3)
			ADC3_PCLK_EN();
	}
	else
	{
		if (pADCx == ADC1)
			ADC1_PCLK_DI();
		else if (pADCx == ADC2)
			ADC2_PCLK_DI();
		else if (pADCx == ADC3)
			ADC3_PCLK_DI();
	}
}

void ADC_Init(ADC_Handle_t *ADC_Instance)
{
    ADC_PeriClockControl(ADC_Instance->pADCx, ENABLE);

	ADC_Instance->pADCC->CCR &= ~(0x3UL << ADC_ADCPRE_POS);
    ADC_Instance->pADCC->CCR |= (ADC_Instance->ADC_Init.ClockPrescaler << ADC_ADCPRE_POS);
	ADC_Instance->pADCx->CR1 &= ~(0x1UL << ADC_SCAN_POS);
	ADC_Instance->pADCx->CR1 |= (ADC_Instance->ADC_Init.ScanConvMode << ADC_SCAN_POS);
	ADC_Instance->pADCx->CR1 &= ~(0x3UL << ADC_RES_POS);
	ADC_Instance->pADCx->CR1 |= (ADC_Instance->ADC_Init.Resolution << ADC_RES_POS);
	ADC_Instance->pADCx->CR2 &= ~(0x1UL << ADC_ALIGN_POS);
	ADC_Instance->pADCx->CR2 |= (ADC_Instance->ADC_Init.DataAlign << ADC_ALIGN_POS);
	ADC_Instance->pADCx->CR2 &= ~(0x1UL << ADC_EOCS_POS);
	ADC_Instance->pADCx->CR2 |= (ADC_Instance->ADC_Init.EOCSelection << ADC_EOCS_POS);
	ADC_Instance->pADCx->CR2 &= ~(0x1UL << ADC_CONT_POS);
	ADC_Instance->pADCx->CR2 |= (ADC_Instance->ADC_Init.ContConvMode << ADC_CONT_POS);
	ADC_Instance->pADCx->SQR[0] &= ~(0xFU << ADC_SEQ_LENGTH_POS);
	ADC_Instance->pADCx->SQR[0] |= (ADC_Instance->ADC_Init.SeqLength << ADC_SEQ_LENGTH_POS);
}

void ADC_ChannelConfig(ADC_Handle_t *ADC_Instance)
{
	uint8_t temp1;
	uint8_t temp2;
	temp1 = ADC_Instance->ADC_Init.Channel / 10;
	temp2 = ADC_Instance->ADC_Init.Channel % 10;
	if (temp1 == 1)
	{
		ADC_Instance->pADCx->SMPR[0] &= ~(0x7U << 3 * temp2);
		ADC_Instance->pADCx->SMPR[0] |= (ADC_Instance->ADC_Init.SamplingTime << 3 * temp2);
	}
	else if (temp1 == 0)
	{
		ADC_Instance->pADCx->SMPR[1] &= ~(0x7U << 3 * temp2);
		ADC_Instance->pADCx->SMPR[1] |= (ADC_Instance->ADC_Init.SamplingTime << 3 * temp2);
	}
	ADC_Instance->pADCx->CR1 &= ~(0x1FU << ADC_AWDCH_POS);
	ADC_Instance->pADCx->CR1 |= (ADC_Instance->ADC_Init.Channel << ADC_AWDCH_POS);
	/* uint8_t temp3;
	uint8_t temp4;
	temp3 = ADC_Instance->ADC_Init.RegSequence / 6;
	temp4 = ADC_Instance->ADC_Init.RegSequence % 6;
	if (temp3 == 2)
	{
		ADC_Instance->pADCx->SQR[0] &= ~(0x1FU << 5 * temp4);
		ADC_Instance->pADCx->SQR[0] |= (ADC_Instance->ADC_Init.Channel << 5 * temp4);
	}
	else if (temp3 == 1)
	{
		ADC_Instance->pADCx->SQR[1] &= ~(0x1FU << 5 * temp4);
		ADC_Instance->pADCx->SQR[1] |= (ADC_Instance->ADC_Init.Channel << 5 * temp4);
	}
	else if (temp3 == 0)
	{
		ADC_Instance->pADCx->SQR[2] &= ~(0x1FU << 5 * temp4);
		ADC_Instance->pADCx->SQR[2] |= (ADC_Instance->ADC_Init.Channel << 5* temp4);
	} */
}

void ADC_Enable(ADC_Handle_t *pADC_Instance)
{
	ADC_ENABLE(pADC_Instance);
	
	uint32_t delay = 10000;
	while (delay--);
}

void ADC_Start(ADC_Handle_t *pADC_Instance)
{
	pADC_Instance->pADCx->SQR[2] = 0;
	pADC_Instance->pADCx->SQR[2] |= (pADC_Instance->ADC_Init.Channel << 0);

	pADC_Instance->pADCx->SR = 0;

	pADC_Instance->pADCx->CR2 |= (0x1UL << ADC_SWSTART_POS);
}

void ADC_WaitForSingleConv(ADC_Handle_t *pADC_Instance)
{
	while(!(pADC_Instance->pADCx->SR & (0x1UL << ADC_EOC_POS)));
}

void ADC_WaitForWDConv(ADC_Handle_t *pADC_Instance)
{
	while(!(pADC_Instance->pADCx->SR & (0x1UL << ADC_AWD_POS)));
}

uint16_t ADC_GetVal(ADC_Handle_t *pADC_Instance)
{
	return pADC_Instance->pADCx->DR;
}

void ADC_AnalogWDConfig(ADC_Handle_t *pADC_Instance, ADC_WDConfig_t *AnalogWDConfig)
{
	if (AnalogWDConfig->ITmode == ENABLE)
		ADC_ENABLE_IT(pADC_Instance, ADC_AWDIE_POS);
	pADC_Instance->pADCx->CR1 &= ~((0x1U << ADC_AWDSGL_POS) | (0x1U << ADC_JAWDEN_POS) | (0x1U << ADC_AWDEN_POS));
	pADC_Instance->pADCx->CR1 |= AnalogWDConfig->WatchdogMode;
	pADC_Instance->pADCx->HTR = AnalogWDConfig->HighThreshold;
	pADC_Instance->pADCx->LTR = AnalogWDConfig->LowThreshold;
	pADC_Instance->pADCx->CR1 &= ~(0x1FU << ADC_AWDCH_POS);
	pADC_Instance->pADCx->CR1 |= (pADC_Instance->ADC_Init.Channel << ADC_AWDCH_POS);
}

void ADC_IRQITConfig(uint8_t irq_no, uint8_t ENorDI)
{
	uint8_t temp1 = irq_no / 32;
	uint8_t temp2 = irq_no % 32;
	if (ENorDI == ENABLE)
		*(NVIC_ISER_BASEADDR + temp1) |= (1 << temp2);
	else
		*(NVIC_ICER_BASEADDR + temp1) |= (1 << temp2);
}

void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t sa = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL); // shift amount
	*(NVIC_IPR_BASEADDR + iprx) &= ~(0xff << sa);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << sa);
}

void ADC_IRQHandling(ADC_Handle_t *pADC_Instance)
{
	if ((ADC_STATUS_FLAG(pADC_Instance, ADC_AWD_POS)) && (pADC_Instance->pADCx->CR1 & (0x1UL << ADC_AWDIE_POS)))
	{
		ADC_CLEAR_IT(pADC_Instance, ADC_AWD_POS);
		ADC_LevelOutOfWindowCallback(pADC_Instance);
	}

	if ((ADC_STATUS_FLAG(pADC_Instance, ADC_EOC_POS)) && (pADC_Instance->pADCx->CR1 & (0x1UL << ADC_EOCIE_POS)))
	{
		ADC_CLEAR_IT(pADC_Instance, ADC_EOC_POS);
		ADC_ConvCpltCallback(pADC_Instance);
	}

	if ((ADC_STATUS_FLAG(pADC_Instance, ADC_JEOC_POS)) && (pADC_Instance->pADCx->CR1 & (0x1UL << ADC_JEOCIE_POS)))
	{
		ADC_CLEAR_IT(pADC_Instance, ADC_JEOC_POS);
		InjectedConvCpltCallback(pADC_Instance);
	}

	if ((ADC_STATUS_FLAG(pADC_Instance, ADC_OVR_POS)) && (pADC_Instance->pADCx->CR1 & (0x1UL << ADC_OVRIE_POS)))
	{
		ADC_CLEAR_IT(pADC_Instance, ADC_OVR_POS);
		ADC_ErrorCallback(pADC_Instance);
	}
}

__attribute__((weak)) void ADC_ConvCpltCallback(ADC_Handle_t *pADC_Instance)
{

}

__attribute__((weak)) void ADC_LevelOutOfWindowCallback(ADC_Handle_t *pADC_Instance)
{

}

__attribute__((weak)) void InjectedConvCpltCallback(ADC_Handle_t *pADC_Instance)
{

}

__attribute__((weak)) void ADC_ErrorCallback(ADC_Handle_t *pADC_Instance)
{

}
