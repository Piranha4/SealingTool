/*
 * stm32f446xx_timer_driver.c
 *
 *  Created on: 18 sie 2022
 *      Author: hj61t7
 */

#include "stm32f446xx_timer_driver.h"

TIMx_Instance_t TIMxTab[14] = {
		{&RCC->APB2ENR, 0},
		{&RCC->APB1ENR, 0},
		{&RCC->APB1ENR, 1},
		{&RCC->APB1ENR, 2},
		{&RCC->APB1ENR, 3},
		{&RCC->APB1ENR, 4},
		{&RCC->APB1ENR, 5},
		{&RCC->APB2ENR, 1},
		{&RCC->APB2ENR, 16},
		{&RCC->APB2ENR, 17},
		{&RCC->APB2ENR, 18},
		{&RCC->APB1ENR, 6},
		{&RCC->APB1ENR, 7},
		{&RCC->APB1ENR, 8}
};

/* RCC clock settings: */

void TIM_PeriClockControl2(TIM_Instance TIMx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
		*(TIMxTab[TIMx].TIM_reg_rcc) |= (1 << TIMxTab->RCCenr_index);
	else
		*(TIMxTab[TIMx].TIM_reg_rcc) &= ~(1 << TIMxTab->RCCenr_index);
}

void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (pTIMx == TIM1)
			TIM1_PCLK_EN();
		else if (pTIMx == TIM2)
			TIM2_PCLK_EN();
		else if (pTIMx == TIM3)
			TIM3_PCLK_EN();
		else if (pTIMx == TIM4)
			TIM4_PCLK_EN();
		else if (pTIMx == TIM5)
			TIM5_PCLK_EN();
		else if (pTIMx == TIM6)
			TIM6_PCLK_EN();
		else if (pTIMx == TIM7)
			TIM7_PCLK_EN();
		else if (pTIMx == TIM8)
			TIM8_PCLK_EN();
		else if (pTIMx == TIM9)
			TIM9_PCLK_EN();
		else if (pTIMx == TIM10)
			TIM10_PCLK_EN();
		else if (pTIMx == TIM11)
			TIM11_PCLK_EN();
		else if (pTIMx == TIM12)
			TIM12_PCLK_EN();
		else if (pTIMx == TIM13)
			TIM13_PCLK_EN();
		else if (pTIMx == TIM14)
			TIM14_PCLK_EN();
	}
	else
	{
		if (pTIMx == TIM1)
			TIM1_PCLK_DI();
		else if (pTIMx == TIM2)
			TIM2_PCLK_DI();
		else if (pTIMx == TIM3)
			TIM3_PCLK_DI();
		else if (pTIMx == TIM4)
			TIM4_PCLK_DI();
		else if (pTIMx == TIM5)
			TIM5_PCLK_DI();
		else if (pTIMx == TIM6)
			TIM6_PCLK_DI();
		else if (pTIMx == TIM7)
			TIM7_PCLK_DI();
		else if (pTIMx == TIM8)
			TIM8_PCLK_DI();
		else if (pTIMx == TIM9)
			TIM9_PCLK_DI();
		else if (pTIMx == TIM10)
			TIM10_PCLK_DI();
		else if (pTIMx == TIM11)
			TIM1_PCLK_DI();
		else if (pTIMx == TIM12)
			TIM12_PCLK_DI();
		else if (pTIMx == TIM13)
			TIM13_PCLK_DI();
		else if (pTIMx == TIM14)
			TIM14_PCLK_DI();
	}
}

/* 1. Clock choice (HSI, HSE etc.) in RCC->CR
 * 2. Prescaling of data buses (AHB1, ABP1 etc) in RCC->CFGR
 * 3. Enabling chosen clock in RCC->CR
 * 4. Switch clock source if needed by RCC->CFGR
 *
 *  */

void TIM_BaseInit(TIM_Handle_t *timer, uint8_t ENorDI, uint8_t irq_no)
{
	TIM_PeriClockControl(timer->pTIMx, ENorDI);
	timer->pTIMx->ARR = timer->TIM_Config.Autoreload;
	timer->pTIMx->PSC = timer->TIM_Config.Prescaler;
	/* 1. Enable clock for timer in RCC register
	 * Set interrupt priority and enable NVIC IRQ for specified timer
	 */
	/* 2. Set frequency by timer peripheral */
	/* 3. Set prescaler value
	 * 4. Enable update interrupt
	 * 5. initialize counter
	 * 6. clear all interrupts
	 * 7. enable counter
	 */
}

void TIM_BaseStartIT(TIM_Handle_t *timer, uint8_t irq_no)
{
	NVIC_IRQ_CONFIG(irq_no, ENABLE);
	NVIC_PRIORITY_CONFIG(irq_no, 15);
	timer->pTIMx->DIER |= (1 << TIM_UIE_POS);
	TIM_BASE_START(timer);
}

void TIM_BaseStartITSlave(TIM_Handle_t *timer, uint8_t irq_no)
{
	NVIC_IRQ_CONFIG(irq_no, ENABLE);
	NVIC_PRIORITY_CONFIG(irq_no, 15);
	timer->pTIMx->DIER |= (1 << TIM_UIE_POS);
}

void TIM_BaseStopIT(TIM_Handle_t *timer)
{
	timer->pTIMx->DIER &= ~(1 << TIM_UIE_POS);
	TIM_BASE_STOP(timer);
}

void TIM_WaitForUIFSet(TIM_RegDef_t *pTIMx)
{
	while(!(pTIMx->SR & (1 << 0)));
}

void TIM_ClearStatusReg(TIM_RegDef_t *pTIMx)
{
	pTIMx->SR = 0;
}

void TIM_PWM_Config(TIM_Handle_t *timer, TIM_OC_Init_t *timConfig)
{
	timer->pTIMx->CCMR1 &= ~(0x7UL << TIM_OC1M_POS);
	timer->pTIMx->CCMR1 |= (timConfig->OCMode << TIM_OC1M_POS);
	timer->pTIMx->CCER &= ~(0x1UL << TIM_CC1P_POS);
	timer->pTIMx->CCER |= (timConfig->OCPolarity << TIM_CC1P_POS);
	// set preload enable bit for Channel 1
	timer->pTIMx->CCMR1 |= (0x1UL << TIM_OC1PE_POS);
	// set fast mode
	timer->pTIMx->CCMR1 &= ~(0x1UL << TIM_OC1FE_POS);
	timer->pTIMx->CCMR1 |= (timConfig->OCFastMode << TIM_OC1FE_POS);
	timer->pTIMx->CCR1 = timConfig->Pulse;
}

void TIM_PWM_Start(TIM_Handle_t *timer)
{
	timer->pTIMx->CCER |= (1UL << TIM_CC1E_POS);
	if (timer->pTIMx == TIM1)
		timer->pTIMx->BDTR |= (1UL << TIM_MOE_POS);
	TIM_BASE_START(timer);
}

void TIM_PWM_Stop(TIM_Handle_t *timer)
{
	timer->pTIMx->CCER &= ~(1UL << TIM_CC1E_POS);
	timer->pTIMx->CR1 &= ~(1UL << TIM_CEN_POS);
	//timer->pTIMx->BDTR &= ~(1UL << TIM_MOE_POS);
}

void TIM_SlaveConfig(TIM_Handle_t *timer, TIM_SlaveConfig_t *timSlave)
{
	timer->pTIMx->SMCR = (timSlave->SlaveModeSel << TIM_SMS_POS);
	timer->pTIMx->SMCR = (timSlave->InputTriggerSel << TIM_TS_POS);
	timer->pTIMx->SMCR = (timSlave->MSmode << TIM_MSM_POS);
	timer->pTIMx->SMCR = (timSlave->ExternalTriggerFilter << TIM_ETF_POS);
	timer->pTIMx->SMCR = (timSlave->ExternalTriggerPrc << TIM_ETPS_POS);
	timer->pTIMx->SMCR = (timSlave->ExternalClockEn << TIM_ECE_POS);
	timer->pTIMx->SMCR = (timSlave->ExternalTriggerPol << TIM_ETP_POS);
}

void TIM_MasterConfig(TIM_Handle_t *timer, TIM_MasterConfig_t *timMaster)
{
	timer->pTIMx->CR2  = (timMaster->MasterOutputTrigger << TIM_MMS_POS);
	timer->pTIMx->SMCR = (timMaster->MasterSlaveMode << TIM_MSM_POS);
}



/* Function which check why interrupt occurs */

void TIM_IRQHandler(TIM_Handle_t *timer)
{
	if (TIM_STATUS_FLAG(timer, TIM_UIF_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_UIE_POS))
		{
			TIM_CLEAR_IT(timer, TIM_UIE_POS);
			TIM_PeriodElapsedCallback(timer);
		}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_CC1IE_POS))
	{
			if (TIM_IT_SOURCE_CHECK(timer, TIM_CC1IF_POS))
			{
				TIM_CLEAR_IT(timer, TIM_CC1IE_POS);
				if ((timer->pTIMx->CCMR1 & (0x3UL << TIM_CC1S_POS)) != 0x00U)
				{
					TIM_CaptureCallback(timer);
				}
				else
				{
					OC_DelayElapsedCallback(timer);
					PWM_PulseFinishedCallback(timer);
				}
			}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_CC2IE_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_CC2IF_POS))
		{
			TIM_CLEAR_IT(timer, TIM_CC2IE_POS);
			if ((timer->pTIMx->CCMR1 & (0x3UL << TIM_CC2S_POS)) != 0x00U)
			{
				TIM_CaptureCallback(timer);
			}
			else
			{
				OC_DelayElapsedCallback(timer);
				PWM_PulseFinishedCallback(timer);
			}
		}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_CC3IE_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_CC3IF_POS))
		{
			TIM_CLEAR_IT(timer, TIM_CC3IE_POS);
			if ((timer->pTIMx->CCMR2 & (0x3UL << TIM_CC3S_POS)) != 0x00U)
			{
				TIM_CaptureCallback(timer);

			}
			else
			{
				OC_DelayElapsedCallback(timer);
				PWM_PulseFinishedCallback(timer);
			}
		}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_CC4IE_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_CC4IF_POS))
		{
			TIM_CLEAR_IT(timer, TIM_CC4IE_POS);
			if ((timer->pTIMx->CCMR2 & (0x3UL << TIM_CC4S_POS)) != 0x00U)
			{
				TIM_CaptureCallback(timer);
			}
			else
			{
				OC_DelayElapsedCallback(timer);
				PWM_PulseFinishedCallback(timer);
			}
		}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_COMIF_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_COMIE_POS))
		{
			TIM_CLEAR_IT(timer, TIM_COMIE_POS);
			TIM_CommutationCallback(timer);
		}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_TIF_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_TIE_POS))
		{
			TIM_CLEAR_IT(timer, TIM_TIE_POS);
			TIM_TriggerCallback(timer);
		}
	}
	else if (TIM_STATUS_FLAG(timer, TIM_BIF_POS))
	{
		if (TIM_IT_SOURCE_CHECK(timer, TIM_BIE_POS))
		{
			TIM_CLEAR_IT(timer, TIM_BIE_POS);
			TIM_BreakCallback(timer);
		}
	}
}


__attribute__((weak)) void TIM_PeriodElapsedCallback(TIM_Handle_t *pTIMx)
{

}

__attribute__((weak)) void TIM_CaptureCallback(TIM_Handle_t *pTIMx)
{

}

__attribute__((weak)) void OC_DelayElapsedCallback(TIM_Handle_t *pTIMx)
{

}

__attribute__((weak)) void PWM_PulseFinishedCallback(TIM_Handle_t *pTIMx)
{

}

__attribute__((weak)) void TIM_CommutationCallback(TIM_Handle_t *pTIMx)
{

}

__attribute__((weak)) void TIM_TriggerCallback(TIM_Handle_t *pTIMx)
{

}

__attribute__((weak)) void TIM_BreakCallback(TIM_Handle_t *pTIMx)
{

}

/* timer IRQ handler */
/* callback with code execution, counted by timer */
