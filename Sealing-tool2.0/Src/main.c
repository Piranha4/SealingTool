#include <stdint.h>
#include "main.h"
#include <string.h>

volatile uint8_t Toolstatus = 0;

TIM_Handle_t timer1;
TIM_Handle_t timer2;
TIM_Handle_t timer3;
TIM_Handle_t timer4;
TIM_Handle_t timer5;
TIM_Handle_t timer6;
ADC_Handle_t adc1;

void delay()
{
	for (int i = 0; i < 250000/2; ++i);
}

int main(void)
{
	/* Initialization GPIOs for stepper motor */
	stepper_init();
	/* Set GPIOs for stepper motor for DRV8825 driver */
	drv8825_activate();
	/* Initialization pins of actuators  */
	GPIOactuator_Init();
	/* Initialization of timer 1 - step PWM control for stepper motor */
	TIMER1_Init();
	/* Initialization of timer 2 - step counting (slave of master timer 1) */
	TIMER2_Init();
	/* Initialization of timers for actuators */
	TIMER3_Init();
	TIMER4_Init();
	TIMER5_Init();
	/* Initialization of start button */
	GPIO_button_Init();
	Limit_SwitchInit();

	TIM_PWM_Start(&timer3);
	ADC_Meas_Init();
	for(;;)
	{

	}
}

void stepper_init()
{
	GPIO_Handle_t gpio_step;
	GPIO_Handle_t gpio_dir;
	GPIO_Handle_t gpio_reset;
	GPIO_Handle_t gpio_sleep;
	/* GPIO_Handle_t gpio_fault;
	GPIO_Handle_t gpio_enable; */
	gpio_step.pGPIOx = GPIOA;
	gpio_step.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio_step.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpio_step.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_step.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpio_step.GPIO_PinConfig.GPIO_PinAltFunMode = 1;

	GPIO_Handle_t gpio_m0;
	gpio_m0.pGPIOx = GPIOC;
	gpio_m0.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_m0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_m0.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_m0.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_m1;
	gpio_m1.pGPIOx = GPIOD;
	gpio_m1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_m1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	gpio_m1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_m1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpio_m2;
	gpio_m2.pGPIOx = GPIOB;
	gpio_m2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_m2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_m2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_m2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpio_dir.pGPIOx = GPIOC;
	gpio_dir.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_dir.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	gpio_dir.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_dir.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpio_reset.pGPIOx = GPIOC;
	gpio_reset.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_reset.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	gpio_reset.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_reset.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpio_sleep.pGPIOx = GPIOC;
	gpio_sleep.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_sleep.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_sleep.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_sleep.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpio_step);
	GPIO_Init(&gpio_sleep);
	GPIO_Init(&gpio_dir);
	GPIO_Init(&gpio_reset);
	GPIO_Init(&gpio_m1);
	GPIO_Init(&gpio_m2);
	GPIO_Init(&gpio_m0);
}

void GPIO_button_Init()
{
	GPIO_Handle_t gpio_btn1;
	gpio_btn1.pGPIOx = GPIOC;
	gpio_btn1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	gpio_btn1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_btn1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpio_btn1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* Initialization with enable RCC for GPIO */
	GPIO_Init(&gpio_btn1);
	GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI4, 3);
}

void Limit_SwitchInit()
{
	GPIO_Handle_t gpio_limit;
	gpio_limit.pGPIOx = GPIOA;
	gpio_limit.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	gpio_limit.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpio_limit.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpio_limit.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpio_limit.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&gpio_limit);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 4);
}

void drv8825_activate()
{
	GPIO_WriteToOutputPin(GPIOC, DIR_PIN, GPIO_PIN_SET); // set direction
	GPIO_WriteToOutputPin(GPIOC, M0_PIN, GPIO_PIN_SET); // set direction
	GPIO_WriteToOutputPin(GPIOB, M1_PIN, GPIO_PIN_SET); // set microsteps m1 output
	GPIO_WriteToOutputPin(GPIOB, M2_PIN, GPIO_PIN_RESET); // set microsteps m2 output
	GPIO_WriteToOutputPin(GPIOC, RESET_PIN, GPIO_PIN_SET); // set reset pin
	GPIO_WriteToOutputPin(GPIOC, SLEEP_PIN, GPIO_PIN_RESET); // set sleep pin
	delay();
}

void GPIOactuator_Init()
{
	GPIO_Handle_t gpio_act1;
	GPIO_Handle_t gpio_act2;
	GPIO_Handle_t gpio_act3;

	/* Duzy silownik */
	gpio_act1.pGPIOx = GPIOB;
	gpio_act1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio_act1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_act1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpio_act1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_act1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpio_act1.GPIO_PinConfig.GPIO_PinAltFunMode = 2;

	/* Srodkowy maly silownik */
	gpio_act2.pGPIOx = GPIOC;
	gpio_act2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio_act2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_act2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpio_act2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_act2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpio_act2.GPIO_PinConfig.GPIO_PinAltFunMode = 2;

	/* Dwa boczne silowniki */
	gpio_act3.pGPIOx = GPIOA;
	gpio_act3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio_act3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_act3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_act3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_act3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpio_act3.GPIO_PinConfig.GPIO_PinAltFunMode = 2;

	GPIO_Init(&gpio_act1);
	GPIO_Init(&gpio_act2);
	GPIO_Init(&gpio_act3);
}

void TIMER1_Init()
{
	TIM_OC_Init_t tim1PWM_Config;
	TIM_MasterConfig_t tim1Master;
	memset(&tim1PWM_Config, 0, sizeof(tim1PWM_Config));
	timer1.pTIMx = TIM1;
	timer1.TIM_Config.Autoreload = TIM1_PERIOD - 1;
	timer1.TIM_Config.Prescaler = TIM1_PRESCALER_VALUE;
	tim1PWM_Config.OCMode = TIM_OC1M_PWM_MODE1;
	tim1PWM_Config.OCPolarity = TIM_CC1P_ACT_HIGH;
	tim1PWM_Config.Pulse = TIM1_PULSE;
	tim1Master.MasterOutputTrigger = TIM_MMS_UPD;
	tim1Master.MasterSlaveMode = TIM_MSM_ON;
	TIM_BaseInit(&timer1, ENABLE,  TIM1_IRQ_NO);
	TIM_PWM_Config(&timer1, &tim1PWM_Config);
	TIM_MasterConfig(&timer1, &tim1Master);
}

void TIMER2_Init()
{
	TIM_MasterConfig_t tim2Master;
	TIM_SlaveConfig_t tim2Slave;
	memset(&tim2Slave, 0, sizeof(tim2Slave));
	timer2.pTIMx = TIM2;
	timer2.TIM_Config.Autoreload = TIM2_PERIOD - 1;
	timer2.TIM_Config.Prescaler = TIM2_PRESCALER_VALUE;
	tim2Slave.SlaveModeSel = TIM_SMS_EXT_CLK;
	tim2Slave.MSmode = TIM_MSM_ON;
	tim2Slave.InputTriggerSel = TIM_ITR0;
	tim2Master.MasterOutputTrigger = TIM_MMS_RST;
	tim2Master.MasterSlaveMode = TIM_MSM_OFF;
	TIM_BaseInit(&timer2, ENABLE,  TIM2_IRQ_NO);
	NVIC_PRIORITY_CONFIG(TIM2_IRQ_NO, 15);
	TIM_EGR_UG_SET(&timer2);
	TIM_MasterConfig(&timer2, &tim2Master);
	TIM_SlaveConfig(&timer2, &tim2Slave);
}

void TIMER3_Init()
{
	TIM_OC_Init_t tim3PWM_Config;
	memset(&tim3PWM_Config, 0, sizeof(tim3PWM_Config));
	timer3.pTIMx = TIM3;
	timer3.TIM_Config.Autoreload = TIM3_PERIOD - 1;
	timer3.TIM_Config.Prescaler = TIM3_PRESCALER_VALUE;
	tim3PWM_Config.OCMode = TIM_OC1M_PWM_MODE1;
	tim3PWM_Config.OCPolarity = TIM_CC1P_ACT_HIGH;
	tim3PWM_Config.Pulse = TIM3_PULSE;
	TIM_BaseInit(&timer3, ENABLE, TIM3_IRQ_NO);
	TIM_PWM_Config(&timer3, &tim3PWM_Config);
}

void TIMER4_Init()
{
	TIM_OC_Init_t tim4PWM_Config;
	memset(&tim4PWM_Config, 0, sizeof(tim4PWM_Config));
	timer4.pTIMx = TIM4;
	timer4.TIM_Config.Autoreload = TIM4_PERIOD - 1;
	timer4.TIM_Config.Prescaler = TIM4_PRESCALER_VALUE;
	tim4PWM_Config.OCMode = TIM_OC1M_PWM_MODE1;
	tim4PWM_Config.OCPolarity = TIM_CC1P_ACT_HIGH;
	tim4PWM_Config.Pulse = TIM4_PULSE;
	TIM_BaseInit(&timer4, ENABLE, TIM4_IRQ_NO);
	TIM_PWM_Config(&timer4, &tim4PWM_Config);
}

void TIMER5_Init()
{
	TIM_OC_Init_t tim5PWM_Config;
	memset(&tim5PWM_Config, 0, sizeof(tim5PWM_Config));
	timer5.pTIMx = TIM5;
	timer5.TIM_Config.Autoreload = TIM5_PERIOD - 1;
	timer5.TIM_Config.Prescaler = TIM5_PRESCALER_VALUE;
	tim5PWM_Config.OCMode = TIM_OC1M_PWM_MODE1;
	tim5PWM_Config.OCPolarity = TIM_CC1P_ACT_HIGH;
	tim5PWM_Config.Pulse = TIM5_PULSE;
	TIM_BaseInit(&timer5, ENABLE, TIM5_IRQ_NO);
	TIM_PWM_Config(&timer5, &tim5PWM_Config);
}

void ADC_Meas_Init()
{
	GPIO_Handle_t act_pos1, act_pos2, act_pos3;
	ADC_WDConfig_t adcWD;

	memset(&adc1, 0, sizeof(adc1));

	adc1.pADCx = ADC1;
	adc1.ADC_Init.ClockPrescaler = ADC_ADCPRE_DIV2;
	adc1.ADC_Init.ContConvMode = ADC_CONT_EN;
	adc1.ADC_Init.Resolution = ADC_RES_10_BITS;
	adc1.ADC_Init.DataAlign = ADC_ALIGN_RIGHT;
	adc1.ADC_Init.EOCSelection = ADC_EOCS_EN;
	adc1.ADC_Init.ScanConvMode = ADC_SCAN_EN;
	adc1.ADC_Init.SeqLength = ADC_SEQ_LENGTH_4;

	adcWD.HighThreshold = 650;
	adcWD.LowThreshold = 250;
	adcWD.ITmode = ENABLE;
	adcWD.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;

	act_pos1.pGPIOx = GPIOA;
	act_pos1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	act_pos1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	act_pos1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	act_pos1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	act_pos1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	act_pos2.pGPIOx = GPIOA;
	act_pos2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	act_pos2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	act_pos2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	act_pos2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	act_pos2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	act_pos3.pGPIOx = GPIOB;
	act_pos3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	act_pos3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	act_pos3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	act_pos3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	act_pos3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	ADC_Init(&adc1);
	ADC_AnalogWDConfig(&adc1, &adcWD);
	GPIO_Init(&act_pos1);
	GPIO_Init(&act_pos2);
	GPIO_Init(&act_pos3);
}

void stepper_run(uint8_t dir)
{
	GPIO_IRQITConfig(IRQ_NO_EXTI15_10, ENABLE);
	GPIO_WriteToOutputPin(GPIOC, DIR_PIN, dir);
	GPIO_WriteToOutputPin(GPIOC, SLEEP_PIN, GPIO_PIN_SET);
	timer2.pTIMx->ARR = (((STEPS_PER_REV * MICROSTEPS * 75) / 360)*2);
	TIM_PWM_Start(&timer1);
	TIM_BaseStartIT(&timer2, TIM2_IRQ_NO);
}
void stepper_stop()
{
	GPIO_WriteToOutputPin(GPIOC, SLEEP_PIN, GPIO_PIN_RESET);
	TIM_PWM_Stop(&timer1);
	TIM_BaseStopIT(&timer2);
}

/* Srodkowy silownik */
void ADC_ACT1_Start()
{
	adc1.ADC_Init.Channel = 1;
	adc1.ADC_Init.SamplingTime = ADC_SMPR_480_CYC;
	ADC_ChannelConfig(&adc1);		
	ADC_WD_HTR_CFG(&adc1, ADC_WDH_ACT1);
	ADC_WD_LTR_CFG(&adc1, ADC_WDL_ACT1);
	ADC_Enable(&adc1);
	ADC_Start(&adc1);
	NVIC_IRQ_CONFIG(IRQ_NO_ADC, ENABLE);
}

/* Duzy silownik */
void ADC_ACT2_Start()
{
	adc1.ADC_Init.Channel = 4;
	adc1.ADC_Init.SamplingTime = ADC_SMPR_480_CYC;
	ADC_ChannelConfig(&adc1);		
	ADC_WD_HTR_CFG(&adc1, ADC_WDH_ACT2);
	ADC_WD_LTR_CFG(&adc1, ADC_WDL_ACT2);
	ADC_Enable(&adc1);
	ADC_Start(&adc1);
	NVIC_IRQ_CONFIG(IRQ_NO_ADC, ENABLE);
}

/* Boczne silowniki */
void ADC_ACT3_Start()
{
	adc1.ADC_Init.Channel = 8;
	adc1.ADC_Init.SamplingTime = ADC_SMPR_480_CYC;
	ADC_ChannelConfig(&adc1);		
	ADC_WD_HTR_CFG(&adc1, ADC_WDH_ACT3);
	ADC_WD_LTR_CFG(&adc1, ADC_WDL_ACT3);
	ADC_Enable(&adc1);
	ADC_Start(&adc1);
	NVIC_IRQ_CONFIG(IRQ_NO_ADC, ENABLE);
}


/* IRQ handler and callback functions */
void EXTI4_IRQHandler()
{
	delay();
	if (Toolstatus == 0)
	{
		/* Zalaczenie silownika rozciagajacego uszczelki */
		GPIO_IRQHandling(BUTTON1_PIN);
		GPIO_IRQITConfig(IRQ_NO_EXTI4, DISABLE);
		TIM_PWM_Stop(&timer3);
		ADC_ACT1_Start();
		Toolstatus = 1;
	}
}

void EXTI15_10_IRQHandler()
{
	delay();
	if (Toolstatus == 2)
	{
		GPIO_IRQHandling(LIMIT_SW_PIN);
		GPIO_IRQITConfig(IRQ_NO_EXTI15_10, DISABLE);
		TIM_PWM_Stop(&timer3);
		ADC_ACT1_Start();
		Toolstatus = 3;
	}
}

void TIM2_IRQHandler()
{
	TIM_IRQHandler(&timer2);
}

void ADC_IRQHandler()
{
	ADC_IRQHandling(&adc1);
	/* dopisac implementacje funckji callback i zintegrowac z callbackiem z TIM2 */
}

void ADC_LevelOutOfWindowCallback(ADC_Handle_t *pADC_Instance)
{
	switch(Toolstatus)
	{
		case 1:
			ADC_STOP(&adc1);
			stepper_run(CW);
			Toolstatus = 2;
			break;
		case 3:
			/* Zalaczenie PWM silownikow dociskajacych uszczelke od dolu (timer 5) */
			ADC_STOP(&adc1);
			TIM_PWM_Start(&timer5);
			ADC_ACT3_Start();
			Toolstatus = 4;
			break;
		case 4:
			/* Wylaczenie PWM duzego silownika (timer 4) */
			ADC_STOP(&adc1);
			TIM_PWM_Stop(&timer4);
			ADC_ACT2_Start();
			Toolstatus = 5;
			break;
		case 5:
			ADC_STOP(&adc1);
			TIM_PWM_Stop(&timer5);
			TIM_PWM_Start(&timer3);
			ADC_ACT1_Start();
			stepper_run(CCW);
			Toolstatus = 6;
			break;
	}
}

void TIM_PeriodElapsedCallback(TIM_Handle_t *timer)
{
	switch (Toolstatus)
	{
		case 2:
			/* Zalaczenie PWM duzego silownika (timer 4) */
			delay();
			if (timer->pTIMx == TIM2)
			{
				stepper_stop();
				TIM_PWM_Start(&timer4);
				ADC_ACT2_Start();
				Toolstatus = 3;
			}
			break;
		case 6:
			if (timer->pTIMx == TIM2)
			{
				stepper_stop();
				ADC_STOP(&adc1);
				ADC_DISABLE(&adc1);
				NVIC_IRQ_CONFIG(IRQ_NO_ADC, DISABLE);
				GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);
				Toolstatus = 0;
			}
			break;
	}
}