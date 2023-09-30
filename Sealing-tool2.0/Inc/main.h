/*
 * main.h
 *
 *  Created on: 22 sie 2022
 *      Author: hj61t7
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx_timer_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_adc_driver.h"
#include "stm32f446xx_usart_driver.h"

#define TIM6_PERIOD 			64000
#define TIM6_PRESCALER_VALUE 	2499
#define TIM6_IRQ_NO				54

#define TIM1_PERIOD 			6400
#define TIM1_PRESCALER_VALUE 	31
#define TIM1_IRQ_NO				25
#define TIM1_DUTY_CYCLE			50
#define TIM1_PULSE				((TIM1_PERIOD)*(TIM1_DUTY_CYCLE)/100)

#define TIM2_PERIOD 			0
#define TIM2_PRESCALER_VALUE 	9999
#define TIM2_IRQ_NO				28
#define TIM2_DUTY_CYCLE			100
#define TIM2_PULSE				((TIM2_PERIOD)*(TIM2_DUTY_CYCLE)/100)

#define TIM3_PERIOD 			8000
#define TIM3_PRESCALER_VALUE 	1
#define TIM3_IRQ_NO				29
#define TIM3_DUTY_CYCLE			30
#define TIM3_PULSE				((TIM3_PERIOD)*(TIM3_DUTY_CYCLE)/100)

#define TIM4_PERIOD 			8000
#define TIM4_PRESCALER_VALUE 	1
#define TIM4_IRQ_NO				30
#define TIM4_DUTY_CYCLE			50
#define TIM4_PULSE				((TIM4_PERIOD)*(TIM4_DUTY_CYCLE)/100)

#define TIM5_PERIOD 			8000
#define TIM5_PRESCALER_VALUE 	1
#define TIM5_IRQ_NO				50
#define TIM5_DUTY_CYCLE			100
#define TIM5_PULSE				((TIM5_PERIOD)*(TIM5_DUTY_CYCLE)/100)

#define ADC_WDH_ACT1            3800
#define ADC_WDL_ACT1            2000

#define ADC_WDH_ACT2            4200
#define ADC_WDL_ACT2            1000

#define ADC_WDH_ACT3            4000
#define ADC_WDL_ACT3            500

#define STEP_PIN				GPIO_PIN_NO_8   // PA8
#define DIR_PIN					GPIO_PIN_NO_10  // PC10 
#define RESET_PIN				GPIO_PIN_NO_11  // PC11
#define SLEEP_PIN				GPIO_PIN_NO_12  // PC12
#define M0_PIN                  GPIO_PIN_NO_5   // PC5
#define M1_PIN					GPIO_PIN_NO_4   // PD2
#define M2_PIN					GPIO_PIN_NO_5   // PB5

#define BUTTON1_PIN				GPIO_PIN_NO_4   // PC4
#define LIMIT_SW_PIN            GPIO_PIN_NO_15   // PA15

#define ACTUATOR1_PIN			GPIO_PIN_NO_6   // PB6
#define ACTUATOR2_PIN			GPIO_PIN_NO_6   // PC6
#define ACTUATOR3_PIN			GPIO_PIN_NO_0   // PA0

#define ADC_ACT1_PIN            GPIO_PIN_NO_1   //PA1
#define ADC_ACT2_PIN            GPIO_PIN_NO_4   //PA4
#define ADC_ACT3_PIN            GPIO_PIN_NO_0   //PB0

#define STEP_ANGLE				1.8f
#define STEPS_PER_REV			(360/(STEP_ANGLE))
#define MICROSTEPS				8
#define CW						ENABLE
#define CCW						DISABLE

void USART2_Init();
void GPIO_button_Init();
void stepper_init();
void drv8825_activate();
void GPIOactuator_Init();
void TIMER1_Init();
void TIMER2_Init();
void TIMER3_Init();
void TIMER4_Init();
void TIMER5_Init();
void stepper_run(uint8_t dir);
void ADC_ACT1_Start();
void ADC_Meas_Init();
void Limit_SwitchInit();
void middle_act_Init();
int _write(int file, char *ptr, int len);
void Device_Init();

#endif /* MAIN_H_ */
