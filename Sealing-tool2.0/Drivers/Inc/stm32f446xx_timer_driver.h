/*
 * stm32f446xx_timer_driver.h
 *
 *  Created on: 18 sie 2022
 *      Author: hj61t7
 */

#include "stm32f446xx.h"

#ifndef INC_STM32F446XX_TIMER_DRIVER_H_
#define INC_STM32F446XX_TIMER_DRIVER_H_

typedef struct
{
	uint16_t Prescaler;
	uint32_t CounterMode;
	uint32_t Autoreload;
	uint32_t ClockDivision;
	uint8_t RepetitionCounter;
} TIM_Base_Init_t;

typedef struct
{
	TIM_RegDef_t *pTIMx;
	TIM_Base_Init_t TIM_Config;
} TIM_Handle_t;

typedef struct
{
	uint32_t OCMode;
	uint32_t Pulse;
	uint32_t OCPolarity;
	uint32_t OCNPolarity;
	uint32_t OCFastMode;
	uint32_t OCIdleState;
	uint32_t OCNIdleState;
} TIM_OC_Init_t;

typedef struct
{
	uint32_t SlaveModeSel;
	uint32_t InputTriggerSel;
	uint32_t MSmode;
	uint32_t ExternalTriggerFilter;
	uint32_t ExternalTriggerPrc;
	uint32_t ExternalClockEn;
	uint32_t ExternalTriggerPol;
} TIM_SlaveConfig_t;

typedef struct
{
	uint32_t MasterOutputTrigger;
	uint32_t MasterSlaveMode;
	uint32_t MasterTriggerSel;
} TIM_MasterConfig_t;



typedef enum
{
	tim1, tim2, tim3, tim4, tim5, tim6, tim7, tim8, tim9, tim10, tim11, tim12, tim13, tim14
} TIM_Instance;

typedef struct
{
	__vo uint32_t *TIM_reg_rcc;
	uint8_t RCCenr_index;
} TIMx_Instance_t;

void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t ENorDI);
void TIM_BaseInit(TIM_Handle_t *timer, uint8_t ENorDI, uint8_t irq_no);
void TIM_WaitForUIFSet(TIM_RegDef_t *pTIMx);
void TIM_BaseStartIT(TIM_Handle_t *timer, uint8_t irq_no);
void TIM_BaseStartITSlave(TIM_Handle_t *timer, uint8_t irq_no);
void TIM_BaseStopIT(TIM_Handle_t *timer);
void TIM_IRQHandler(TIM_Handle_t *timer);
void TIM_ClearStatusReg(TIM_RegDef_t *pTIMx);
void TIM_PWM_Config(TIM_Handle_t *timer, TIM_OC_Init_t *timConfig);
void TIM_PWM_Start(TIM_Handle_t *timer);
void TIM_PWM_Stop(TIM_Handle_t *timer);
void TIM_SlaveConfig(TIM_Handle_t *timer, TIM_SlaveConfig_t *timSlave);
void TIM_MasterConfig(TIM_Handle_t *timer, TIM_MasterConfig_t *timMaster);
__attribute__((weak)) void TIM_PeriodElapsedCallback(TIM_Handle_t *pTIMx);
__attribute__((weak)) void TIM_CaptureCallback(TIM_Handle_t *pTIMx);
__attribute__((weak)) void OC_DelayElapsedCallback(TIM_Handle_t *pTIMx);
__attribute__((weak)) void PWM_PulseFinishedCallback(TIM_Handle_t *pTIMx);
__attribute__((weak)) void TIM_CommutationCallback(TIM_Handle_t *pTIMx);
__attribute__((weak)) void TIM_TriggerCallback(TIM_Handle_t *pTIMx);
__attribute__((weak)) void TIM_BreakCallback(TIM_Handle_t *pTIMx);

/*
 * TIM Control register 1 (TIMx_CR1) bit definitions
 */

/*
 * @TIM_CounterEnable
 */

#define TIM_CEN_POS						0U
#define TIM_CEN_DI						DISABLE
#define TIM_CEN_EN						ENABLE

/*
 * @TIM_UpdateDisable
 */

#define TIM_UDIS_POS					1U
#define TIM_UDIS_EN						0
#define TIM_UDIS_DI						1

/*
 * @TIM_UpdateRequestSource
 */

#define TIM_URS_POS						2U
#define TIM_URS_ALL						0
#define TIM_URS_CNT						1

/*
 * @TIM_OnePulseMode
 */

#define TIM_OPM_POS						3U
#define TIM_OPM_DI						0
#define TIM_OPM_EN						1

/*
 * @TIM_Direction
 */

#define TIM_DIR_POS						4U
#define TIM_DIR_UP						0
#define TIM_DIR_DW						1

/*
 * @TIM_Center-alignedModeSelection
 */

#define TIM_CMS_POS						5U
#define TIM_CMS_EDGE_MODE				0
#define TIM_CMS_CENTER_MODE1			1
#define TIM_CMS_CENTER_MODE2			2
#define TIM_CMS_CENTER_MODE3			3

/*
 * @TIM_Auto-reloadPreloadEnable
 */

#define TIM_ARPE_POS					7U
#define TIM_ARPE_DI						0
#define TIM_ARPE_EN						1

/*
 * @TIM_ClockDivision
 */

#define TIM_CKDIV_POS					8U
#define TIM_CKDIV_1						0
#define TIM_CKDIV_2						1
#define TIM_CKDIV_3						2

/*
 * TIM Control register 2 (TIMx_CR2) bit definitions
 */

/*
 * @TIM_Capture/comparePreload Capture/compare DMA selection
 */

#define TIM_CCPC_POS					0U
#define TIM_CCPC_DI						0
#define TIM_CCPC_EN						1

/*
 * @TIM_Capture/compareUpdate Capture/compare control update selection
 */

#define TIM_CCUS_POS					2U
#define TIM_CCUS_COMG					0
#define TIM_CCUS_COMG_RE				1

/*
 * @TIM_Capture/compareDMA Capture/compare DMA selection
 */

#define TIM_CCDS_POS					3U
#define TIM_CCDS_CCX					0
#define TIM_CCDS_UPD					1

/*
 * @TIM_MasterMode Master mode selection
 */

#define TIM_MMS_POS						4U
#define TIM_MMS_RST						0
#define TIM_MMS_EN						1
#define TIM_MMS_UPD						2
#define TIM_MMS_CMP						3
#define TIM_MMS_CMP_OC1REF				4
#define TIM_MMS_CMP_OC2REF				5
#define TIM_MMS_CMP_OC3REF				6
#define TIM_MMS_CMP_OC4REF				7

/*
 * @TIM_TI1 selection
 */

#define TIM_TI1S_POS					7U
#define TIM_TI1S_CH1					0
#define TIM_TI1S_XOR					1

/*
 * @TIM_OutputIdle1 state 1 (OC1 output)
 */

#define TIM_OIS1_POS					8U
#define TIM_OIS1_RST					0
#define TIM_OIS1_SET					1

/*
 * @TIM_OutputIdle1 state 1 (OC1N output)
 */

#define TIM_OIS1N_POS					9U
#define TIM_OIS1N_RST					0
#define TIM_OIS1N_SET					1

/*
 * @TIM_OutputIdle2 state 2 (OC2 output)
 */

#define TIM_OIS2_POS					10U
#define TIM_OIS2_RST					0
#define TIM_OIS2_SET					1

/*
 * @TIM_OutputIdle1 state 2 (OC2N output)
 */

#define TIM_OIS2N_POS					11U
#define TIM_OIS2N_RST					0
#define TIM_OIS2N_SET					1

/*
 * @TIM_OutputIdle1 state 3 (OC3 output)
 */

#define TIM_OIS3_POS					12U
#define TIM_OIS3_RST					0
#define TIM_OIS3_SET					1

/*
 * @TIM_OutputIdle1 state 3 (OC3N output)
 */

#define TIM_OIS3N_POS					13U
#define TIM_OIS3N_RST					0
#define TIM_OIS3N_SET					1

/*
 * @TIM_OutputIdle1 state 4 (OC4 output)
 */

#define TIM_OIS4_POS					14U
#define TIM_OIS4_RST					0
#define TIM_OIS4_SET					1

/*
 * TIM Slave mode control register (TIMx_SMCR) bit definitions
 */

/*
 * @TIM_SlaveMode selection
 */

#define TIM_SMS_POS						0U
#define TIM_SMS_DI						0
#define TIM_SMS_ENCODER_MODE_1			1
#define TIM_SMS_ENCODER_MODE_2			2
#define TIM_SMS_ENCODER_MODE_3			3
#define TIM_SMS_RESET_MODE				4
#define TIM_SMS_GATED_MODE				5
#define TIM_SMS_TRIGGER_MODE			6
#define TIM_SMS_EXT_CLK					7

/*
 * @TIM_Trigger selection
 */

#define TIM_TS_POS						4U
#define TIM_ITR0						0
#define TIM_ITR1						1
#define TIM_ITR2						2
#define TIM_ITR3						3
#define TIM_TI1F_ED						4
#define TIM_TI1FP1						5
#define TIM_TI1FP2						6
#define TIM_ETRF						7

/*
 * @TIM_MSM Master/slave mode
 */

#define TIM_MSM_POS						7U
#define TIM_MSM_OFF						0
#define TIM_MSM_ON						1

/*
 * @TIM_ETF External trigger filter
 */

#define TIM_ETF_POS						8U
#define TIM_ETF_OFF						0
#define TIM_ETF_FCK_N2					1
#define TIM_ETF_FCK_N4					2
#define TIM_ETF_FCK_N8					3
#define TIM_ETF_FDTS_DIV2_N6			4
#define TIM_ETF_FDTS_DIV2_N8			5
#define TIM_ETF_FDTS_DIV4_N6			6
#define TIM_ETF_FDTS_DIV4_N8			7
#define TIM_ETF_FDTS_DIV8_N6			8
#define TIM_ETF_FDTS_DIV8_N8			9
#define TIM_ETF_FDTS_DIV16_N5			10
#define TIM_ETF_FDTS_DIV16_N6			11
#define TIM_ETF_FDTS_DIV16_N8			12
#define TIM_ETF_FDTS_DIV32_N5			13
#define TIM_ETF_FDTS_DIV32_N6			14
#define TIM_ETF_FDTS_DIV32_N8			15

/*
 * @TIM_ETPS External trigger prescaler
 */

#define TIM_ETPS_POS					12U
#define TIM_ETPS_OFF					0
#define TIM_ETPS_DIV2					1
#define TIM_ETPS_DIV4					2
#define TIM_ETPS_DIV8					3

/*
 * @TIM_ECE: External clock enable - external clock mode 2 enabling
 */

#define TIM_ECE_POS						14U
#define TIM_ECE_DI						0
#define TIM_ECE_EN						1

/*
 * @TIM_ETP: External trigger polarity
 */

#define TIM_ETP_POS						15U
#define TIM_ETP_NON_INV					0
#define TIM_ETP_EN_INV					1

/*
 * TIM DMA/interrupt enable register (TIMx_DIER) bit definition
 *
 */

/*
 * @TIM_UIE: Update interrupt enable
 */

#define TIM_UIE_POS						0UL
#define TIM_UIE_DI						0
#define TIM_UIE_EN						1

/*
 * @TIM_CC1IE: Capture/Compare 1 interrupt enable
 */

#define TIM_CC1IE_POS					1UL
#define TIM_CC1IE_DI					0
#define TIM_CC1IE_EN					1

/*
 * @TIM_CC2IE: Capture/Compare 2 interrupt enable
 */

#define TIM_CC2IE_POS					2UL
#define TIM_CC2IE_DI					0
#define TIM_CC2IE_EN					1

/*
 * @TIM_CC3IE: Capture/Compare 3 interrupt enable
 */

#define TIM_CC3IE_POS					3UL
#define TIM_CC3IE_DI					0
#define TIM_CC3IE_EN					1

/*
 * @TIM_CC4IE: Capture/Compare 4 interrupt enable
 */

#define TIM_CC4IE_POS					4UL
#define TIM_CC4IE_DI					0
#define TIM_CC4IE_EN					1

/*
 * @TIM_COMIE: COM interrupt enable
 */

#define TIM_COMIE_POS					5UL
#define TIM_COMIE_DI					0
#define TIM_COMIE_EN					1

/*
 * @TIM_TIE: Trigger interrupt enable
 */

#define TIM_TIE_POS						6UL
#define TIM_TIE_DI						0
#define TIM_TIE_EN						1

/*
 * @TIM_BIE: Break interrupt enable
 */

#define TIM_BIE_POS						7UL
#define TIM_BIE_DI						0
#define TIM_BIE_EN						1

/*
 * @TIM_UDE: Update DMA request enable
 */

#define TIM_UDE_POS						8UL
#define TIM_UDE_DI						0
#define TIM_UDE_EN						1

/*
 * @TIM_CC1DE: Capture/Compare 1 DMA request enable
 */

#define TIM_CC1DE_POS					9UL
#define TIM_CC1DE_DI					0
#define TIM_CC1DE_EN					1

/*
 * @TIM_CC2DE: Capture/Compare 2 DMA request enable
 */

#define TIM_CC2DE_POS					10UL
#define TIM_CC2DE_DI					0
#define TIM_CC2DE_EN					1

/*
 * @TIM_CC3DE: Capture/Compare 3 DMA request enable
 */

#define TIM_CC3DE_POS					11UL
#define TIM_CC3DE_DI					0
#define TIM_CC3DE_EN					1

/*
 * @TIM_CC4DE: Capture/Compare 4 DMA request enable
 */

#define TIM_CC4DE						12UL
#define TIM_CC4DE_DI					0
#define TIM_CC4DE_EN					1

/*
 * @TIM_COMDE: COM DMA request enable
 */

#define TIM_COMDE_POS					13UL
#define TIM_COMDE_DI					0
#define TIM_COMDE_EN					1

/*
 * @TIM_TDE: Trigger DMA request enable
 */

#define TIM_TDE_POS						14UL
#define TIM_TDE_DI						0
#define TIM_TDE_EN						1

/*
 * TIM status register (TIMx_SR) bit definitions
 */

/*
 * @TIM_UIF: Update interrupt flag
 */

#define TIM_UIF_POS						0U
#define TIM_UIF_NO_UPDATE				0
#define TIM_UIF_UPDATE_INT_PEND			1

/*
 * @TIM_CC1IF: Capture/Compare 1 interrupt flag
 */

#define TIM_CC1IF_POS					1U
#define TIM_CC1IF_NO_MATCH				0
#define TIM_CC1IF_MATCH					1

/*
 * @TIM_CC2IF: Capture/Compare 2 interrupt flag
 */

#define TIM_CC2IF_POS					2U
#define TIM_CC2IF_NO_MATCH				0
#define TIM_CC2IF_MATCH					1

/*
 * @TIM_CC3IF: Capture/Compare 3 interrupt flag
 */

#define TIM_CC3IF_POS					3U
#define TIM_CC3IF_NO_MATCH				0
#define TIM_CC3IF_MATCH					1

/*
 * @TIM_CC4IF: Capture/Compare 4 interrupt flag
 */

#define TIM_CC4IF_POS					4U
#define TIM_CC4IF_NO_MATCH				0
#define TIM_CC4IF_MATCH					1

/*
 * @TIM_COMIF: COM interrupt flag
 */

#define TIM_COMIF_POS					5U
#define TIM_COMIF_NO_EVENT				0
#define TIM_COMIF_PENDING				1

/*
 * @TIM_TIF: Trigger interrupt flag
 */

#define TIM_TIF_POS						6U
#define TIM_TIF_NO_EVENT				0
#define TIM_TIF_PENDING					1

/*
 * @TIM_BIF: Break interrupt flag
 */

#define TIM_BIF_POS						7U
#define TIM_BIF_NOT_DETECTED			0
#define TIM_BIF_DETECTED				1

/*
 * @TIM_CC1OF: Capture/Compare 1 overcapture flag
 */

#define TIM_CC1OF_NOT_DETECTED			0
#define TIM_CC1OF_DETECTED				1

/*
 * @TIM_CC2OF: Capture/Compare 2 overcapture flag
 */

#define TIM_CC2OF_NOT_DETECTED			0
#define TIM_CC2OF_DETECTED				1

/*
 * @TIM_CC3OF: Capture/Compare 3 overcapture flag
 */

#define TIM_CC3OF_NOT_DETECTED			0
#define TIM_CC3OF_DETECTED				1


/*
 * @TIM_CC4OF: Capture/Compare 4 overcapture flag
 */

#define TIM_CC4OF_NOT_DETECTED			0
#define TIM_CC4OF_DETECTED				1

/*
 * TIM event generation register (TIMx_EGR) bit definitions
 */

/*
 * @TIM_UG: Update generation
 */

#define TIM_UG_POS						0U
#define TIM_UG_NO_ACTION				0
#define TIM_UG_UPDATE					1

/*
 * @TIM_CC1G: Capture/Compare 1 generation
 */

#define TIM_CC1G_NO_ACTION				0
#define TIM_CC1G_GENERATION				1

/*
 * @TIM_CC2G: Capture/Compare 2 generation
 */

#define TIM_CC2G_NO_ACTION				0
#define TIM_CC2G_GENERATION				1

/*
 * @TIM_CC3G: Capture/Compare 3 generation
 */

#define TIM_CC3G_NO_ACTION				0
#define TIM_CC3G_GENERATION				1

/*
 * @TIM_CC4G: Capture/Compare 4 generation
 */

#define TIM_CC4G_NO_ACTION				0
#define TIM_CC4G_GENERATION				1

/*
 * @TIM_COMG: Capture/Compare control update generation
 */

#define TIM_COMG_NO_ACTION				0
#define TIM_COMG_GENERATION				1

/*
 * @TIM_TG: Trigger generation
 */

#define TIM_TG_NO_ACTION				0
#define TIM_TG_SET_TIF					1

/*
 * @TIM_BG: Trigger generation
 */

#define TIM_BG_NO_ACTION				0
#define TIM_BG_BREAK_EV_GEN				1

/*
 * TIM capture/compare mode register 1 (TIMx_CCMR1) bit definitions
 * Output compare mode:
 */

/*
 * @TIM_CC1S: Capture/Compare 1 selection
 */

#define TIM_CC1S_POS					0U
#define TIM_CC1S_OUTPUT					0
#define TIM_CC1S_INPUT_TI1				1
#define TIM_CC1S_INPUT_TI2				2
#define TIM_CC1S_INPUT_TRC				3

/*
 * @TIM_OC1FE: Output Compare 1 fast enable
 */

#define TIM_OC1FE_POS					2U
#define TIM_OC1FE_DI					0
#define TIM_OC1FE_EN					1

/*
 * @TIM_OC1PE: Output Compare 1 preload enable
 */

#define TIM_OC1PE_POS					3U
#define TIM_OC1PE_DI					0
#define TIM_OC1PE_EN					1

/*
 * @TIM_OC1M: Output Compare 1 mode
 */

#define TIM_OC1M_POS					4U
#define TIM_OC1M_FROZEN					0
#define TIM_OC1M_SET_CH1_HIGH			1
#define TIM_OC1M_SET_CH1_LOW			2
#define TIM_OC1M_TOGGLE					3
#define TIM_OC1M_FORCE_HIGH				4
#define TIM_OC1M_FORCE_LOW				5
#define TIM_OC1M_PWM_MODE1				6
#define TIM_OC1M_PWM_MODE2				7

/*
 * @TIM_OC1CE: Output Compare 1 clear enable
 */

#define TIM_OC1CE_POS					7U
#define TIM_OC1CE_DI					0
#define TIM_OC1CE_EN					1

/*
 * @TIM_CC2S: Capture/Compare 2 selection
 */

#define TIM_CC2S_POS					8U
#define TIM_CC2S_OUTPUT					0
#define TIM_CC2S_INPUT_TI2				1
#define TIM_CC2S_INPUT_TI1				2
#define TIM_CC2S_INPUT_TRC				3

/*
 * @TIM_OC2FE: Capture/Compare 2 fast enable
 */

#define TIM_OC2FE_POS					10U
#define TIM_OC2FE_DI					0
#define TIM_OC2FE_EN					1

/*
 * @TIM_OC2PE: Output Compare 2 preload enable
 */

#define TIM_OC2PE_POS					11U
#define TIM_OC2PE_DI					0
#define TIM_OC2PE_EN					1

/*
 * @TIM_OC2M: Output Compare 2 mode
 */

#define TIM_OC2M_POS					12U
#define TIM_OC2M_FROZEN					0
#define TIM_OC2M_SET_CH1_HIGH			1
#define TIM_OC2M_SET_CH1_LOW			2
#define TIM_OC2M_TOGGLE					3
#define TIM_OC2M_FORCE_HIGH				4
#define TIM_OC2M_FORCE_LOW				5
#define TIM_OC2M_PWM_MODE1				6
#define TIM_OC2M_PWM_MODE2				7

/*
 * @TIM_OC2CE: Output Compare 2 clear enable
 */

#define TIM_OC2CE_POS					15U
#define TIM_OC2FE_DI					0
#define TIM_OC2FE_EN					1

/*
 * TIM capture/compare mode register 1 (TIMx_CCMR1) bit definitions
 * Input compare mode:
 */

/*
 * @TIM_IC1PSC: Input capture 1 prescaler
 */

#define TIM_IC1PSC_POS					2U
#define TIM_IC1PSC_NO_PRESCALER			0
#define TIM_IC1PSC_2_EVENTS				1
#define TIM_IC1PSC_4_EVENTS				2
#define TIM_IC1PSC_8_EVENTS				3

/*
 * @TIM_IC1F: Input capture 1 filter
 */

#define TIM_IC1F_POS					4U
#define TIM_IC1F_OFF					0
#define TIM_IC1F_FCK_N2					1
#define TIM_IC1F_FCK_N4					2
#define TIM_IC1F_FCK_N8					3
#define TIM_IC1F_FDTS_DIV2_N6			4
#define TIM_IC1F_FDTS_DIV2_N8			5
#define TIM_IC1F_FDTS_DIV4_N6			6
#define TIM_IC1F_FDTS_DIV4_N8			7
#define TIM_IC1F_FDTS_DIV8_N6			8
#define TIM_IC1F_FDTS_DIV8_N8			9
#define TIM_IC1F_FDTS_DIV16_N5			10
#define TIM_IC1F_FDTS_DIV16_N6			11
#define TIM_IC1F_FDTS_DIV16_N8			12
#define TIM_IC1F_FDTS_DIV32_N5			13
#define TIM_IC1F_FDTS_DIV32_N6			14
#define TIM_IC1F_FDTS_DIV32_N8			15

/*
 * @TIM_IC2PSC: Input capture 2 prescaler
 */

#define TIM_IC2PSC_POS					10U
#define TIM_IC2PSC_NO_PRESCALER			0
#define TIM_IC2PSC_2_EVENTS				1
#define TIM_IC2PSC_4_EVENTS				2
#define TIM_IC2PSC_8_EVENTS				3

/*
 * @TIM_IC2F: Input capture 2 filter
 */

#define TIM_IC2F_POS					12U
#define TIM_IC2F_OFF					0
#define TIM_IC2F_FCK_N2					1
#define TIM_IC2F_FCK_N4					2
#define TIM_IC2F_FCK_N8					3
#define TIM_IC2F_FDTS_DIV2_N6			4
#define TIM_IC2F_FDTS_DIV2_N8			5
#define TIM_IC2F_FDTS_DIV4_N6			6
#define TIM_IC2F_FDTS_DIV4_N8			7
#define TIM_IC2F_FDTS_DIV8_N6			8
#define TIM_IC2F_FDTS_DIV8_N8			9
#define TIM_IC2F_FDTS_DIV16_N5			10
#define TIM_IC2F_FDTS_DIV16_N6			11
#define TIM_IC2F_FDTS_DIV16_N8			12
#define TIM_IC2F_FDTS_DIV32_N5			13
#define TIM_IC2F_FDTS_DIV32_N6			14
#define TIM_IC2F_FDTS_DIV32_N8			15

/*
 * TIM capture/compare mode register 2 (TIMx_CCMR2) bit definitions
 * Output compare mode:
 */

/*
 * @TIM_CC3S: Capture/Compare 3 selection
 */
#define TIM_CC3S_POS					0U
#define TIM_CC3S_OUTPUT					0
#define TIM_CC3S_INPUT_TI3				1
#define TIM_CC3S_INPUT_TI4				2
#define TIM_CC3S_INPUT_TRC				3

/*
 * @TIM_OC3FE: Output Compare 3 fast enable
 */

#define TIM_OC3FE_DI					0
#define TIM_OC3FE_EN					1

/*
 * @TIM_OC3PE: Output Compare 3 preload enable
 */

#define TIM_OC3PE_DI					0
#define TIM_OC3PE_EN					1

/*
 * @TIM_OC3M: Output Compare 3 mode
 */

#define TIM_OC3M_FROZEN					0
#define TIM_OC3M_SET_CH1_HIGH			1
#define TIM_OC3M_SET_CH1_LOW			2
#define TIM_OC3M_TOGGLE					3
#define TIM_OC3M_FORCE_HIGH				4
#define TIM_OC3M_FORCE_LOW				5
#define TIM_OC3M_PWM_MODE1				6
#define TIM_OC3M_PWM_MODE2				7

/*
 * @TIM_OC3CE: Output Compare 3 clear enable
 */

#define TIM_OC3CE_DI					0
#define TIM_OC3CE_EN					1

/*
 * @TIM_CC4S: Capture/Compare 4 selection
 */
#define TIM_CC4S_POS					0
#define TIM_CC4S_OUTPUT					0
#define TIM_CC4S_INPUT_TI2				1
#define TIM_CC4S_INPUT_TI1				2
#define TIM_CC4S_INPUT_TRC				3

/*
 * @TIM_OC4FE: Capture/Compare 4 fast enable
 */

#define TIM_OC4FE_DI					0
#define TIM_OC4FE_EN					1

/*
 * @TIM_OC4PE: Output Compare 4 preload enable
 */

#define TIM_OC4PE_DI					0
#define TIM_OC4PE_EN					1

/*
 * @TIM_OC4M: Output Compare 4 mode
 */

#define TIM_OC4M_FROZEN					0
#define TIM_OC4M_SET_CH1_HIGH			1
#define TIM_OC4M_SET_CH1_LOW			2
#define TIM_OC4M_TOGGLE					3
#define TIM_OC4M_FORCE_HIGH				4
#define TIM_OC4M_FORCE_LOW				5
#define TIM_OC4M_PWM_MODE1				6
#define TIM_OC4M_PWM_MODE2				7

/*
 * @TIM_OC4CE: Output Compare 4 clear enable
 */

#define TIM_OC4FE_DI					0
#define TIM_OC4FE_EN					1

/*
 * TIM capture/compare mode register 3 (TIMx_CCMR3) bit definitions
 * Input compare mode:
 */

/*
 * @TIM_IC3PSC: Input capture 3 prescaler
 */

#define TIM_IC3PSC_NO_PRESCALER			0
#define TIM_IC3PSC_2_EVENTS				1
#define TIM_IC3PSC_4_EVENTS				2
#define TIM_IC3PSC_8_EVENTS				3

/*
 * @TIM_IC3F: Input capture 3 filter
 */

#define TIM_IC3F_OFF					0
#define TIM_IC3F_FCK_N2					1
#define TIM_IC3F_FCK_N4					2
#define TIM_IC3F_FCK_N8					3
#define TIM_IC3F_FDTS_DIV2_N6			4
#define TIM_IC3F_FDTS_DIV2_N8			5
#define TIM_IC3F_FDTS_DIV4_N6			6
#define TIM_IC3F_FDTS_DIV4_N8			7
#define TIM_IC3F_FDTS_DIV8_N6			8
#define TIM_IC3F_FDTS_DIV8_N8			9
#define TIM_IC3F_FDTS_DIV16_N5			10
#define TIM_IC3F_FDTS_DIV16_N6			11
#define TIM_IC3F_FDTS_DIV16_N8			12
#define TIM_IC3F_FDTS_DIV32_N5			13
#define TIM_IC3F_FDTS_DIV32_N6			14
#define TIM_IC3F_FDTS_DIV32_N8			15

/*
 * @TIM_IC4PSC: Input capture 4 prescaler
 */

#define TIM_IC4PSC_NO_PRESCALER			0
#define TIM_IC4PSC_2_EVENTS				1
#define TIM_IC4PSC_4_EVENTS				2
#define TIM_IC4PSC_8_EVENTS				3

/*
 * @TIM_IC4F: Input capture 4 filter
 */

#define TIM_IC4F_OFF					0
#define TIM_IC4F_FCK_N2					1
#define TIM_IC4F_FCK_N4					2
#define TIM_IC4F_FCK_N8					3
#define TIM_IC4F_FDTS_DIV2_N6			4
#define TIM_IC4F_FDTS_DIV2_N8			5
#define TIM_IC4F_FDTS_DIV4_N6			6
#define TIM_IC4F_FDTS_DIV4_N8			7
#define TIM_IC4F_FDTS_DIV8_N6			8
#define TIM_IC4F_FDTS_DIV8_N8			9
#define TIM_IC4F_FDTS_DIV16_N5			10
#define TIM_IC4F_FDTS_DIV16_N6			11
#define TIM_IC4F_FDTS_DIV16_N8			12
#define TIM_IC4F_FDTS_DIV32_N5			13
#define TIM_IC4F_FDTS_DIV32_N6			14
#define TIM_IC4F_FDTS_DIV32_N8			15

/*
 * TIM capture/compare enable register (TIMx_CCER)
 */

/*
 * @TIM_CC1E: Capture/Compare 1 output enable (CC1 as input and output)
 */

#define TIM_CC1E_POS					0U
#define TIM_CC1E_DI						0
#define TIM_CC1E_EN						1

/*
 * @TIM_CC1P: Capture/Compare 1 output polarity (CC1 as output)
 */

#define TIM_CC1P_POS					1U
#define TIM_CC1P_ACT_HIGH				0
#define TIM_CC1P_ACT_LOW				1

/*
 * @TIM_CC1P: Capture/Compare 1 output polarity (CC1 as input - CC1NP/CC1P bits)
 */

#define TIM_CC1P_POS					1U
#define TIM_CC1P_NON_INV_RE				0
#define TIM_CC1P_INV_FE					1
#define TIM_CC1P_NON_INV_RFE			3

/*
 * @TIM_CC1NE: Capture/Compare 1 complementary output enable
 */

#define TIM_CC1NE_POS					2U
#define TIM_CC1NE_DI					0
#define TIM_CC1NE_EN					1

/*
 * @TIM_CC1NP: Capture/Compare 1 complementary output polarity
 */

#define TIM_CC1NP_POS					3U
#define TIM_CC1NP_ACT_HIGH				0
#define TIM_CC1NP_ACT_LOW				1

/*
 * @TIM_CC2E: Capture/Compare 2 output enable
 */

#define TIM_CC2E_POS					4U
#define TIM_CC2E_DI						0
#define TIM_CC2E_EN						1

/*
 * @TIM_CC2P: Capture/Compare 2 output polarity (CC2 as output)
 */

#define TIM_CC2P_POS					5U
#define TIM_CC2P_ACT_HIGH				0
#define TIM_CC2P_ACT_LOW				1

/*
 * @TIM_CC2P: Capture/Compare 2 output polarity (CC2 as input - CC2NP/CC2P bits)
 */

#define TIM_CC2P_POS					5U
#define TIM_CC2P_NON_INV_RE				0
#define TIM_CC2P_INV_FE					1
#define TIM_CC2P_NON_INV_RFE			3

/*
 * @TIM_CC2NE: Capture/Compare 2 complementary output enable
 */

#define TIM_CC2NE_POS					6U
#define TIM_CC2NE_DI					0
#define TIM_CC2NE_EN					1

/*
 * @TIM_CC2NP: Capture/Compare 2 complementary output polarity
 */

#define TIM_CC2NP_POS					7U
#define TIM_CC2NP_ACT_HIGH				0
#define TIM_CC2NP_ACT_LOW				1

/*
 * @TIM_CC3E: Capture/Compare 3 output enable
 */

#define TIM_CC3E_POS					8U
#define TIM_CC3E_DI						0
#define TIM_CC3E_EN						1

/*
 * @TIM_CC3P: Capture/Compare 3 output polarity (CC3 as output)
 */

#define TIM_CC3P_POS					9U
#define TIM_CC3P_ACT_HIGH				0
#define TIM_CC3P_ACT_LOW				1

/*
 * @TIM_CC3P: Capture/Compare 3 output polarity (CC3 as input - CC3NP/CC3P bits)
 */

#define TIM_CC3P_POS					9U
#define TIM_CC3P_NON_INV_RE				0
#define TIM_CC3P_INV_FE					1
#define TIM_CC3P_NON_INV_RFE			3

/*
 * @TIM_CC3NE: Capture/Compare 3 complementary output enable
 */

#define TIM_CC3NE_POS					10U
#define TIM_CC3NE_DI					0
#define TIM_CC3NE_EN					1

/*
 * @TIM_CC3NP: Capture/Compare 3 complementary output polarity
 */

#define TIM_CC3NP_POS					11U
#define TIM_CC3NP_ACT_HIGH				0
#define TIM_CC3NP_ACT_LOW				1


/*
 * @TIM_CC4E: Capture/Compare 4 output enable
 */

#define TIM_CC4E_POS					12U
#define TIM_CC4E_DI						0
#define TIM_CC4E_EN						1

/*
 * @TIM_CC4P: Capture/Compare 4 output polarity (CC4 as output)
 */

#define TIM_CC4P_POS					13U
#define TIM_CC4P_ACT_HIGH				0
#define TIM_CC4P_ACT_LOW				1

/*
 * @TIM_BDTR: break and dead-time register (TIMx_BDTR)
 */

#define TIM_MOE_POS						15U

/*
 * Clear timer interrupt status register specific bit set by hardware after event
 */

#define TIM_CLEAR_IT(HANDLE, INTERRUPT)				((HANDLE)->pTIMx->SR = ~(1UL << INTERRUPT))
#define TIM_BASE_START(HANDLE)						((HANDLE)->pTIMx->CR1 |= (1UL << TIM_CEN_POS))
#define TIM_BASE_STOP(HANDLE)						((HANDLE)->pTIMx->CR1 &= ~(1UL << TIM_CEN_POS))
#define TIM_CLEAR_STATUS(HANDLE)					((HANDLE)->pTIMx->SR = 0)
#define TIM_STATUS_FLAG(HANDLE, FLAG)				(((HANDLE)->pTIMx->SR & (0x1UL << (FLAG))) == (0x1UL << (FLAG)))
#define TIM_IT_SOURCE_CHECK(HANDLE, INTERRUPT)		((((HANDLE)->pTIMx->DIER & (0x1UL << (INTERRUPT))) == (0x1UL << (INTERRUPT))) ? SET : RESET)
#define TIM_EGR_UG_SET(HANDLE)						(((HANDLE)->pTIMx->EGR) |= (0x1UL << TIM_UG_POS))


#endif /* INC_STM32F446XX_TIMER_DRIVER_H_ */
