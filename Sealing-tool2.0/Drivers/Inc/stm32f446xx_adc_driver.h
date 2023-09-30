

#ifndef INC_STM32F446XX_ADC_DRIVER_H_
#define INC_STM32F446XX_ADC_DRIVER_H_


#include "stm32f446xx.h"

typedef struct
{
    uint32_t ClockPrescaler;
    uint32_t Resolution;
    uint32_t DataAlign;
    uint32_t ScanConvMode;
    uint32_t ContConvMode;
    uint32_t EOCSelection;
    uint32_t Channel;
    uint32_t SamplingTime;
    uint32_t SeqLength;
} ADC_InitConfig_t;

typedef struct
{
    ADC_RegDef_t *pADCx;
    ADC_Common_RegDef_t *pADCC;
    ADC_InitConfig_t ADC_Init;
} ADC_Handle_t;

typedef struct
{
    uint32_t WatchdogMode;
    uint32_t HighThreshold;
    uint32_t LowThreshold;
    uint8_t ITmode;
} ADC_WDConfig_t;


void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t ENorDI);
void ADC_Init();
void ADC_Enable(ADC_Handle_t *pADC_Instance);
void ADC_Start(ADC_Handle_t *pADC_Instance);
void ADC_WaitForSingleConv(ADC_Handle_t *pADC_Instance);
uint16_t ADC_GetVal(ADC_Handle_t *pADC_Instance);
void ADC_AnalogWDConfig(ADC_Handle_t *pADC_Instance, ADC_WDConfig_t *AnalogWDConfig);
void ADC_ChannelConfig(ADC_Handle_t *ADC_Instance);
void ADC_IRQITConfig(uint8_t irq_no, uint8_t ENorDI);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void ADC_IRQHandling(ADC_Handle_t *pADC_Instance);
__attribute__((weak)) void ADC_ConvCpltCallback(ADC_Handle_t *pADC_Instance);
__attribute__((weak)) void InjectedConvCpltCallback(ADC_Handle_t *pADC_Instance);
__attribute__((weak)) void ADC_LevelOutOfWindowCallback(ADC_Handle_t *pADC_Instance);
__attribute__((weak)) void ADC_ErrorCallback(ADC_Handle_t *pADC_Instance);

/*
 * ADC status register (ADC_SR) bit definitions
 */

#define ADC_AWD_POS                         0U
#define ADC_EOC_POS                         1U
#define ADC_JEOC_POS                        2U
#define ADC_JSTRT_POS                       3U
#define ADC_STRT_POS                        4U
#define ADC_OVR_POS                         5U

/*
 * ADC control register 1 (ADC_CR1) bit definitions
 */

#define ADC_AWDCH_POS                       0U
#define ADC_EOCIE_POS                       5U
#define ADC_AWDIE_POS                       6U
#define ADC_JEOCIE_POS                      7U

#define ADC_SCAN_POS                        8U
#define ADC_SCAN_DI                         0UL
#define ADC_SCAN_EN                         1UL

#define ADC_AWDSGL_POS                      9U
#define ADC_JAWDEN_POS                      10U
#define ADC_AWDEN_POS                       23U

#define ADC_RES_POS                         24U
#define ADC_RES_12_BITS                     0U
#define ADC_RES_10_BITS                     1U
#define ADC_RES_8_BITS                      2U
#define ADC_RES_6_BITS                      3U

#define ADC_OVRIE_POS                       26U

/*
 * ADC control register 2 (ADC_CR2) bit definitions
 */

#define ADC_ADON_POS                        0U

#define ADC_CONT_POS                        1U
#define ADC_CONT_DIS                        0UL
#define ADC_CONT_EN                         1UL

#define ADC_ALIGN_POS                       11U
#define ADC_ALIGN_RIGHT                     0UL
#define ADC_ALIGN_LEFT                      1UL

#define ADC_EOCS_POS                        10U
#define ADC_EOCS_DI                         0UL
#define ADC_EOCS_EN                         1UL

#define ADC_SWSTART_POS                     30U

/*
 * ADC common control register (ADC_CCR) bit definitions
 */

#define ADC_ADCPRE_POS                      16U
#define ADC_ADCPRE_DIV2                     0U
#define ADC_ADCPRE_DIV4                     1U
#define ADC_ADCPRE_DIV6                     2U
#define ADC_ADCPRE_DIV8                     3U

#define ADC_SMPR_3_CYC                      0U
#define ADC_SMPR_15_CYC                     1U
#define ADC_SMPR_28_CYC                     2U
#define ADC_SMPR_56_CYC                     3U
#define ADC_SMPR_84_CYC                     4U
#define ADC_SMPR_112_CYC                    5U
#define ADC_SMPR_144_CYC                    6U
#define ADC_SMPR_480_CYC                    7U

#define ADC_SEQ_LENGTH_POS                  20U
#define ADC_SEQ_LENGTH_1                    0U
#define ADC_SEQ_LENGTH_2                    1U
#define ADC_SEQ_LENGTH_3                    2U
#define ADC_SEQ_LENGTH_4                    3U
#define ADC_SEQ_LENGTH_5                    4U
#define ADC_SEQ_LENGTH_6                    5U
#define ADC_SEQ_LENGTH_7                    6U
#define ADC_SEQ_LENGTH_8                    7U
#define ADC_SEQ_LENGTH_9                    8U
#define ADC_SEQ_LENGTH_10                   9U
#define ADC_SEQ_LENGTH_11                   10U
#define ADC_SEQ_LENGTH_12                   11U
#define ADC_SEQ_LENGTH_13                   12U
#define ADC_SEQ_LENGTH_14                   13U
#define ADC_SEQ_LENGTH_15                   14U
#define ADC_SEQ_LENGTH_16                   15U

#define ADC_ANALOGWATCHDOG_NONE                  0U
#define ADC_ANALOGWATCHDOG_SINGLE_REG            ((uint32_t)((0x1U << ADC_AWDSGL_POS) | (0x1U << ADC_AWDEN_POS)))
#define ADC_ANALOGWATCHDOG_SINGLE_INJEC          ((uint32_t)((0x1U << ADC_AWDSGL_POS) | (0x1U << ADC_JAWDEN_POS)))
#define ADC_ANALOGWATCHDOG_SINGLE_REGINJEC       ((uint32_t)((0x1U << ADC_AWDSGL_POS) | (0x1U << ADC_AWDEN_POS) | (0x1U << ADC_JAWDEN_POS)))
#define ADC_ANALOGWATCHDOG_ALL_REG               ((uint32_t)(0x1U << ADC_AWDEN_POS))
#define ADC_ANALOGWATCHDOG_ALL_INJEC             ((uint32_t)(0x1U << ADC_JAWDEN_POS))
#define ADC_ANALOGWATCHDOG_ALL_REGINJEC          ((uint32_t)((0x1U << ADC_AWDEN_POS) | (0x1U << ADC_JAWDEN_POS)))

#define ADC_ENABLE(HANDLE)                       ((HANDLE)->pADCx->CR2 |= (0x1UL << ADC_ADON_POS))                                  
#define ADC_DISABLE(HANDLE)                      ((HANDLE)->pADCx->CR2 &= ~(0x1UL << ADC_ADON_POS))
#define ADC_CLEAR_IT(HANDLE, FLAG)               (((HANDLE)->pADCx->SR) = ~(0x1UL << (FLAG)))
#define ADC_STATUS_FLAG(HANDLE, FLAG)		     (((HANDLE)->pADCx->SR & (0x1UL << (FLAG))) == (0x1UL << (FLAG)))
#define ADC_ENABLE_IT(HANDLE, INTERRUPT)         (((HANDLE)->pADCx->CR1) |= (0x1U << (INTERRUPT)))
#define ADC_DISABLE_IT(HANDLE, INTERRUPT)        (((HANDLE)->pADCx->CR1) &= ~(0x1U << (INTERRUPT)))
#define ADC_WD_HTR_CFG(HANDLE, VAL)              (((HANDLE)->pADCx->HTR) = (VAL))
#define ADC_WD_LTR_CFG(HANDLE, VAL)              (((HANDLE)->pADCx->LTR) = (VAL))
#define ADC_STOP(HANDLE)                         (((HANDLE)->pADCx->CR2)&= ~(0x1UL << ADC_SWSTART_POS))

#endif /* INC_STM32F446XX_ADC_DRIVER_H_ */