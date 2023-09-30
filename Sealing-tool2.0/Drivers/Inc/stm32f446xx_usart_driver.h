
#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"
#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx_rcc_driver.h"

typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_Baud;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
}USART_Config_t;

typedef struct
{
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_Config;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint8_t TxBusyState;
    uint8_t RxBusyState;
}USART_Handle_t;

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI);
void USART_Init(USART_Handle_t *USART_Instance);
void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *USART_Instance, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *USART_Instance, uint8_t *pRxBuffer, uint32_t Len);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *USART_Instance);

void USART_ApplicationEventCallback(USART_Handle_t *USART_Instance, uint8_t AppEv);

/* USART_SR bit definitions */

#define USART_SR_PE                         0U
#define USART_SR_FE                         1U
#define USART_SR_NF                         2U
#define USART_SR_ORE                        3U
#define USART_SR_IDLE                       4U
#define USART_SR_RXNE                       5U
#define USART_SR_TC                         6U
#define USART_SR_TXE                        7U
#define USART_SR_LBD                        8U
#define USART_SR_CTS                        9U

/* USART_CR1 bit definitions */

#define USART_CR1_SBK                       0U
#define USART_CR1_RWU                       1U
#define USART_CR1_RE                        2U
#define USART_CR1_TE                        3U
#define USART_CR1_IDLEIE                    4U
#define USART_CR1_RXNEIE                    5U
#define USART_CR1_TCIE                      6U
#define USART_CR1_TXEIE                     7U
#define USART_CR1_PEIE                      8U
#define USART_CR1_PS                        9U
#define USART_CR1_PCE                       10U
#define USART_CR1_WAKE                      11U
#define USART_CR1_M                         12U
#define USART_CR1_UE                        13U
#define USART_CR1_OVER8                     15U

/* USART_CR2 bit definitions */

#define USART_CR2_ADD                       0U
#define USART_CR2_LBDL                      5U
#define USART_CR2_LBDIE                     6U
#define USART_CR2_LBCL                      8U
#define USART_CR2_CPHA                      9U
#define USART_CR2_CPOL                      10U
#define USART_CR2_CLKEN                     11U
#define USART_CR2_STOP                      12U
#define USART_CR2_LINEN                     14U

/* USART_CR3 bit definitions */

#define USART_CR3_EIE                       0U
#define USART_CR3_IREN                      1U
#define USART_CR3_IRLP                      2U
#define USART_CR3_HDSEL                     3U
#define USART_CR3_NACK                      4U
#define USART_CR3_SCEN                      5U
#define USART_CR3_DMAR                      6U
#define USART_CR3_DMAT                      7U
#define USART_CR3_RTSE                      8U
#define USART_CR3_CTSE                      9U
#define USART_CR3_CTSIE                     10U
#define USART_CR3_ONEBIT                    11U

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX                  0
#define USART_MODE_ONLY_RX                  1
#define USART_MODE_TXRX                     2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD                 2
#define USART_PARITY_EN_EVEN                1
#define USART_PARITY_DISABLE                0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS                 0
#define USART_WORDLEN_9BITS                 1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1                    0
#define USART_STOPBITS_0_5                  1
#define USART_STOPBITS_2                    2
#define USART_STOPBITS_1_5                  3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	        0
#define USART_HW_FLOW_CTRL_CTS    	        1
#define USART_HW_FLOW_CTRL_RTS    	        2
#define USART_HW_FLOW_CTRL_CTS_RTS	        3

#define USART_BUSY_IN_RX                    1
#define USART_BUSY_IN_TX                    2
#define USART_READY                         0

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */