#include <stm32f446xx_usart_driver.h>


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (pUSARTx == USART1)
            USART1_PCLK_EN();
        else if (pUSARTx == USART2)
            USART2_PCLK_EN();
        else if (pUSARTx == USART3)
            USART3_PCLK_EN();
        else if (pUSARTx == UART4)
            UART4_PCLK_EN();
        else if (pUSARTx == UART5)
            UART5_PCLK_EN();
    }
    else
    {
        if (pUSARTx == USART1)
            USART1_PCLK_DI();
        else if (pUSARTx == USART2)
            USART2_PCLK_DI();
        else if (pUSARTx == USART3)
            USART3_PCLK_DI();
        else if (pUSARTx == UART4)
            UART4_PCLK_DI();
        else if (pUSARTx == UART5)
            UART5_PCLK_DI();
    }
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	else
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
}

void USART_Init(USART_Handle_t *USART_Instance)
{
    uint32_t tempreg = 0;
    USART_PeriClockControl(USART_Instance->pUSARTx, ENABLE);

    if (USART_Instance->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        tempreg |= (0x1UL << USART_CR1_RE);
    }
    else if (USART_Instance->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        tempreg |= (0x1UL << USART_CR1_TE);
    }
    else if (USART_Instance->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        tempreg |= ((0x1UL << USART_CR1_RE) | (0x1UL << USART_CR1_TE));
    }

    if (USART_Instance->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        tempreg |= (0x1UL << USART_CR1_PCE);
    }
    else if (USART_Instance->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        tempreg |= (0x1UL << USART_CR1_PCE);
        tempreg |= (0x1UL << USART_CR1_PS);
    }

    tempreg |= (USART_Instance->USART_Config.USART_WordLength << USART_CR1_M);
    USART_Instance->pUSARTx->CR1 = tempreg;

    tempreg = 0;
    tempreg |= (USART_Instance->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);
    USART_Instance->pUSARTx->CR2 = tempreg;
    tempreg = 0;

    if (USART_Instance->USART_Config.USART_HWFlowControl != USART_HW_FLOW_CTRL_NONE)
    {
        if (USART_Instance->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
        {
            tempreg |= (0x1UL << USART_CR3_CTSE);
        }
        else if (USART_Instance->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
        {
            tempreg |= (0x1UL << USART_CR3_RTSE);
        }
        else if (USART_Instance->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
        {
            tempreg |= ((0x1UL << USART_CR3_CTSE) | (0x1UL << USART_CR3_RTSE));
        }
        
    }
    USART_Instance->pUSARTx->CR3 = tempreg;

	USART_SetBaudRate(USART_Instance->pUSARTx, USART_Instance->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
        USART1_REG_RESET();
    else if (pUSARTx == USART2)
        USART2_REG_RESET();
    else if (pUSARTx == USART3)
        USART3_REG_RESET();
    else if (pUSARTx == UART4)
        UART4_REG_RESET();
    else if (pUSARTx == UART5)
        UART5_REG_RESET();
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits 
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
			
			//check for USART_ParityControl
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice 
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer 
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			
			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame
			//check are we using USART_ParityControl control or not
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE);

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (0x1UL << USART_CR1_TXEIE);
		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (0x1UL << USART_CR1_TCIE);
	}
	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE);

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (0x1UL << USART_CR1_RXNEIE);
	}
	return rxstate;
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLKx;
	uint32_t usartdiv;
	uint32_t M_part, F_part;
	uint32_t tempreg = 0;

	if (pUSARTx == USART1 || pUSARTx == USART6)
		PCLKx = RCC_GetPCLK2Value();
	else
		PCLKx = RCC_GetPCLK1Value();

	if (pUSARTx->CR1 & (0x1UL << USART_CR1_OVER8))
		usartdiv = ((25*PCLKx) / (2 * BaudRate));
	else
		usartdiv = ((25*PCLKx) / (4 * BaudRate));

	M_part = usartdiv / 100;
	tempreg |= (M_part << 4U);
	
	if (pUSARTx->CR1 & (0x1UL << USART_CR1_OVER8))
	{
		F_part = ((usartdiv - (M_part * 100))*8 + 50) / 100;
		F_part &= ((uint8_t)0x07);
	}
	else
	{
		F_part = ((usartdiv - (M_part * 100))*16 + 50) / 100;
		F_part &= ((uint8_t)0x0F);
	}
	tempreg |= F_part;

	pUSARTx->BRR = tempreg;
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if (pUSARTx->SR & (0x1UL << StatusFlagName))
        return FLAG_SET;
    return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    pUSARTx->SR &= ~(0x1UL << StatusFlagName);
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
    uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;
	if (ENorDI == ENABLE)
		*(NVIC_ISER_BASEADDR + temp1) |= (1 << temp2);
	else
		*(NVIC_ICER_BASEADDR + temp1) |= (1 << temp2);
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t sa = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL); // shift amount
	*(NVIC_IPR_BASEADDR + iprx) &= ~(0xff << sa);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << sa);
}

void USART_IRQHandling(USART_Handle_t *USART_Instance)
{
	
}

void USART_ApplicationEventCallback(USART_Handle_t *USART_Instance, uint8_t AppEv)
{

}