/**
 * @file stm32f767xx_usart_driver.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-10-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "stm32f767xx_usart_driver.h"





/**
 * @brief USART_PeriClockControl
 * 
 * @param pUSARTx 
 * @param EnorDi ENABLE or DISABLE macros
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
 if (EnorDi == ENABLE)
    {
        if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}else if (pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}else if (pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
    } 
    else
    {
        if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}else if (pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		}else if (pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}
    }
}


/**
 * @brief USART Set Baud Rate
 * 
 * @param pUSARTx 
 * @param BaudRate 
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    //Variable to hold the APB clock
    uint32_t PCLKx;

    uint32_t usartdiv;

    uint32_t tempreg=0;

    //Get the value of APB bus clock in to the variable PCLKx
    if(pUSARTx == USART1 || pUSARTx == USART6)
    {
        //USART1 and USART6 are hanging on APB2 bus
        PCLKx = RCC_GetPCLK2Value();
    }else
    {
        //USART2,3 and UART4,5,7,8 are hanging on APB1 bus
        PCLKx = RCC_GetPCLK1Value();
    }

    //Check for OVER8 configuration bit
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        //OVER8 = 1 , over sampling by 8
        usartdiv = (((PCLKx * 2) + (BaudRate / 2)) / BaudRate);     // Rounded

        tempreg = (uint16_t)(usartdiv & 0xFFF0);                    // BRR[15:4] = USARTDIV[15:4]
        tempreg |= (uint16_t)((usartdiv & (uint16_t)0x000F) >> 1);  // BRR[2:0]  = USARTDIV[3:0] shifted 1 bit to the right.
    }
    else
    {
        //over sampling by 16
        tempreg = ((PCLKx + (BaudRate / 2)) / BaudRate); // BRR = USARTDIV. Rounded.
    }

    //copy the value of tempreg into BRR register
    pUSARTx->BRR = tempreg;
}



/**
 * @brief USART initialization
 * 
 * @param pUSARTHandle 
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	// peripheral clock enable
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		// enable Receiver bit field 
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		// enable Transmitter bit field 
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		// enable the both Transmitter and Receiver bit fields 
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    // configure the Word length
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M0 ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// enable the parity control 
		tempreg |= ( 1 << USART_CR1_PCE);

		//EVEN parity is default

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		// enable the parity control 
	    tempreg |= ( 1 << USART_CR1_PCE);

	    // enable ODD parity 
	    tempreg |= ( 1 << USART_CR1_PS);

	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	// configure the number of stop bits inserted during USART frame transmission 
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;
	
	//Configuration of USART hardware flow control 
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// enable CTS flow control 
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control 
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS Flow control 
		tempreg |= ( ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

}


/**
 * @brief TODO
 * 
 * @param pUSARTx 
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    // TODO
}

/**
 * @brief USART Peripheral Enable / Disable
 * 
 * @param pUSARTx 
 * @param EnOrDi 
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pUSARTx->CR1 |= (1 << USART_CR1_UE); // set UE (bit #0)
    }else
    {
        pUSARTx->CR1 &= ~(1 << USART_CR1_UE); // reset bit
    }
}


/**
 * @brief 
 * 
 * @param pUSARTx 
 * @param FlagName 
 * @return uint8_t 
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
    if(pUSARTx->ISR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;    
}

/**
 * @brief 
 * 
 * @param pUSARTx 
 * @param StatusFlagName 
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	// TODO

}



/**
 * @brief USART Send Data
 * 
 * @param pUSARTHandle 
 * @param pTxBuffer 
 * @param Len 
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//wait until TXE flag is set in the ISR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits 
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);
			
			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//increment pTxBuffer twice 
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
			//8-bit data transfer 
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);
			
			//increment the buffer address
			pTxBuffer++;
		}
	}

	//wait until TC flag is set in the ISR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/**
 * @brief USART Receive Data
 * 
 * @param pUSARTHandle 
 * @param pRxBuffer 
 * @param Len 
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//wait until RXNE flag is set in the ISR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
                pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
				 
				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
            }

            else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}



/**
 * @brief USART IRQ Interrupt Enable or Disable
 * 
 * @param IRQNumber 
 * @param EnorDi 
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)							// 0 to 31
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64) // 32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if (IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
			
		}else if (IRQNumber >= 96 && IRQNumber < 128) // 96 to 127 (max is 109 on F767)
		{
			//program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
			
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
			
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			
		}else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
			
		}else if (IRQNumber >= 96 && IRQNumber < 128)
		{
			//program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));

		}
	}
}


/**
 * @brief USART IRQ Priority Configuration
 * 
 * @param IRQNumber 
 * @param IRQPriority 
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    //1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4;
	
	// !! 16 programmable priority levels (4 bits of interrupt priority are used) Cf. RM 10.1
	// only 4 MSBits are implemented!

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}



/**
 * @brief USART Send Data with IT
 * 
 * @param pUSARTHandle 
 * @param pTxBuffer 
 * @param Len 
 * @return uint8_t 
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//enable interrupt for TXE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        //enable interrupt for TC 
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);		
		

	}

	return txstate;

}


/**
 * @brief USART Receive Data with IT
 * 
 * @param pUSARTHandle 
 * @param pRxBuffer 
 * @param Len 
 * @return uint8_t 
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;

}




/**
 * @brief USART Application Event Callback
 * 
 * @param pUSARTHandle 
 * @param AppEv 
 * @note __weak 
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
    // weak implementation --- the application may override this function
}



