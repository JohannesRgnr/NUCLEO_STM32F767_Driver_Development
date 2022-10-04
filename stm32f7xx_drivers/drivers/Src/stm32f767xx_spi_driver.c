/*
 * stm32f767xx_spi_driver.c
 *
 *  Created on: September 05, 2022
 *      Author: johannes regnier
 */


#include "stm32f767xx_spi_driver.h"

/**
 *      private helper functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - 
 *
 * @param[in]         - 
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
 if (EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}else if (pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
    } 
    else
    {
        if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}else if (pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
    }
}


/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

    // peripheral clock enable
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // Configure the SPI_CR1 register
    uint32_t temp = 0;

    // 1. Configure the device mode
    temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // BIDIMODE (bit #15) should be cleared
        temp &= ~(1 << SPI_CR1_BIDIMODE);
    }else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // BIDIMODE should be set
        temp |= (1 << SPI_CR1_BIDIMODE);
    }else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // BIDIMODE should be cleared
        temp &= ~(1 << SPI_CR1_BIDIMODE);
        // RXONLY (bit #10) should be set
        temp |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure the clock speed
    temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. Configure the CPOL
    temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 5. Configure the CPHA
    temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    // Save value of temp register into CR1 register
    pSPIHandle->pSPIx->CR1 = temp;

    // Configure the SPI_CR2 register
    // 1. Configure the data size (DS) (bits 8->11 on CR2 register)
    pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS ); //clearing 4 bits
    pSPIHandle->pSPIx->CR2 |= pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS; 

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 * 
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();
		}else if (pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}else if (pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}else if (pSPIx == SPI4)
		{
			SPI4_REG_RESET();
		}else if (pSPIx == SPI5)
		{
			SPI5_REG_RESET();
		}else if (pSPIx == SPI6)
		{
			SPI6_REG_RESET();
		}    
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Send Data via SPI
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @note              - This is a blocking call (cf. while loops)
 * 
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while( Len > 0 )
    {
        // 1. Wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); // wait while TXE is 0 (or use bitwise check &:  while (!(pSPIx->SR & (1 << SPI_SR_TXE))))

        // 2. Check the DS value in CR2 -- here only 16 and 8 bits lengths are implemented
        if ((0xF & (pSPIx->CR2 >> 8)) == SPI_DS_16BITS)
        {
            // 16 bits data size
            // 1. Load data into the DR
            pSPIx->DR = *((uint16_t *)pTxBuffer); // typecasting to uint16_t to get 16 bits data when dereferencing the pointer
            Len--;
            Len--; // 2 times length reduction because 2 bytes of data got sent
            // 2. Increment the pointer
            (uint16_t *)pTxBuffer++; // increment by 2 (2 bytes data)
        }
        else if ((0xF & (pSPIx->CR2 >> 8)) == SPI_DS_8BITS)
        {
            // 8 bits data size
            *((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer; // cf. data packing RM section 35.5.9
            // pSPIx->DR = *pTxBuffer; // just dereferencing the pointer (pointer if of type uint8_t)
            Len--; // 1 time length reduction because 1 byte of data got sent
            pTxBuffer++;
        }

        // clear OVR flag
        uint32_t tempreg = 0x00;
        tempreg = pSPIx->DR;    // read operation on DR register
        tempreg = pSPIx->SR;    // read operation on SR register
        (void)tempreg;          //typecast to void to avoid gcc/g++ warnings 
    }
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Receive Data via SPI
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @note              - This is a blocking call (cf. while loops)
 * 
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while( Len > 0 )
    {
        // 1. Wait until RXNE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET); // wait while RXNE is 0 (or use bitwise check &:  while (!(pSPIx->SR & (1 << SPI_SR_TXE))))

        // 2. Check the DS value in CR2 -- here only 16 and 8 bits lengths are implemented
        if ((0xF & (pSPIx->CR2 >> 8)) == SPI_DS_16BITS)
        {
            // 16 bits data size
            // 1. Load data from the DR to RX
            *((uint16_t *)pRxBuffer) = pSPIx->DR; // typecasting to uint16_t to get 16 bits data when dereferencing the pointer
            Len--;
            Len--; // 2 times length reduction because 2 bytes of data got received
            // 2. Increment the pointer
            (uint16_t *)pRxBuffer++; // increment by 2 (2 bytes data)
        }
        else if ((0xF & (pSPIx->CR2 >> 8)) == SPI_DS_8BITS)
        {
            // 8 bits data size
            *pRxBuffer = *((volatile uint8_t *)&pSPIx->DR); // cf. data packing RM section 35.5.9
            // pSPIx->DR = *pTxBuffer; // just dereferencing the pointer (pointer if of type uint8_t)
            Len--; // 1 time length reduction because 1 byte of data got received
            pRxBuffer++;
        }
        
    }
}



/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 
 * 
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE); // set SPE (bit #6)
    }else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); // reset bit
    }
}



/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 
 * 
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
        if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI); // set SSI (bit #8)
    }else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI); // reset bit
    }
}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 
 * 
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
        if(EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE); // set SSOE (bit #2)
    }else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE); // reset bit
    }
}


/*********************************************************************
 * @fn      		  - SPI_FRXTHConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 
 * 
 */
void SPI_FRXTHConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
        if(EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_FRXTH); // set FRXTH (bit #12)
    }else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH); // reset bit
    }
}


/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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




/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @note              -

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * @brief SPI Send Data Interrupt Mode
 * 
 * @param pSPIHandle 
 * @param pTxBuffer 
 * @param Len 
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        //2. Mark the SPI state as busy in transmission so that no other code
        // can take over same SPI peripheral until transmission is over.
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

        //4. Data transmission is handled by the ISR code

    }

    return state;
}


/**
 * @brief SPI Receive Data Interrupt Mode
 * 
 * @param pSPIHandle 
 * @param pRxBuffer 
 * @param Len 
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        //2. Mark the SPI state as busy in transmission so that no other code
        // can take over same SPI peripheral until transmission is over.
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        //3. Enable the TXEIE control bit to get interrupt whenever RXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

        //4. Data transmission is handled by the ISR code

    }

    return state;
}





/**
 * @brief SPI IRQ Handling
 * 
 * @param pHandle 
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t temp1, temp2;
    // let's check for TXE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if(temp1 && temp2)
    {
        // handle TXE
        spi_txe_interrupt_handle(pHandle);
    }

    // let's check for RXNE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(temp1 && temp2)
    {
        // handle TXE
        spi_rxne_interrupt_handle(pHandle);
    }

    // let's check for OVR flag 
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if(temp1 && temp2)
    {
        // handle TXE
        spi_ovr_err_interrupt_handle(pHandle);
    }    

}


// helper functions implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // Check the DS value in CR2 -- here only 16 and 8 bits lengths are implemented
    if ((0xF & (pSPIHandle->pSPIx->CR2 >> 8)) == SPI_DS_16BITS)
    {
        // 16 bits data size
        // 1. Load data into the DR
        pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer); // typecasting to uint16_t to get 16 bits data when dereferencing the pointer
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--; // 2 times length reduction because 2 bytes of data got sent
        // 2. Increment the pointer
        (uint16_t *)pSPIHandle->pTxBuffer++; // increment by 2 (2 bytes data)
    }
    else if ((0xF & (pSPIHandle->pSPIx->CR2 >> 8)) == SPI_DS_8BITS)
    {
        // 8 bits data size
        *((volatile uint8_t *)&pSPIHandle->pSPIx->DR) = *pSPIHandle->pTxBuffer; // cf. data packing RM section 35.5.9
        // pSPIx->DR = *pTxBuffer; // just dereferencing the pointer (pointer if of type uint8_t)
        pSPIHandle->TxLen--; // 1 time length reduction because 1 byte of data got sent
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen)
    {
        // TxLen is zero, close the SPI transmission and inform the application that TX is over

        // this prevents interrupts from setting up of TXE flag
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }

    // clear OVR flag
    // uint32_t tempreg = 0x00;
    // tempreg = pSPIHandle->DR;    // read operation on DR register
    // tempreg = pSPIHandle->SR;    // read operation on SR register
    // UNUSED(tempreg);        // prevent compilation warning
    
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if ((0xF & (pSPIHandle->pSPIx->CR2 >> 8)) == SPI_DS_16BITS)
    {
        // 16 bits data size
        // 1. Load data from the DR to RX
        *((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR; // typecasting to uint16_t to get 16 bits data when dereferencing the pointer
        pSPIHandle->RxLen -= 2; // 2 times length reduction because 2 bytes of data got received
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++; // increment by 2 (2 bytes data)
    }
    else if ((0xF & (pSPIHandle->pSPIx->CR2 >> 8)) == SPI_DS_8BITS)
    {
        // 8 bits data size
        *(pSPIHandle->pRxBuffer) = *((volatile uint8_t *)&pSPIHandle->pSPIx->DR); // cf. data packing RM section 35.5.9
        // pSPIx->DR = *pTxBuffer; // just dereferencing the pointer (pointer if of type uint8_t)
        pSPIHandle->RxLen--; // 1 time length reduction because 1 byte of data got received
        pSPIHandle->pRxBuffer++;
    }

    if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    // 1. clear the OVR flag
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
       temp = pSPIHandle->pSPIx->DR;
       temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp; // typecast to void to avoid gcc/g++ warnings 
    // 2. Inform the application
    SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp; // typecast to void to avoid gcc/g++ warnings 
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
    // weak implementation --- the application may override this function
}
