/**
 * @file stm32f767xx_i2c_driver.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-09-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "stm32f767xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

/*********************************************************************************************
 *   Hard-coded I2C_TIMINGR timing settings depending on the I2C clock
 *      See tables in ref manual, section 33.4.9
 *     | SM_10kHz | SM_100kHz | FM_400kHz | FM+_1000kHz |
 *  source: https://github.com/MayaPosch/Nodate/blob/master/arch/stm32/cpp/core/src/i2c.cpp
 * Note: STM32CubeMX calculates and provides the I2C_TIMINGR content in the I2C Config. window.
 * *******************************************************************************************/
uint32_t i2c_timings_4[4]  = {0x004091F3, 0x00400D10, 0x00100002, 0x00000001};
uint32_t i2c_timings_8[4]  = {0x1042C3C7, 0x10420F13, 0x00310309, 0x00100306};
// uint32_t i2c_timings_16[4] = {0x3042C3C7, 0x30420F13, 0x10320309, 0x00200204};
uint32_t i2c_timings_16[4] = {0x3042C3C7, 0x303D5B, 0x10320309, 0x00200204}; // SM_100k timing value from STM32CubeMX, for avoiding glitch (it works)
uint32_t i2c_timings_48[4] = {0xB042C3C7, 0xB0420F13, 0x50330309, 0x50100103};
uint32_t i2c_timings_54[4] = {0xD0417BFF, 0x40D32A31, 0x10A60D20, 0x00900916};




static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR2 |= (1 << I2C_CR2_START);
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    pI2Cx->CR2 |= (SlaveAddr << 1);         // Slave address bit 7:1 (master mode)
    pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN);   // RD_WRN: write transfer direction (master mode)
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    pI2Cx->CR2 |= (SlaveAddr << 1);         // Slave address bit 7:1 (master mode)
    pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN);    // RD_WRN: read transfer direction (master mode)
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}


/**
 * @brief 
 * 
 * @param pI2Cx 
 * @param EnOrDi 
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}


uint32_t RCC_GetPLLOutputClock(void)
{
    uint32_t PLLOutputClock = 0;
    return PLLOutputClock; // TODO
}



/**
 * @brief Return clock speed of APB1: System Clock--->AHB_prescaler--->APB1_Prescaler--->PCLK1 clock
 * 
 * @return uint32_t 
 */
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1;
    uint32_t SystemClk;
    uint16_t ahbp;
    uint8_t abp1p;

    uint8_t clksource;
    uint8_t temp;

    clksource = (RCC->CFGR >> 2) & 0x3; // check SWS bits 2 and 3 (bring to lsb position and mask)

    if (clksource == 0) // clock source is HSI
    {
        SystemClk = 16000000;
    }else if (clksource == 1) // clock source is HSE
    {
        SystemClk = 8000000;
    }else if (clksource == 2) // clock source is PLL
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    // for AHB prescaler
    temp = (RCC->CFGR >> 4) & 0xF;  // check HPRE bits 4 to 7 (bring to lsb position and mask)
    if (temp < 8)
    {
        ahbp = 1;
    }else
    {
        ahbp = AHB_PreScaler[temp - 8]; // remove 8 to index to get: 8->2, 9->4, 10->8, ..., 15->512
    }

    // for APB1 prescaler
    temp = (RCC->CFGR >> 10) & 0x7;  // check PPRE bits 10 to 12 (bring to lsb position and mask)
    if (temp < 4)
    {
        abp1p = 1;
    }else
    {
        abp1p = APB1_PreScaler[temp - 4]; // remove 4 to index to get: 4->2, 5->4, 6->8, 7->16
    }

    pclk1 = (SystemClk / ahbp) / abp1p;

    return pclk1;
}



/**
 * @brief I2C Peripheral Clock Control
 * 
 * @param pI2Cx 
 * @param EnorDi
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}else if (pI2Cx == I2C4)
		{
			I2C4_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}else if (pI2Cx == I2C4)
		{
			I2C4_PCLK_DI();
		}
	}

}


/**
 * @brief 
 * 
 * @param pI2CHandle 
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

    uint32_t tempreg;

    // enable the clock for the I2Cx peripheral
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // program the device own address (I2C_OAR1 bits 1 to 7)
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // fill I2C_TIMINGR register using hard-coded timings values
    uint8_t mode = 0;
    mode = pI2CHandle->I2C_Config.I2C_SCLSpeed; // mode: SM10k, SM100k, FM or FMPLUS
    if (RCC_GetPCLK1Value() == 8000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_8[mode];
    }else if (RCC_GetPCLK1Value() == 16000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_16[mode];
    }else if (RCC_GetPCLK1Value() == 48000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_48[mode];
    }else if (RCC_GetPCLK1Value() == 54000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_54[mode];
    }
}

/**
 * @brief TODO
 * 
 * @param pI2Cx 
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/**
 * @brief I2C Master Mode Send Data
 * 
 * @param pI2CHandle    I2C handle
 * @param pTxbuffer     buffer
 * @param Len           length
 * @param SlaveAddr     Slave address
 * @param Sr 
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    // Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

    // Set autoend to 0, and length of data to be transmitted
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
    pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);  //number of bytes to be transmitted

    // Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // Send the data until Len becomes 0
    while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

    // wait until TC flag is set (when NBYTES data are transferred)
    while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TC)));
    
    if (Sr == I2C_DISABLE_SR)
    {
        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
    

    // clear STOPF flag in I2C_ICR and clear the I2C_CR2 register:
    pI2CHandle->pI2Cx->ICR &= ~(1 << I2C_ICR_STOPCF);
    pI2CHandle->pI2Cx->CR2 = 0x0;
}

/**
 * @brief I2C Master Mode Receive Data
 * 
 * @param pI2CHandle I2C handle
 * @param pRxBuffer buffer
 * @param Len length
 * @param SlaveAddr slave address
 */ 
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    // Send the address of the slave with r/nw bit set to r(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

    // Set autoend to 0, and length of data to be received
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
    pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);  //number of bytes to be received

    // Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // Receive the data until Len becomes 0
    while(Len > 0)
	{   
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) ); //Wait till RXNE is set
		*pRxBuffer = pI2CHandle->pI2Cx->RXDR; // read data into buffer
		pRxBuffer++;
		Len--;
	}

    // wait until TC flag is set (when NBYTES data are transferred)
    while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TC)));
    
    if (Sr == I2C_DISABLE_SR)
    {
        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }


    // clear STOPF flag in I2C_ICR and clear the I2C_CR2 register:
    pI2CHandle->pI2Cx->ICR &= ~(1 << I2C_ICR_STOPCF);
    pI2CHandle->pI2Cx->CR2 = 0x0;
}


/**
 * @brief I2C IRQ Interrupt Enable or Disable
 * 
 * @param IRQNumber 
 * @param EnorDi 
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @brief I2C IRQ Priority Configuration
 * 
 * @param IRQNumber 
 * @param IRQPriority 
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * @brief I2C Interrupt based Master Mode Send Data
 * 
 * @param pI2CHandle 
 * @param pTxbuffer 
 * @param Len 
 * @param SlaveAddr 
 * @param Sr 
 * @return uint8_t
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

        // Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

        // Set autoend to 0, and length of data to be transmitted
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
        pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->TxLen << I2C_CR2_NBYTES);  //number of bytes to be transmitted

        //Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //enable TXIE, TCIE, STOPIE Control Bits
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);

        //enable ERRIE Control Bit
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);
	}

	return busystate;
}




/**
 * @brief I2C Interrupt based Master Mode Receive Data
 * 
 * @param pI2CHandle 
 * @param pRxBuffer 
 * @param Len 
 * @param SlaveAddr 
 * @param Sr 
 * @return uint8_t 
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
        uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

        // Send the address of the slave with r/nw bit set to r(1) (total 8 bits )
	    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

        // Set autoend to 0, and length of data to be received
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
        pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);  //number of bytes to be received

        //Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //enable RXIE, STOPIE, TCIE, NACKIE Control Bits
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_RXIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE);

		//enable ERRIE Control Bit
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);

	}

	return busystate;
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    //disable TXIE, STOPIE, TCIE Control Bits
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TXIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE);

	pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;

    //pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    //disable RXIE, STOPIE, TCIE, NACKIE Control Bits
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_RXIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

    //pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register
}




/**
 * @brief I2C Event IRQ Handling
 * 
 * @param pI2CHandle 
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    // Handle for interrupt generated by TC event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TCIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TC);
    if (temp1 && temp2)
    {
            //1. generate the STOP condition
            if(pI2CHandle->Sr == I2C_DISABLE_SR)
            {
                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            // 2. reset all the member elements of the handle structure.
            I2C_CloseSendData(pI2CHandle);

            pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register

    }

    // Handle for interrupt generated by STOPF event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_STOPIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_STOPF);
    if (temp1 && temp2)
    {
        // STOPF flag is set
        // - set by hardware when a Stop condition is detected.
        // - cleared by software by setting the STOPCF bit:
        pI2CHandle->pI2Cx->ICR &= ~(1 << I2C_ICR_STOPCF);

        //Notify the application that STOP is detected
        //I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    // Handle for interrupt generated by TXIS event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TXIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TXIS);
    if (temp1 && temp2)
    {
        // TXIS flag is set
        // We have to do the data transmission
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            if (pI2CHandle->TxLen > 0)
            {
                // 1. load the data in TXDR
                pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
                // 2. decrement TxLen
                pI2CHandle->TxLen--;
                // 3. increment buffer address
		        pI2CHandle->pTxBuffer++;
		        
            }

        }
    }

    // Handle for interrupt generated by RXNE event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_RXIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_RXNE);
    if (temp1 && temp2)
    {
        // RXNE flag is set
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if (pI2CHandle->RxLen > 0)
            {
                // 1. read RXDR data into buffer
                *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
                // 2. decrement RxLen
                pI2CHandle->RxLen--;
                // 3. increment buffer address
		        pI2CHandle->pRxBuffer++;
		        
            }


            if(pI2CHandle->RxLen == 0)
            {
                //1. generate the stop condition
		        if(pI2CHandle->Sr == I2C_DISABLE_SR){
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
			        
                //2 . Close the I2C rx    
                I2C_CloseReceiveData(pI2CHandle);

                
                //3. Notify the application
		        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);

                pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register

            
            }   

        }
    }

}





/**
 * @brief I2C Error IRQ Handling
 * 
 * @param pI2CHandle 
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1,temp2;

    //Know the status of ERRIE control bit in CR1
    temp2 = (pI2CHandle->pI2Cx->CR1) & ( 1 << I2C_CR1_ERRIE);


    /***********************Check for Bus error************************************/ 
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_BERR);
    if(temp1  && temp2 )
    {
        //This is Bus error
        
        //clear the bus error flag 
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_BERRCF);
        
        //notify the application about the error 
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
    }

    /***********************Check for arbitration lost error************************************/ 
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_ARLO);
    if(temp1  && temp2)
    {
        //This is arbitration lost error
        
        //clear the arbitration lost error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_ARLOCF);

        //notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);     
    }

    /***********************Check for Overrun/underrun error************************************/
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_OVR);
    if(temp1  && temp2)
    {
        //This is Overrun/underrun error
        
        //clear the Overrun/underrun error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_OVRCF);

        //notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);     
    }

    /***********************Check for Time out error************************************/
    temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_TIMEOUT);
    if(temp1  && temp2)
    {
        //This is Time out error
        
        //clear the Time out error flag
        pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_TIMEOUTCF);

        //notify the application about the error 
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT); 
    }
}