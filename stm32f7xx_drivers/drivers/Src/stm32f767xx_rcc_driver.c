/**
 * @file stm32f767xx_rcc_driver.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-10-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "stm32f767xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_PreScaler[4] = {2, 4, 8, 16};

/**
 * @brief TODO
 * 
 * @return uint32_t 
 */
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
        abp1p = APB_PreScaler[temp - 4]; // remove 4 to index to get: 4->2, 5->4, 6->8, 7->16
    }

    pclk1 = (SystemClk / ahbp) / abp1p;

    return pclk1;
}



/**
 * @brief Return clock speed of APB2: System Clock--->AHB_prescaler--->APB2_Prescaler--->PCLK2 clock
 * 
 * @return uint32_t 
 */
uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t pclk2;
    uint32_t SystemClk;
    uint16_t ahbp;
    uint8_t abp2p;

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

    // for APB2 prescaler
    temp = (RCC->CFGR >> 13) & 0x7;  // check PPRE bits 13 to 15 (bring to lsb position and mask)
    if (temp < 4)
    {
        abp2p = 1;
    }else
    {
        abp2p = APB_PreScaler[temp - 4]; // remove 4 to index to get: 4->2, 5->4, 6->8, 7->16
    }

    pclk2 = (SystemClk / ahbp) / abp2p;

    return pclk2;
}