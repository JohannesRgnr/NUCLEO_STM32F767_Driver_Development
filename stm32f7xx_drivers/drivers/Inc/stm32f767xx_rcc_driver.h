/**
 * @file stm32f767xx_rcc_driver.h
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-10-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef STM32F767XX_RCC_DRIVER_H
#define STM32F767XX_RCC_DRIVER_H

#include "stm32f767xx.h"



/*********************************************************************************
 *                  APIs supported by this driver
 *      For more infos, check the function definitions
 *********************************************************************************/

uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif // !STM32F767XX_RCC_DRIVER_H
