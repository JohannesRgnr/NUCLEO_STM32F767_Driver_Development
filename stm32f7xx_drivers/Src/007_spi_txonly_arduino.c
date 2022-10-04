/**
 * @file 007_spi_txonly_arduino.c
 * @author johannes regnier
 * @brief SPI transmission test to Arduino
 * @version 0.1
 * @date 2022-09-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <string.h>
#include "stm32f767xx.h"

#define HIGH            1
#define LOW             0
#define BTN_PRESSED     HIGH

/* Pins to communicate over SPI2 (Cf. datasheet, alternate function mapping)
 * PB15 ---> SPI2_MOSI (CN7 pin 3 on NUCLEO F767)
 * PC2  ---> SPI2_MISO (CN10 pin 9 on NUCLEO F767)
 * PB13 ---> SPI2_SCLK (CN7 pin 5 on NUCLEO F767)
 * PB12 ---> SPI2_NSS  (CN7 pin 7 on NUCLEO F767)
 * ALT function mode: 5
 */

void delay(void){
  for (uint32_t i = 0; i < 200000; i++)
    ;
}

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO ---- pin not used in this example
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&SPIPins);

    // NSS 
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits()
{
    SPI_Handle_t SPI2handle;

    SPI2handle.pSPIx = SPI2;

    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // SCLK of 2MHz
    SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

    SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn;

    GPIOBtn.pGPIOx = GPIOC; // User button is connected to PC13 (Cf. Nucleo User Manual, or check schematics)
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

int main(void)
{

    char user_data[] = "Marit est une super poulette";

    GPIO_ButtonInit();

    // initialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInits();

    // initialize SPI2 peripheral parameters
    SPI2_Inits();

    // enable the SS output (SSOE)
    // then, when SPE = 1, NSS pulled to Low. When SPE = 0, NSS pulled to High.
    SPI_SSOEConfig(SPI2, ENABLE);


    while (1)
    {
        //wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        delay();

        // enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE);

        // first send length info
        uint8_t dataLen = strlen(user_data);
        SPI_SendData(SPI2, &dataLen, 1); // 1 byte of data

        // then send data
        SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

        // Let's confirm SPI is not busy
        while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2,DISABLE);
        
    }

    return 0;

}