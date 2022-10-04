#include <string.h>
#include "stm32f767xx.h"


/* Pins to communicate over SPI2 (Cf. datasheet, alternate function mapping)
 * PB15 ---> SPI2_MOSI
 * PB14 ---> SPI2_MISO
 * PB13 ---> SPI2_SCLK
 * PB12 ---> SPI2_NSS
 * ALT function mode: 5
 */


void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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

    // NSS ---- pin not used in this example
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    // GPIO_Init(&SPIPins);
}

void SPI2_Inits()
{
    SPI_Handle_t SPI2handle;

    SPI2handle.pSPIx = SPI2;

    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
    SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management enabled

    SPI_Init(&SPI2handle);
}

int main(void)
{

    char user_data[] = "Hello world";

    // initialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInits();

    // initialize SPI2 peripheral parameters
    SPI2_Inits();

    // this makes NSS internally high
    SPI_SSIConfig(SPI2, ENABLE);

    // enable the SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    // send data
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));


     // Let's confirm SPI is not busy
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    //Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

    while (1);
}