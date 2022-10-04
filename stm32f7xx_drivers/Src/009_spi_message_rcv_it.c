/**
 * @file 009_spi_message_rcv_it.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-09-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */


#include <string.h>
#include "stm32f767xx.h"
#include <stdio.h>
#include "SEGGER_RTT.h"


SPI_Handle_t SPI4handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;


volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 200000 ; i ++);
}



/* Pins to communicate over SPI4 (Cf. datasheet, alternate function mapping)
 * PE6  ---> SPI4_MOSI (CN9 pin 20 on NUCLEO F767)
 * PE5  ---> SPI4_MISO (CN9 pin 18 on NUCLEO F767)
 * PE2  ---> SPI4_SCLK (CN9 pin 14 on NUCLEO F767)
 * PE4  ---> SPI4_NSS  (CN9 pin 16 on NUCLEO F767)
 * ALT function mode: 5
 */

void SPI4_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOE;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&SPIPins);

    // MISO 
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&SPIPins);

    // NSS 
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPins);
}


void SPI4_Inits()
{
    SPI4handle.pSPIx = SPI4;

    SPI4handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI4handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI4handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // SCLK of 0.5 MHz -- faster doesn't seem to work
    SPI4handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
    SPI4handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI4handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI4handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

    SPI_Init(&SPI4handle);
}

/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

}


int main(void)
{

	uint8_t dummy = 0xff;

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    printf("Application is running\n");

	Slave_GPIO_InterruptPinInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI4_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI4_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI4,ENABLE);

    // enable the FIFO threshold to 8 bits (FRXTH)
    SPI_FRXTHConfig(SPI4, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI4,ENABLE);

	while(1){

		rcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI4,ENABLE);


		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPI_SendDataIT(&SPI4handle,&dummy,1) == SPI_BUSY_IN_TX);
			while ( SPI_ReceiveDataIT(&SPI4handle, &ReadByte,1) == SPI_BUSY_IN_RX );
		}


		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI4,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI4,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);


	}

	return 0;

}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI4_IRQHandler(void)
{

	SPI_IRQHandling(&SPI4handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}

