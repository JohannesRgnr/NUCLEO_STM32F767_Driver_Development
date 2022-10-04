/**
 * @file 008_spi_cmd_handling.c
 * @author johannes regnier
 * @brief SPI transmission test to Arduino
 * @version 0.1
 * @date 2022-09-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <string.h>
#include "stm32f767xx.h"
#include <stdio.h>
#include "SEGGER_RTT.h"

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

/* Pins to communicate over SPI4 (Cf. datasheet, alternate function mapping)
 * PE6  ---> SPI4_MOSI (CN9 pin 20 on NUCLEO F767)
 * PE5  ---> SPI4_MISO (CN9 pin 18 on NUCLEO F767)
 * PE2  ---> SPI4_SCLK (CN9 pin 14 on NUCLEO F767)
 * PE4  ---> SPI4_NSS  (CN9 pin 16 on NUCLEO F767)
 * ALT function mode: 5
 */

void delay(void){
  for (uint32_t i = 0; i < 200000; i++)
    ;
}

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
    SPI_Handle_t SPI4handle;

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}
    // else nack
	return 0;
}


int main(void)
{
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    printf("Application is running\n");


    uint8_t dummy_write = 0xff;
    uint8_t dummy_read;

    GPIO_ButtonInit();

    // initialize the GPIO pins to behave as SPI4 pins
    SPI4_GPIOInits();

    // initialize SPI4 peripheral parameters
    SPI4_Inits();

    printf("SPI Init. done\n");

    // enable the SS output (SSOE)
    // then, when SPE = 1, NSS pulled to Low. When SPE = 0, NSS pulled to High.
    SPI_SSOEConfig(SPI4, ENABLE);

    // enable the FIFO threshold to 8 bits (FRXTH)
    SPI_FRXTHConfig(SPI4, ENABLE);

    while (1)
    {
        //wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        delay();

        // enable the SPI4 peripheral
        SPI_PeripheralControl(SPI4, ENABLE);

	    //1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI4,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI4,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI4,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI4,&ackbyte,1);

        if (SPI_VerifyResponse(ackbyte))
        {
            // send arguments
            args[0] = LED_PIN;
            args[1] = LED_ON;
            SPI_SendData(SPI4, args, 2);
            SPI_ReceiveData(SPI4,args,2);
            printf("COMMAND_LED_CTRL Executed\n");
        }
        // end of CMD_LED_CTRL

        // 2. CMD_SENSOR_READ <analog pin number(1)>
        //wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        delay();

        commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI4,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI4,&dummy_read,1);


		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI4,&dummy_write,1);


		//read the ack byte received
		SPI_ReceiveData(SPI4,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI4,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI4,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI4,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI4,&analog_read,1);
            printf("COMMAND_SENSOR_READ %d\n",analog_read);
			
		}

        //3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI4,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI4,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI4,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI4,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI4,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI4,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI4,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI4,&led_status,1);
            printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI4,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI4,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI4,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI4,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI4,args,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI4,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI4,&message[i],1);
				SPI_ReceiveData(SPI4,&dummy_read,1);
			}

            printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI4,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI4,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI4,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI4,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI4,&dummy_write,1);
				SPI_ReceiveData(SPI4,&id[i],1);
			}

			id[10] = '\0';

            printf("COMMAND_ID : %s \n",id);


		}



        // Let's confirm SPI is not busy
        while(SPI_GetFlagStatus(SPI4, SPI_BUSY_FLAG));

        //Disable the SPI4 peripheral
        SPI_PeripheralControl(SPI4,DISABLE);

        printf("SPI Communication Closed\n");
        
    }

    return 0;

}