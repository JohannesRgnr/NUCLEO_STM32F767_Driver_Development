/**
 * @file 014_i2c_slave_tx_string2.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-10-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <string.h>
#include "stm32f767xx.h"
#include <stdio.h>
#include "SEGGER_RTT.h"


#define MY_ADDR     0x68



void delay(void){
  for (uint32_t i = 0; i < 200000; i++)
    ;
}



/* Pins to communicate over I2C1 (Cf. datasheet, alternate function mapping)
 * PB8  ---> I2C1_SCL (CN7 pin 2 on NUCLEO F767)
 * PB9  ---> I2C1_SDA (CN7 pin 4 on NUCLEO F767)
 * ALT function mode: 4
 */

/* Pins to communicate over I2C2 (Cf. datasheet, alternate function mapping)
 * PF1  ---> I2C2_SCL (CN9 pin 19 on NUCLEO F767)
 * PF0  ---> I2C2_SDA (CN9 pin 21 on NUCLEO F767)
 * ALT function mode: 4
 */



I2C_Handle_t I2C2Handle;


// tx buffer
uint8_t Tx_buf[] = "STM32 slave mode testing with long message....STM32 slave mode testing with long message....STM32 slave mode testing with long message....STM32 slave mode testing with long message....STM32 slave mode testing with long message....STM32 slave mode testing with long message....STM32 slave mode testing with long message....STM32 slave mode testing with long message....";

uint32_t data_len=0;


void I2C2_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;
    I2CPins.pGPIOx = GPIOF;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_Init(&I2CPins);

    //SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Init(&I2CPins);

}

void I2C2_Inits()
{
    I2C2Handle.pI2Cx = I2C2;
    I2C2Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; // only needed if slave mode (@see protocol reference for reserved addresses)
    I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM100K;

    I2C_Init(&I2C2Handle);
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

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    printf("Application is running\n");

    data_len = strlen((char*)Tx_buf);

    // GPIO button init
    GPIO_ButtonInit();

    // I2C pin inits
    I2C2_GPIOInits();

    // I2C peripheral configuration
    I2C2_Inits();

    // I2C IRQ config
    I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV, ENABLE);
    // I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER, ENABLE);

    

    // enable the I2C peripheral
    I2C_PeripheralControl(I2C2, ENABLE);

    I2C_SlaveEnableDisableCallbackEvents(I2C2, ENABLE);

    while (1);
}




void I2C2_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C2Handle);
}

void I2C2_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C2Handle);
}

uint8_t commandCode = 0;

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
    static uint8_t commandCode = 0;
    static uint8_t cnt = 0;
    static uint32_t w_ptr = 0;

    if(AppEvent == I2C_EV_DATA_REQ)
    {
        //Master is requesting for the data . send data
		if(commandCode == 0x51)
		{
			//Here we are sending 4 bytes of length information
			I2C_SlaveSendData(I2C2,((data_len >> ((cnt%4) * 8)) & 0xFF));
		    cnt++;
		}else if (commandCode == 0x52)
		{
			//sending Tx_buf contents indexed by w_ptr variable
			I2C_SlaveSendData(I2C2,Tx_buf[w_ptr++]);
		}
    }else if(AppEvent == I2C_EV_DATA_RCV)
    {
        // data waiting to be read by the slave
        commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

    }else if(AppEvent == I2C_EV_STOP)
    {
        // master has ended I2C communication
    }else if(AppEvent == I2C_EV_NACK)
    {
        //This happens only during slave txing .
		//Master has sent the NACK. so slave should understand that master doesnt need
		//more data.
		//if the current active code is 0x52 then dont invalidate
		if(! (commandCode == 0x52))
			commandCode = 0XFF;

		//reset the cnt variable because its end of transmission
		cnt = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(w_ptr >= (data_len))
		{
			w_ptr=0;
			commandCode = 0xff;
		}
    }
}
