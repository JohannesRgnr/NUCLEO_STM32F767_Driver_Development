/**
 * @file 012_i2c_master_rx_testing_it.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-09-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <string.h>
#include "stm32f767xx.h"
#include <stdio.h>
#include "SEGGER_RTT.h"


#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

uint8_t rxComplt = RESET;

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


// receive buffer
uint8_t rcv_buf[32];

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

    uint8_t commandcode;
    uint8_t len;

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

    while (1)
    {
        // wait for button press
        while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        delay();

        commandcode = 0x51;

        while (I2C_MasterSendDataIT(&I2C2Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        while (I2C_MasterReceiveDataIT(&I2C2Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        while (rxComplt != SET);

        printf("len: %d\n", len);

        rxComplt = RESET;

        commandcode = 0x52;

        while (I2C_MasterSendDataIT(&I2C2Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        while (I2C_MasterReceiveDataIT(&I2C2Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR)!= I2C_READY); 

        while (rxComplt != SET);

        printf("buffer: %s\n", rcv_buf);

        rxComplt = RESET;
    }
}




void I2C2_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C2Handle);
}

void I2C2_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C2Handle);
}   


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
    if(AppEvent == I2C_EV_TX_CMPLT)
    {
        //printf("Transmission complete \n");
    }else if (AppEvent == I2C_EV_RX_CMPLT)
    {
        //printf("Reception complete \n");
        rxComplt = SET;
    }
}
