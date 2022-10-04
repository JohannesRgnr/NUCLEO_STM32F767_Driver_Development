/**
 * @file 011_i2c_master_rx_testing
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-09-29
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

    // enable the I2C peripheral
    I2C_PeripheralControl(I2C2, ENABLE);

    while (1)
    {
        // wait for button press
        while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        //printf("button pressed\n");

        delay();

        commandcode = 0x51;

        I2C_MasterSendData(&I2C2Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR); // I2C_ENABLE_SR: no stop at the end --> restart

        I2C_MasterReceiveData(&I2C2Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        //printf("data len: %d\n", len);

        commandcode = 0x52;

        I2C_MasterSendData(&I2C2Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C2Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR); // I2C_DISABLE_SR: stop at the end

        printf("buffer: %s", rcv_buf);
        //printf(rcv_buf);


    }
}