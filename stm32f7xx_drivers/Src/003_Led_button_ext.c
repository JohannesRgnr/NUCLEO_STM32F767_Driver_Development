
#include "stm32f767xx.h"

#define HIGH            1
#define LOW             0
#define BTN_PRESSED     LOW

void delay(void){
  for (uint32_t i = 0; i < 200000; i++)
    ;
}

int main(void){

    GPIO_Handle_t GpioLed, GPIOBtn;

    GpioLed.pGPIOx = GPIOB; // LED1 is connected to PB0 (Cf. Nucleo User Manual, or check schematics)
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&GpioLed);

    GPIOBtn.pGPIOx = GPIOD; // ext button is connected to PD5 (connector CN9, pin 6, cf. Nucleo User Manual)
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // internal pull up resistor

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GPIOBtn);

    while (1)
    {
        if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5 ) == BTN_PRESSED)
        {
            delay();
            GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
        }
  }

  return 0;
  
}