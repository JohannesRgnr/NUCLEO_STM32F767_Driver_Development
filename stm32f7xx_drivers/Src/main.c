
#include "stm32f767xx.h"
// #include "stm32f767xx_gpio_driver.h"

void delay(void){
  for (uint32_t i = 0; i < 500000; i++)
    ;
}

int main(void){

  GPIO_Handle_t GpioLed;

  GpioLed.pGPIOx = GPIOB;
  GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
  GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
  GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

  GPIO_PeriClockControl(GPIOB, ENABLE);
  GPIO_Init(&GpioLed);

  while(1)
  {
    GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
    delay();
  }

  return 0;
  
}