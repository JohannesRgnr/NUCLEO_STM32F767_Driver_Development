#ifndef STM32F767XX_H
#define STM32F767XX_H

#include<stddef.h>
#include<stdint.h>



#define __vo volatile
#define __weak __attribute__((weak))




/***************************START: Processor Specific Details****************************/

/*
 * ARM Cortex Mx NVIC ISERx registers (cf. generic user guide)
 */

#define NVIC_ISER0                  ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                  ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                  ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                  ((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex Mx NVIC ICERx registers (cf. generic user guide)
 */

#define NVIC_ICER0                  ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                  ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                  ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                  ((__vo uint32_t*)0xE000E18C)


/*
 * ARM Cortex Mx NVIC Priority registers (cf. generic user guide)
 */

#define NVIC_PR_BASEADDR            ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED      4 // 4 bits of interrupt priority are used. Cf. RM 10.1

/*
 * base addresses of FLASH and SRAM memories
 */

#define FLASH_BASEADDR              0x08000000U 
#define SRAM1_BASEADDR              0x20000000U
#define SRAM2_BASEADDR              0x2007C000U
#define ROM                         0x1FF00000U // cf. Ref. manual 3.3.1
#define SRAM                        SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR             0x40000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR         0x40010000U
#define AHB1PERIPH_BASEADDR         0x40020000U
#define AHB2PERIPH_BASEADDR         0x50000000U

/*
 * base addresses of peripherals hanging on AHB1 bus
 */

#define GPIOA_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR              (AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR              (AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR              (AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

                      

/*
 * base addresses of peripherials hanging on APB1 bus
 */

#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)
#define I2C4_BASEADDR               (APB1PERIPH_BASEADDR + 0x6000)
#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR              (APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR              (APB1PERIPH_BASEADDR + 0x7C00)

/*
 * base addresses of peripherials hanging on APB2 bus
 */

#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR               (APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR               (APB2PERIPH_BASEADDR + 0x5400)
#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)


/************************ peripheral register definition structures ***********************/

/**
 * @brief peripheral register definition for GPIO
 * 
 */
typedef struct 
{
    __vo uint32_t MODER;             // GPIO Mode register
    __vo uint32_t OTYPER;            // GPIO Output type register
    __vo uint32_t OSPEEDR;           // GPIO Speed register
    __vo uint32_t PUPDR;             // GPIO pull-up/pull-dowm register
    __vo uint32_t IDR;               // GPIO Input data register
    __vo uint32_t ODR;               // GPIO Output data register
    __vo uint32_t BSRR;              // GPIO bit set/reset register
    __vo uint32_t LCKR;              // GPIO Configuration lock register
    __vo uint32_t AFR[2];            // AFR[0]: GPIO Alternate function low register, AFR[1]: GPIO Alternate function high register
}GPIO_RegDef_t;

/**
 * @brief peripheral register definition for RCC
 * 
 */
typedef struct 
{
    __vo uint32_t CR;               // clock control register             
    __vo uint32_t PLLCFGR;          // PLL configuration register
    __vo uint32_t CFGR;             // clock configuration register 
    __vo uint32_t CIR;              // clock interrupt register 
    __vo uint32_t AHB1RSTR;         // AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;         // AHB2 peripheral reset register
    __vo uint32_t AHB3RSTR;         // AHB3 peripheral reset register
    __vo uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;         // APB1 peripheral reset register
    __vo uint32_t APB2RSTR;         // APB2 peripheral reset register
    __vo uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;          // AHB1 peripheral clock register
    __vo uint32_t AHB2ENR;          // AHB2 peripheral clock register
    __vo uint32_t AHB3ENR;          // AHB3 peripheral clock register
    __vo uint32_t RESERVED2;
    __vo uint32_t APB1ENR;          // APB1 peripheral clock enable register
    __vo uint32_t APB2ENR;          // APB2 peripheral clock enable register
    __vo uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;        // AHB1 peripheral clock enable in low-power mode register            
    __vo uint32_t AHB2LPENR;        // AHB2 peripheral clock enable in low-power mode register
    __vo uint32_t AHB3LPENR;        // AHB3 peripheral clock enable in low-power mode register
    __vo uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;        // APB1 peripheral clock enable in low-power mode register
    __vo uint32_t APB2LPENR;        // APB2 peripheral clock enable in low-power mode register
    __vo uint32_t RESERVED5[2];
    __vo uint32_t BDCR;             // backup domain control register
    __vo uint32_t CSR;              // clock control & status register 
    __vo uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;            // spread spectrum clock generation register 
    __vo uint32_t PLLI2SCFGR;       // PLLI2S configuration register
    __vo uint32_t PLLSAICFGR;       // PLLSAI configuration register
    __vo uint32_t DCKCFGR1;         // dedicated clocks configuration register
    __vo uint32_t DCKCFGR2;         // dedicated clocks configuration register
} RCC_RegDef_t;

/**
 * @brief peripheral register definition for EXTI
 * 
 */
typedef struct
{
    __vo uint32_t IMR;               // EXTI Interrupt mask register
    __vo uint32_t EMR;               // EXTI Event mask register
    __vo uint32_t RTSR;              // EXTI Rising trigger selection register
    __vo uint32_t FTSR;              // EXTI Falling trigger selection register
    __vo uint32_t SWIER;             // EXTI Software interrupt event register
    __vo uint32_t PR;                // EXTI Pending register
} EXTI_RegDef_t;

/**
 * @brief peripheral register definition for SPI
 * 
 */
typedef struct
{
	__vo uint32_t CR1;              // SPI Control Register 1
	__vo uint32_t CR2;              // SPI Control Register 2
	__vo uint32_t SR;               // SPI Status Register
	__vo uint32_t DR;               // SPI Data Register
	__vo uint32_t CRCPR;            // SPI CRC Polynomial Register
	__vo uint32_t RXCRCR;           // SPI Rx CRC Register
	__vo uint32_t TXCRCR;           // SPI Tx CRC Register
	__vo uint32_t I2SCFGR;          // SPI I2S Configuration Register
	__vo uint32_t I2SPR;            // SPI I2S Prescaler Register
} SPI_RegDef_t;

/**
 * @brief peripheral register definition for I2C
 * 
 */
typedef struct
{
    __vo uint32_t CR1;              // I2C Control Register 1
    __vo uint32_t CR2;              // I2C Control Register 2
    __vo uint32_t OAR1;             // I2C Own address 1 register  
    __vo uint32_t OAR2;             // I2C Own address 1 register
    __vo uint32_t TIMINGR;          // I2C Timing register
    __vo uint32_t TIMEOUTR;         // I2C Timeout register
    __vo uint32_t ISR;              // I2C Interrupt and Status register
    __vo uint32_t ICR;              // I2C Interrupt clear register
    __vo uint32_t PECR;             // I2C PEC register
    __vo uint32_t RXDR;             // I2C Receive data register
    __vo uint32_t TXDR;             // I2C Transmit data register
} I2C_RegDef_t;

/**
 * @brief peripheral register definition for SYSCFG
 * 
 */
typedef struct
{
    __vo uint32_t MEMRMP;           // SYSCFG memory remap register
    __vo uint32_t PMC;              // SYSCFG peripheral mode configuration register 
    __vo uint32_t EXTICR[4];        // SYSCFG external interrupt configuration registers 1
    uint32_t      RESERVED;         // address: 0x18
    __vo uint32_t CBR;              // Class B register
    __vo uint32_t CMPCR;            // Compensation cell control register
} SYSCFG_RegDef_t;


/**
 * @brief peripheral register definition for USART
 * 
 */
typedef struct
{
    __vo uint32_t CR1;              // USART Control Register 1
    __vo uint32_t CR2;              // USART Control Register 2
    __vo uint32_t CR3;              // USART Control Register 3 
    __vo uint32_t BRR;              // USART Baud rate register
    __vo uint32_t GTPR;             // USART Guard time and prescaler register
    __vo uint32_t RTOR;             // USART Receiver timeout register
    __vo uint32_t RQR;              // USART Request register
    __vo uint32_t ISR;              // USART Interrupt and status register
    __vo uint32_t ICR;              // USART Interrupt flag clear register 
    __vo uint32_t RDR;              // USART Receive data register
    __vo uint32_t TDR;              // USART Transmit data register
} USART_RegDef_t;


/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA                       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                       ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                       ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                       ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                       ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                       ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI                       ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ                       ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK                       ((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC                         ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI                        ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG                      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1                        ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                        ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                        ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                        ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5                        ((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6                        ((SPI_RegDef_t*)SPI6_BASEADDR)

#define I2C1                        ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                        ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                        ((I2C_RegDef_t*)I2C3_BASEADDR)
#define I2C4                        ((I2C_RegDef_t*)I2C4_BASEADDR)

#define USART1                      ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2                      ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3                      ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4                       ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5                       ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6                      ((USART_RegDef_t*)USART6_BASEADDR)
#define UART7                       ((USART_RegDef_t*)UART7_BASEADDR)
#define UART8                       ((USART_RegDef_t*)UART8_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()       ( RCC->AHB1ENR |= (1 << 10) )


/*
 * Clock Enable Macros for I2CX peripherals
 */

#define I2C1_PCLK_EN()       ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()       ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()       ( RCC->APB1ENR |= (1 << 23) )
#define I2C4_PCLK_EN()       ( RCC->APB1ENR |= (1 << 24) )



/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()       ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()       ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()       ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()       ( RCC->APB2ENR |= (1 << 13) )
#define SPI5_PCLK_EN()       ( RCC->APB2ENR |= (1 << 20) )
#define SPI6_PCLK_EN()       ( RCC->APB2ENR |= (1 << 21) )


/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()       ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()       ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()       ( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()        ( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()        ( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()       ( RCC->APB2ENR |= (1 << 5) )
#define UART7_PCLK_EN()        ( RCC->APB1ENR |= (1 << 30) )
#define UART8_PCLK_EN()        ( RCC->APB1ENR |= (1 << 31) )

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()       ( RCC->APB2ENR |= (1 << 14) )


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 0) )     // clear bit
#define GPIOB_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()       ( RCC->AHB1ENR &= ~(1 << 10) )


/*
 * Clock Disable Macros for I2CX peripherals
 */

#define I2C1_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 23) )
#define I2C4_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 24) )


/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 13) )
#define SPI5_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 20) )
#define SPI6_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 21) )

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()       ( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()        ( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()        ( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 5) )
#define UART7_PCLK_DI()        ( RCC->APB1ENR &= ~(1 << 30) )
#define UART8_PCLK_DI()        ( RCC->APB1ENR &= ~(1 << 31) )

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()       ( RCC->APB2ENR &= ~(1 << 14) )


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0) // set then reset using do/while(0) loop
#define GPIOB_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)
#define GPIOJ_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9));}while(0)
#define GPIOK_REG_RESET()      do{(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10));}while(0)


/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)
#define SPI6_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)


/*
 * returns port code for given GPIOx address
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
                                    (x == GPIOB) ? 1 :\
                                    (x == GPIOC) ? 2 :\
                                    (x == GPIOD) ? 3 :\
                                    (x == GPIOE) ? 4 :\
                                    (x == GPIOF) ? 5 :\
                                    (x == GPIOG) ? 6 :\
                                    (x == GPIOH) ? 7 :\
                                    (x == GPIOI) ? 8 :\
                                    (x == GPIOJ) ? 9 :\
                                    (x == GPIOK) ? 10 : 0)


/*
 * IRQ Numbers of STM32F767x MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4         84
#define IRQ_NO_SPI5         85
#define IRQ_NO_SPI6         86

#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER      73
#define IRQ_NO_I2C4_EV      95
#define IRQ_NO_I2C4_ER      96

#define IRQ_NO_USART1       37
#define IRQ_NO_USART2       38
#define IRQ_NO_USART3       39
#define IRQ_NO_UART4        52
#define IRQ_NO_UART5        53
#define IRQ_NO_USART6       71
#define IRQ_NO_UART7        82
#define IRQ_NO_UART8        83



/*
 * IRQ possible priority levels
 */

#define NVIC_IRQ_PRIO0      0
#define NVIC_IRQ_PRIO1      1
#define NVIC_IRQ_PRIO2      2
#define NVIC_IRQ_PRIO3      3
#define NVIC_IRQ_PRIO4      4
#define NVIC_IRQ_PRIO5      5
#define NVIC_IRQ_PRIO6      6
#define NVIC_IRQ_PRIO7      7
#define NVIC_IRQ_PRIO8      8
#define NVIC_IRQ_PRIO9      9
#define NVIC_IRQ_PRIO10     10
#define NVIC_IRQ_PRIO11     11
#define NVIC_IRQ_PRIO12     12
#define NVIC_IRQ_PRIO13     13
#define NVIC_IRQ_PRIO14     14    
#define NVIC_IRQ_PRIO15     15


// some generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET        SET
#define FLAG_RESET      RESET



/***********************************************************************************************
 *                      Bit position definitions of SPI peripheral
 ***********************************************************************************************/
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_CRCL        11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_NSSP        3
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7
#define SPI_CR2_DS          8
#define SPI_CR2_FRXTH       12
#define SPI_CR2_LDMA_RX     13
#define SPI_CR2_LDMA_TX     14

#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8
#define SPI_SR_FRLVL        9
#define SPI_SR_FTLVL        11



/***********************************************************************************************
 *                      Bit position definitions of I2C peripheral
 ***********************************************************************************************/
#define I2C_CR1_PE          0
#define I2C_CR1_TXIE        1
#define I2C_CR1_RXIE        2
#define I2C_CR1_ADDRIE      3
#define I2C_CR1_NACKIE      4
#define I2C_CR1_STOPIE      5
#define I2C_CR1_TCIE        6
#define I2C_CR1_ERRIE       7
#define I2C_CR1_DNF         8
#define I2C_CR1_ANFOFF      12
#define I2C_CR1_TXDMAEN     14
#define I2C_CR1_RXDMAEN     15
#define I2C_CR1_SBC         16
#define I2C_CR1_NOSTRETCH   17
#define I2C_CR1_GCEN        19
#define I2C_CR1_SMBHEN      20
#define I2C_CR1_SMBDEN      21
#define I2C_CR1_ALERTEN     22
#define I2C_CR1_PECEN       23

#define I2C_CR2_SADD        0
#define I2C_CR2_RD_WRN      10
#define I2C_CR2_ADD10       11
#define I2C_CR2_HEAD10R     12
#define I2C_CR2_START       13
#define I2C_CR2_STOP        14
#define I2C_CR2_NACK        15
#define I2C_CR2_NBYTES      16
#define I2C_CR2_RELOAD      24
#define I2C_CR2_AUTOEND     25
#define I2C_CR2_PECBYTE     26

#define I2C_OAR1_OA1_10BIT  0
#define I2C_OAR1_OA1_7BIT   1
#define I2C_OAR1_OA1MODE    10
#define I2C_OAR1_OA1EN      15

#define I2C_TIMINGR_SCLL    0
#define I2C_TIMINGR_SCLH    8
#define I2C_TIMINGR_SDADEL  16
#define I2C_TIMINGR_SCLDEL  20
#define I2C_TIMINGR_PRESC   28

#define I2C_ISR_TXE         0
#define I2C_ISR_TXIS        1
#define I2C_ISR_RXNE        2
#define I2C_ISR_ADDR        3
#define I2C_ISR_NACKF       4
#define I2C_ISR_STOPF       5
#define I2C_ISR_TC          6
#define I2C_ISR_TCR         7
#define I2C_ISR_BERR        8
#define I2C_ISR_ARLO        9
#define I2C_ISR_OVR         10
#define I2C_ISR_PECERR      11
#define I2C_ISR_TIMEOUT     12
#define I2C_ISR_ALERT       13
#define I2C_ISR_BUSY        15
#define I2C_ISR_DIR         16
#define I2C_ISR_ADDCODE     17

#define I2C_ICR_ADDRCF      3
#define I2C_ICR_NACKCF      4
#define I2C_ICR_STOPCF      5
#define I2C_ICR_BERRCF      8
#define I2C_ICR_ARLOCF      9
#define I2C_ICR_OVRCF       10
#define I2C_ICR_PECCF       11
#define I2C_ICR_TIMEOUTCF   12
#define I2C_ICR_ALERTCF     13



/***********************************************************************************************
 *                      Bit position definitions of USART peripheral
 ***********************************************************************************************/
#define USART_CR1_UE        0
#define USART_CR1_UESM      1
#define USART_CR1_RE        2
#define USART_CR1_TE        3
#define USART_CR1_IDLEIE    4
#define USART_CR1_RXNEIE    5
#define USART_CR1_TCIE      6
#define USART_CR1_TXEIE     7
#define USART_CR1_PEIE      8
#define USART_CR1_PS        9
#define USART_CR1_PCE       10
#define USART_CR1_WAKE      11
#define USART_CR1_M0        12
#define USART_CR1_MME       13
#define USART_CR1_CMIE      14
#define USART_CR1_OVER8     15
#define USART_CR1_DEDT      16
#define USART_CR1_DEAT      21
#define USART_CR1_RTOIE     26
#define USART_CR1_EOBIE     27
#define USART_CR1_M1        28

#define USART_CR2_ADDM7     4
#define USART_CR2_LBDL      5
#define USART_CR2_LBDIE     6
#define USART_CR2_LBCL      8
#define USART_CR2_CPHA      9
#define USART_CR2_CPOL      10
#define USART_CR2_CLKEN     11
#define USART_CR2_STOP      12
#define USART_CR2_LINEN     14
#define USART_CR2_SWAP      15
#define USART_CR2_RXINV     16
#define USART_CR2_TXINV     17
#define USART_CR2_DATAINV   18
#define USART_CR2_MSBFIRST  19
#define USART_CR2_ABREN     20
#define USART_CR2_ABRMOD    21
#define USART_CR2_RTOEN     23
#define USART_CR2_ADD_3_0   24
#define USART_CR2_ADD_7_4   28

#define USART_CR3_EIE       0
#define USART_CR3_IREN      1
#define USART_CR3_IRLP      2
#define USART_CR3_HDSEL     3
#define USART_CR3_NACK      4
#define USART_CR3_SCEN      5
#define USART_CR3_DMAR      6
#define USART_CR3_DMAT      7
#define USART_CR3_RTSE      8
#define USART_CR3_CTSE      9
#define USART_CR3_CTSIE     10
#define USART_CR3_ONEBIT    11
#define USART_CR3_OVRDIS    12
#define USART_CR3_DDRE      13
#define USART_CR3_DEM       14
#define USART_CR3_DEP       15
#define USART_CR3_SCARCNT0  17
#define USART_CR3_SCARCNT1  18
#define USART_CR3_SCARCNT2  19
#define USART_CR3_WUS0      20
#define USART_CR3_WUS1      21
#define USART_CR3_WUFIE     22
#define USART_CR3_UCESM     23
#define USART_CR3_TCBGTIE   24

#define USART_RQR_ABRRQ     0
#define USART_RQR_SBKRQ     1
#define USART_RQR_MMRQ      2
#define USART_RQR_RXFRQ     3
#define USART_RQR_TXFRQ     4

#define USART_ISR_PE        0
#define USART_ISR_FE        1
#define USART_ISR_NF        2
#define USART_ISR_ORE       3
#define USART_ISR_IDLE      4
#define USART_ISR_RXNE      5
#define USART_ISR_TC        6
#define USART_ISR_TXE       7
#define USART_ISR_LBDF      8
#define USART_ISR_CTSIF     9
#define USART_ISR_CTS       10
#define USART_ISR_RTOF      11
#define USART_ISR_EOBF      12
#define USART_ISR_ABRE      14
#define USART_ISR_ABRF      15
#define USART_ISR_BUSY      16
#define USART_ISR_CMF       17
#define USART_ISR_SBKF      18
#define USART_ISR_RWU       19
#define USART_ISR_WUF       20
#define USART_ISR_TEACK     21
#define USART_ISR_REACK     22
#define USART_ISR_TCBGT     25

#define USART_ICR_PECF      0
#define USART_ICR_FECF      1
#define USART_ICR_NCF       2
#define USART_ICR_ORECF     3
#define USART_ICR_IDLECF    4
#define USART_ICR_TCCF      6
#define USART_ICR_TCBGTCF   7
#define USART_ICR_LBDCF     8
#define USART_ICR_CTSCF     9
#define USART_ICR_RTOCF     11
#define USART_ICR_EOBCF     12
#define USART_ICR_CMCF      17
#define USART_ICR_WUCF      20







// These have to be at the end!!! because it needs the macros and definitions of this current header
#include "stm32f767xx_gpio_driver.h" 
#include "stm32f767xx_rcc_driver.h"
#include "stm32f767xx_spi_driver.h"
#include "stm32f767xx_i2c_driver.h"
#include "stm32f767xx_usart_driver.h"

#endif // !STM32F767XX_H
