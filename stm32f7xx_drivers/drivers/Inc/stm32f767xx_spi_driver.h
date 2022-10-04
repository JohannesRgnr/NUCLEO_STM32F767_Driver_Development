#ifndef STM32F767XX_SPI_DRIVER_H
#define STM32F767XX_SPI_DRIVER_H


#include "stm32f767xx.h"



/*
 * Configuration structure for SPIx peripheral
 */
typedef struct 
{
    uint8_t SPI_DeviceMode;         
    uint8_t SPI_BusConfig;          
    uint8_t SPI_SclkSpeed;          
    uint8_t SPI_DS;                
    uint8_t SPI_CPOL;               
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t    *pSPIx;       	/*!< This holds the base address of SPIx peripheral >*/
    SPI_Config_t    SPIConfig;      /*!< This holds SPIx configuration settings >*/
    uint8_t         *pTxBuffer;     /*!< stores the TX buffer address >*/
    uint8_t         *pRxBuffer;     /*!< stores the RX buffer address >*/
    uint32_t        TxLen;          /*!< stores Tx len >*/
    uint32_t        RxLen;          /*!< stores Rx len >*/
    uint8_t         TxState;        /*!< stores Tx state >*/
    uint8_t         RxState;        /*!< stores Rx state >*/
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER          1
#define SPI_DEVICE_MODE_SLAVE           0

/*
 * @SPI_BusConfig 
 * !TX only is actually full duplex with MOSI only.. so no option simplex TXonly needed.
 */
#define SPI_BUS_CONFIG_FD               1
#define SPI_BUS_CONFIG_HD               2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             0
#define SPI_SCLK_SPEED_DIV4             1
#define SPI_SCLK_SPEED_DIV8             2
#define SPI_SCLK_SPEED_DIV16            3
#define SPI_SCLK_SPEED_DIV32            4
#define SPI_SCLK_SPEED_DIV64            5
#define SPI_SCLK_SPEED_DIV128           6
#define SPI_SCLK_SPEED_DIV256           7

/*
 * @SPI_DS
 * STM32F767: more options are possible than 8 bits / 16 bits
 */
#define SPI_DS_4BITS                  3
#define SPI_DS_5BITS                  4
#define SPI_DS_6BITS                  5
#define SPI_DS_7BITS                  6
#define SPI_DS_8BITS                  7
#define SPI_DS_9BITS                  8
#define SPI_DS_10BITS                 9
#define SPI_DS_11BITS                 10
#define SPI_DS_12BITS                 11
#define SPI_DS_13BITS                 12
#define SPI_DS_14BITS                 13
#define SPI_DS_15BITS                 14
#define SPI_DS_16BITS                 15

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH                   1
#define SPI_CPOL_LOW                    0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH                   1
#define SPI_CPHA_LOW                    0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN                      1
#define SPI_SSM_DI                      0

/*
 * SPI status flags definitions
 */
#define SPI_TXE_FLAG                    (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                   (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG                   (1 << SPI_SR_BSY)

/*
 * SPI application states 
 */
#define SPI_READY                       0
#define SPI_BUSY_IN_RX                  1
#define SPI_BUSY_IN_TX                  2

/*
 * SPI possible application events
 */
#define SPI_EVENT_TX_CMPLT              1
#define SPI_EVENT_RX_CMPLT              2
#define SPI_EVENT_OVR_ERR               3
#define SPI_EVENT_CRC_ERR               4


/*********************************************************************************
 *                  APIs supported by this driver
 *      For more infos, check the function definitions
 *********************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ config and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * SPI Peripheral Control
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_FRXTHConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif // !STM32F767XX_SPI_DRIVER_H