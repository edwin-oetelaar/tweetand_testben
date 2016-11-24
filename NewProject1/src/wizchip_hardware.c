#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "my_helpers.h"
#include "wizchip_hardware.h"
#include "wizchip_conf.h"

extern SemaphoreHandle_t xSemaphoreWIZCHIP; // global
extern void SERIAL_puts(const char *s); // global

void init_Wiz_SPI_GPIO(void)
{
    // de wiznet zit op SPI 1

    // en de CS zit op PA13 (out)
    // de Int zit op PA14 (in)
    // de Rdy zit op PA15 (in)
    // de Reset zit op PA12 (out)

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */

    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO
    SPI_InitTypeDef SPI_InitStruct;

    // enable Clocks for APB1 and GPIOA
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* defaults in struct */
    GPIO_StructInit(&GPIO_InitStruct);

    /* set up GPIO pins */

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // Pins 2 (TX) and 3 (RX) are used
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // map de pin naar de SPI functie
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); //
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); //
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); //
    // de CS pin op PA11 is output
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; // Pins 12 RESET en 11 CS
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // de inputs
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15; /*  | GPIO_Pin_10 */ // Pins 10 : Int en Pin 15 : RDY
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // enable the SPI clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); // for SPI1
    // set the SPI parameters to sane defaults
    SPI_StructInit(&SPI_InitStruct);
    // fill in the blanks
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master; // transmit in master mode
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // clock is low when idle
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; // data sampled at first edge
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS to software
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI_BaudRatePrescaler_4; // 56; // SPI frequency is
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB; // data is transmitted MSB first
    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE); // enable SPI1

}

void wiz_chip_select(void)
{
    // PA11 modify for your hardware
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, 0);
}

void wiz_chip_deselect(void)
{
    // PA11 modify for your hardware
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, 1);
}

void wizchip_lock(void)
{
    xSemaphoreTake(xSemaphoreWIZCHIP, 5000);
}

void wizchip_unlock(void)
{
    xSemaphoreGive(xSemaphoreWIZCHIP);
}

/**
* @brief  Reads a byte from the SPI .
* @param  None
* @retval Byte Read from the SPI .
*/

uint8_t WIZ_SPI_ReadByte(void)
{
    return (WIZ_SPI_SendByte(0));
}

/**
* @brief  Sends a byte through the SPI interface and return the byte received
*         from the SPI bus.
* @param  byte: byte to send.
* @retval The value of the received byte.
*/
uint8_t WIZ_SPI_SendByte(uint8_t byte)
{
    /*!< Loop while DR register in not empty */
    while (SPI_I2S_GetFlagStatus(WIZNET_SPI, SPI_I2S_FLAG_TXE) == RESET);

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(WIZNET_SPI, byte);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(WIZNET_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(WIZNET_SPI);
}


uint32_t wiz_hardware_reset_chip(void)
{
    // reset the chip, pull RSn low and release
    GPIO_WriteBit(GPIOA, GPIO_Pin_12, 0); // RESET
    vTaskDelay(1); // minimaal 500 us
    GPIO_WriteBit(GPIOA, GPIO_Pin_12, 1); // not RESET
    // wait for RDY signal to activate
    uint32_t count = 100; // dit duurt normaal 33 ms (max 50 volgens spec) we nemen 100ms als maximum, daarna is er iets mis
    // wachten tot RDY signaal hoog wordt
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == RESET) {
        vTaskDelay(1); // 1 ms
        count--;
        if (count == 0) {
            break;
        }
    }
    // niet op tijd hoog geworden, hardware fail ofzo
    if (count == 0) {
        // timeout in reset van de wizchip handle dit later
        SERIAL_puts("Wiz Reset Timeout");
        return -1; // error
    } else {
        /* dat werkte dus goed */
        wiz_NetTimeout xx = { .retry_cnt = 0x08, .time_100us = 0x07d0 };
        ctlnetwork( CN_SET_TIMEOUT, &xx);
    }
    return 0; // ok
}

