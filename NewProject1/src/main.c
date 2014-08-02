
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "my_queue.h"
#include "LCDshield.h"
#include "analog_input.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "dhcp.h"
#include "dns.h"
#include "my_helpers.h"
#include "term_io.h"
#include "vs10xx.h"

#define min(a,b) (((a)<(b))?(a):(b))

uint8_t SPI_ReadByte(void);
uint8_t SPI_SendByte(uint8_t byte);
uint8_t do_http_get(uint8_t sn, const char *url, void (writefunc)(const char *buf, uint32_t len));

struct {
    queue_hdr_t hdr; // must be named "hdr"
    uint8_t items[256]; // must be named "items", 1 space wasted
} my_TX_queue;

struct {
    queue_hdr_t hdr; // must be named "hdr"
    uint8_t items[128]; // must be named "items", 1 space wasted
} my_RX_queue;

struct {
    queue_hdr_t hdr;
    uint8_t items[16]; // key buffer
} my_KEYS_queue;

lcd_context_t LCD; // make a context

EventGroupHandle_t xEventBits; // set by dhcp task when network is up and running, bit 0x01 is NETWORK-OK
SemaphoreHandle_t xSemaphoreSPI2;
SemaphoreHandle_t xSemaphoreWRAM; // WRAM is mem inside VS1063, must be protected, needs many read writes to registers, may not be interrupted
SemaphoreHandle_t xSemaphoreWIZCHIP;

// freetronicsLCDShield lcdshield(D8, D9, D4, D5, D6, D7, D3, A0);
// RS  E   D0   D1  D2  D3   BL A0
// PA9 PC7 PB5 PB4 PB10 PA8 PB6

/* stop een string in de output buffer van de serial poort
de interrupt handler zorgt ervoor dat de buffer vanzelf
verstuurd wordt zonder dat dit tijd kost in het programma
Als de buffer vol is worden de tekens zomaar weggegooid, jammer dan.
 */
void SERIAL_puts(const char *s)
{
    while (*s) {
        if (!QUEUE_FULL(my_TX_queue)) {
            // er is ruimte, stop teken in de queue
            QUEUE_PUT(my_TX_queue, *s);
            // volgende teken
            s++;
        } else {
            // buffer vol, we moeten wat anders doen nu
            // deze functie blockt nu
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
            vTaskDelay(0);
        }
    }
    // de interrupt handler voor Transmit register Empty aanzetten
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

void SERIAL_write(const char *s, uint32_t len)
{
    while (len) {
        if (!QUEUE_FULL(my_TX_queue)) {
            // er is ruimte, stop teken in de queue
            QUEUE_PUT(my_TX_queue, *s);
            // volgende teken
            s++;
            len--;
        } else {
            // buffer vol, we moeten wat anders doen nu
            // deze functie blockt nu
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
            vTaskDelay(0);
        }
    }
    // de interrupt handler voor Transmit register Empty aanzetten
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART1->DR;
        // SERIAL_write(&ch,1);
    }
}

// this is the interrupt request handler (IRQ) for ALL USART2 interrupts
void USART2_IRQHandler(void)
{
    // check if the USART receive interrupt flag was set
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART2->DR; // the character from the USART data register is saved in ch
        // als de buffer niet vol dan erbij proppen, anders jammer dan, weg ermee
        if (!QUEUE_FULL(my_RX_queue)) {
            QUEUE_PUT(my_RX_queue, ch);
        }
    }
    // moeten we wat zenden, TX empty interrupt?
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        uint8_t ch;
        if (!QUEUE_EMPTY(my_TX_queue)) {
            QUEUE_GET(my_TX_queue, ch);
            USART_SendData(USART2, ch);
        } else {
            // no data in buf, disable Transmit Data Register empty interrupt
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
    // Clear all flags
    // USART_ClearITPendingBit(USART2, (USART_IT_CTS | USART_IT_LBD | USART_IT_TC | USART_IT_RXNE));
}

/* USART 1 gaat aan de VS1063 hangen, hiermee komt data van de encoder binnen op interrupt basis, mooi he */




static void vTask1(void *arg)
{
    uint32_t x = 0;
    for (;;) {
        vTaskDelay(100);
        x = !x;
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, x); // LED pin
    }
}

#if 0

/* handler for all the ADC interrupts */
void ADC_IRQHandler(void)
{
    static uint32_t cnt = 0;
    char buf[20];
    // read a value from the ADC keyboard and compare to last
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        uint16_t val = ADC1->DR;
        uint8_t key = val2key(val);
        int i = sprintf(buf, "%5" PRIu32 " %5d %c", cnt, val, key);
        cnt++;
        lcd_home(&LCD);
        lcd_write(&LCD, buf, i);
    }
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}
#endif

static void vTask2(void *arg)
{
    lcd_init_context(&LCD); // put values in the context
    lcd_init_gpio(&LCD); // init the gpio
    // lcd_write(&LCD,"aaap",4); // write a string
    int x=10;
    while (x--) {
        lcd_set_backlight(&LCD, 0);
        vTaskDelay(200);
        lcd_set_backlight(&LCD, 1);
        vTaskDelay(200);
    }

    uint32_t cnt = 0;
    char buf[20];
    for (;;) {
        uint16_t val = adc_convert();
        uint8_t key = val2key(val); // lookup de ADC naar key
        int i = sprintf(buf, "%5" PRIu32 " x %5" PRIu16 " %c", cnt, val, key);
        cnt++;
        lcd_home(&LCD);
        lcd_write(&LCD, buf, i);
        vTaskDelay(100);
        //vTaskDelay(90);
        //x = !x;
        //GPIO_WriteBit(GPIOB, GPIO_Pin_6, x);
    }
}

static uint32_t wiz_hardware_reset_chip()
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
    }
    return 0; // ok
}

void GPIO_Init_edwin(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    // pin B-6 dit wordt TX voor Serial USART 1
    //GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    //GPIO_Init(GPIOB, &GPIO_InitStructure);

    // pin A-5
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Data/Control pin PC4 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Chip Select (CS) van ILI9341 pin PB1 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/* USART 1 aan de Vs1063, pins PB6 en PB7 */
void init_USART1(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART2 initialization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

    // enable Clocks for APB2 (dus niet APB1) and GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ok
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // ok

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct); // poort B B B B B B

    //Connect to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
    //disable Transmit Data Register empty interrupt
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE); //Enable USART1
}

/* debug info op USART 2 */
void init_USART2(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART2 initialization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

    // enable Clocks for APB1 and GPIOA
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 2 (TX) and 3 (RX) are used
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    //Connect to AF
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt
    //disable Transmit Data Register empty interrupt
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE); //Enable USART2
}



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
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_10; // Pins 10 : Int en Pin 15 : RDY
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
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // 56; // SPI frequency is
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB; // data is transmitted MSB first
    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE); // enable SPI1

}



void wiz_chip_select(void)
{
    // PA11
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, 0);
}

void wiz_chip_deselect(void)
{
    // PA11
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, 1);
}

// volatile int32_t lockcounter =0;

void wizchip_lock(void)
{
    //  lockcounter++;
    //  xprintf("X=%d\r\n",lockcounter);
    xSemaphoreTake(xSemaphoreWIZCHIP,5000);
}

void wizchip_unlock(void)
{
    //  lockcounter--;
    //  xprintf("Y=%d\r\n",lockcounter);

    //  xprintf("Y\r\n");
    xSemaphoreGive(xSemaphoreWIZCHIP);
}

/**
 * @brief  Reads a byte from the SPI .
 * @note   This function must be used only if the Start_Read_Sequence function
 *         has been previously called.
 * @param  None
 * @retval Byte Read from the SPI .
 */

uint8_t SPI_ReadByte(void)
{
    return (SPI_SendByte(0));
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @retval The value of the received byte.
 */
uint8_t SPI_SendByte(uint8_t byte)
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


static void vTaskDHCP(void *arg)
{
    /* de netwerk config moet uit een eeprom komen, mogeljk later een 24c32 erbij bakken op de i2c bus */

    uint32_t netconfig = NETINFO_DHCP; /* NETINFO_STATIC */
    uint8_t old_ip_address[4] = {0};   /* het IP van de PIC */
    uint8_t old_mac_address[6] = {0};  /* het MAC van de PIC */


    /* netwerk buffer sizes per socket */
    uint8_t buffsizes[2][8] = {
        {2, 2, 8, 2, 2, 0, 0, 0},
        {2, 2, 8, 2, 2, 0, 0, 0}
    };

    wiz_NetInfo ni = { // new network info, we copy the MAC from eeprom
        .mac = {0,0,0,0,0,0},
        .ip =  {192, 168, 1, 3},
        .sn =  {255, 255, 255, 0},
        .gw =  {192, 168, 1, 1},
        .dns = {8, 8, 8, 8},
        .dhcp = 1 // 1=static 2=dhcp 0=undefined
    };

    /* hardware initialisatie */

    init_Wiz_SPI_GPIO();
    /* callbacks toekennen voor chipselect, zend en ontvang byte van SPI */
    reg_wizchip_cris_cbfunc(wizchip_lock,wizchip_unlock);
    reg_wizchip_cs_cbfunc(wiz_chip_select, wiz_chip_deselect);
    reg_wizchip_spi_cbfunc(SPI_ReadByte, /* read single byte */
                           SPI_SendByte /* send single byte */
                          );

    /* reset w550io hardware module, dit laadt de defaults */
    wiz_hardware_reset_chip();

    /* ik kan nu de data uit de chip lezen voor later gebruik */
    wiz_NetInfo *netinfo = pvPortMalloc(sizeof(wiz_NetInfo)); /* memory for temp buffer */
    if (netinfo) {
        ctlnetwork(CN_GET_NETINFO, netinfo); // GET info from chip

        int i;
        for (i=0; i<6; i++) {
            old_mac_address[i] = netinfo->mac[i]; // copy de gekregen MAC in de nieuwe structure
        }

        for (i=0; i<4; i++) {
            old_ip_address[i] = netinfo->ip[i]; // copy de gekregen IP
        }

        /* free buffer */
        vPortFree(netinfo);
    }

    /* PHY link status check, wacht tot link online is, na 2 sec een melding op serial port */

    uint32_t tmp=0;
    TickType_t timeout = xTaskGetTickCount() + 2000 ;

    do {
        if (ctlwizchip(CW_GET_PHYLINK, &tmp) == -1) {
            SERIAL_puts("Unknown PHY Link status.\r\n");
        }

        vTaskDelay(1); // boring stuff nothing
        TickType_t t = xTaskGetTickCount();
        if (t > timeout) {
            SERIAL_puts("Check network cable\r\n");
            timeout = t + 2000;
        }
    } while (tmp == PHY_LINK_OFF);

    /* als we hier komen zit er een stekker in de UTP met signalen */

    ctlwizchip(CW_INIT_WIZCHIP, buffsizes); // set up the buffer sizes inside the chip

//    /* maak kopie van status en mac die de chip nu heeft die heeft de PIC erin geschoten */
//
//    wiz_NetInfo *oldinfo = pvPortMalloc(sizeof(wiz_NetInfo)); /* memory for temp buffer */
//
//    if (oldinfo) {
//        /* get the info */
//        ctlnetwork(CN_GET_NETINFO, oldinfo); // GET info from chip
//        /* copy just the mac address */
//        int i;
//        for (i=0; i<6; i++) {
//            ni.mac[i] = oldinfo->mac[i];     // copy de gekregen MAC in de nieuwe structure
//        }
//        /* free buffer */
//        vPortFree(oldinfo);
//    }

    SERIAL_puts("=== before configure ==\r\n");
    dump_network_info(SERIAL_puts); // gebruik een callback voor de output

    /* ik wil nu of STATIC of DHCP doen */

    if (netconfig != NETINFO_DHCP) {
        /* haal de static settings op uit de eeprom en stop die in de chip, TODO, nog een eeprom */
        int i;
        for (i=0; i<6; i++) {
            ni.mac[i]=old_mac_address[i];
        }
        ctlnetwork(CN_SET_NETINFO, &ni);
        SERIAL_puts("\r\n=== after configure ==\r\n");
        dump_network_info(SERIAL_puts); // gebruik een callback voor de output

        SERIAL_puts("\r\n");
    } else {
        /* we doen DHCP */
        /* buffer maken */
        uint8_t *gDATABUF = pvPortMalloc(2048);  // dit moet niet zo groot zijn....
        DHCP_init(0, gDATABUF, 0x12345678 ^ xTaskGetTickCount()); // use socket 0 voor alle DHCP dingen
        /* Loop until we know DHCP is ok or failed */
        uint8_t dhcp_ret;

        while(1) {
            /* DHCP */
            char txt[20];
            /* DHCP IP allocation and check the DHCP lease time (for IP renewal) */
            dhcp_ret = DHCP_run(); // call state machine
            sprintf(txt,"dhcp=%d\r\n",dhcp_ret);
            SERIAL_puts(txt);
            /* normal value would be waiting for lease to expire, 4  */
            if (dhcp_ret == DHCP_IP_LEASED) {
                // dan doen we niks behalve slapen, 5 seconden en nog eens kijken
                vTaskDelay(10000);
            }
            /* in het begin zijn we bezig met de DHCP server, dan krijgen we 1 terug */
            if (dhcp_ret == DHCP_RUNNING) {
                // nog geen IP gekregen, wacht nu 10 ms en kijk of er een antwoord is
                vTaskDelay(10);
            }

            // indien succes of andere IP, dan dit toekennen, dit is slechts de eerste keer
            // en dan na een dag of week of zoiets
            if((dhcp_ret == DHCP_IP_ASSIGN) || (dhcp_ret == DHCP_IP_CHANGED)) {
                /* IP etc van de DHCP server */
                getIPfromDHCP(ni.ip);
                getGWfromDHCP(ni.gw);
                getSNfromDHCP(ni.sn);
                getDNSfromDHCP(ni.dns);

                ni.dhcp = NETINFO_DHCP;
                /* de MAC moet ook gezet worden, gebruik de oude MAC */
                int i;
                for (i=0; i<6; i++) {
                    ni.mac[i]=old_mac_address[i];
                }

                /* zet nu alle netwerk info in de chip */
                ctlnetwork(CN_SET_NETINFO, &ni);

                // display_netinfo();

                SERIAL_puts("DHCP Leased Time : ");
                char txt[20];
                sprintf(txt,"%ld Sec\r\n", getDHCPLeasetime());
                SERIAL_puts(txt);
                dump_network_info(SERIAL_puts); // gebruik een callback voor de output

                SERIAL_puts("\r\n");
                // flag other waiting threads, network is up
                xEventGroupSetBits(xEventBits,0x01); // set network bit TODO make CONST
            }

            if(dhcp_ret == DHCP_FAILED)	{
                SERIAL_puts(">> DHCP Failed\r\n");
                /* we kunnen nu de eeprom gebruiken of de opgeslagen IP of we kunnen UPNP gaan proberen TODO later */
                // User's parts : DHCP failed
                // ===== Example pseudo code =====
                // netconfig = NETINFO_STATIC;
                // set_netinfo_default();
                xEventGroupSetBits(xEventBits,0x01); // set network bit TODO make CONST
            }
        } // while
    }
}

void vTimerCallback( void *ptr)
{
    DHCP_time_handler();
    DNS_time_handler();
    /* add music player timers here too ? */
}

void vTask5( void *pvParameters )
{
    /* we gaan hier blocken tot dat de vlag van de Netwerk Bit 0x01 gezet is */\

    EventBits_t xx = xEventGroupWaitBits( xEventBits, 0x01, pdFALSE, pdFALSE, portMAX_DELAY );

    SERIAL_puts("Network flag is OK\r\n");
    /* we gaan een DNS lookup doen naar de 'SERVER' die ik nog niet ken */

    // uint8_t host[] = "icecast.omroep.nl";
    uint8_t host[] = "vergadering-gemist.nl";

    uint8_t host_ip[]= {0,0,0,0}; // ip dat ik van de DNS terug krijg (de laatste)
    uint8_t mijn_dns[] = { 0,0,0,0 }; // moet van DHCP komen, hierin kopieren
    // DHCP_allocated_dns ... oh oh
    getDNSfromDHCP(mijn_dns);

    char google_dns[] = {8,8,8,8}; // backup

    /* DNS client initialization */
    uint8_t   *buf_dns = pvPortMalloc(MAX_DNS_BUF_SIZE);
    DNS_init(1, buf_dns, xTaskGetTickCount()); // gebruik voor DNS socket 1, 0 is in gebruik voor dhcp die af en toe kan zenden en ontvangen
    int8_t rvx = DNS_run(mijn_dns,host,host_ip);

    if (rvx == -1) {
        SERIAL_puts("parse error\r\n");
    }
    if (rvx == -2) {
        SERIAL_puts("timeout error\r\n");
    }
    if (rvx == -4) {
        SERIAL_puts("DNS server error\r\n");
    }
    if (rvx == 0) {
        // success flag
        char txt[40];
        sprintf(txt,"ip=%3d.%3d.%3d.%3d\r\n",host_ip[0],host_ip[1],host_ip[2],host_ip[3]);
        SERIAL_puts(txt);
    }

    if (rvx < 0) {
        SERIAL_puts("< 0 antw error\r\n");
    }

    vPortFree(buf_dns);
    SERIAL_puts("step 1\r\n");
    VS_Hard_Reset();
    SERIAL_puts("step 2\r\n");

    VS_Test_Sine(1, 100);
    SERIAL_puts("step 3\r\n");
    vTaskDelay(200);
    VS_Test_Sine(0, 100);
    SERIAL_puts("step 4\r\n");
    // VS_Registers_Dump();

    VS_Volume_Set(0x0202);
    SERIAL_puts("step volume\r\n");


    extern const uint8_t sample_edwin[] ;
    VS_SDI_JAS_Buffer(sample_edwin, 20081 );
    int i;
    uint8_t zeroes[] = {0,0,0,0,0,0,0,0,0,0};
    for (i=0; i<100; i++) {
        VS_SDI_JAS_Buffer(zeroes, sizeof(zeroes) );
    }

    SERIAL_puts("step 5\r\n");
    extern const uint8_t sample_ben[] ;
    VS_SDI_JAS_Buffer(sample_ben, 10421 );

    for (i=0; i<100; i++) {
        VS_SDI_JAS_Buffer(zeroes, sizeof(zeroes) );
    }

    SERIAL_puts("step 6\r\n");
    int8_t rv;
    rv= socket(2,Sn_MR_TCP,32000+xTaskGetTickCount(),0); // must be random number
    xprintf("sock=%d\r\n",rv);

    /* opname starten */
    const uint32_t recbufsize = 32768; // bytes
    const uint32_t recbufsize_wrds = recbufsize >> 1;

    uint8_t *tmp = pvPortMalloc(recbufsize);
    radio_player_t *rp = pvPortMalloc(sizeof(radio_player_t));
    const uint32_t serial_variant = 1;

    if (serial_variant == 0) {
        /* variant met uitlezen van data uit de REGISTERS van de VS1063 */
        if(tmp && rp) {
            xprintf("rec test %d %d\r\n", recbufsize, recbufsize_wrds);
            memset(rp,0,sizeof(radio_player_t));

            rp->devicemode = DevMode_TX_Ice;
            rp->encoding_preset = 0; // mp3
            VS_Encoder_Init(rp);
            int n=0;
            int fileSize =0;
            int counter = recbufsize_wrds;
            uint8_t *rbp = tmp; // ptr naar buf
            while (counter) {
                /* how many words are available in the vs1063 to read now */
                if ((n = VS_Read_SCI(SCI_RECWORDS)) > 0) {
                    int i;
                    /* check buffer boundry */
                    n = min(n, counter);
                    /* read the words from the chip */
                    for (i=0; i<n; i++) {
                        uint16_t w = VS_Read_SCI(SCI_RECDATA);
                        *rbp++ = (uint8_t)(w >> 8);
                        *rbp++ = (uint8_t)(w & 0xFF);
                    }
                    /* */
                    counter -= n;
                    xprintf("%d\r\n",n);
                    // fwrite(recBuf, 1, 2*n, writeFp);
                    fileSize += 2*n;
                } else {
                    /* The following read from SCI_RECWORDS may appear redundant.
                       But it's not: SCI_RECWORDS needs to be rechecked AFTER we
                       have seen that SM_CANCEL have cleared. */
                    //  if (playerState != psPlayback && !(ReadSci(SCI_MODE) & SM_CANCEL)
                    //          && !ReadSci(SCI_RECWORDS)) {
                    //     playerState = psStopped;
                    // }
                }
            }
        }
    } else {
        /* de serial port variant */
        if(tmp && rp) {
            xprintf("rec test serial %d %d\r\n", recbufsize, recbufsize_wrds);
            memset(rp,0,sizeof(radio_player_t));

            rp->devicemode = DevMode_TX_Ice_Serial;
            rp->encoding_preset = 0; // mp3
            VS_Encoder_Init(rp);

            /* data starts flowing into usart 1 now */
            xprintf("via serial\r\n");
            /* just wait until done */

            vTaskDelay(10000); // 10 sec

        }

        /* nu de encoder stoppen */
        VS_Write_SCI(SCI_MODE, VS_Read_SCI(SCI_MODE) | SM_CANCEL);
        xprintf("wait enc stop\r\n");
        /* kijken of de encoder gestopt is */
        do {
            int x = VS_Read_SCI(SCI_RECWORDS);
            xprintf("x in encoder =%d\r\n",x);
        } while (VS_Read_SCI(SCI_MODE) & SM_CANCEL);

        xprintf("enc is stop\r\n");
        /*spec page 57 */
        /*
        When you want to finish encoding a file, set bit SM_CANCEL in SCI_MODE.
        After a while (typically less than 100 ms), SM_CANCEL will clear.
        Done that
        */

        /* If using SCI for data transfers, read all remaining words using SCI_HDAT1/SCI_HDAT0. */
        int x = VS_Read_SCI(SCI_RECWORDS);
        xprintf("still in encoder =%d\r\n",x);
        while (x) {
            uint16_t w = VS_Read_SCI(SCI_RECDATA);
            //    *rbp++ = (uint8_t)(w >> 8);
            //    *rbp++ = (uint8_t)(w & 0xFF);
            x--;
        }
        x = VS_Read_SCI(SCI_RECWORDS);
        xprintf("yet still in encoder =%d\r\n",x);

        /*
        Then read parametric_x.endFillByte.
        If the most significant bit (bit 15) is set to 1, then the file is of an odd length
        and bits 7:0 contain the last byte that still should be written
        to the output file. Now write 0 to endFillByte.
        */
        uint16_t endfilbyte = VS_Read_SCI(0x1e06);
        xprintf("endfil = %d\r\n",endfilbyte);
        VS_Write_SCI(0x1e06,0);
        VS_Write_SCI(0x1e06,0);

        /* When all samples have been transmitted, SM_ENCODE bit of SCI_MODE will be
        cleared by VS1063a, and SCI_HDAT1 and SCI_HDAT0 are cleared.
        */
        do {
            int x = VS_Read_SCI(SCI_RECWORDS);
            xprintf("x in encoder =%d\r\n",x);
        } while (VS_Read_SCI(SCI_MODE) & SM_ENCODE);

        VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
        VS_Volume_Set(0x0202);
//        VS_Soft_Reset(DEFAULT_DIVIDER);

        /* afspelen van opgenomen buffer */
        VS_SDI_JAS_Buffer(tmp, recbufsize );

        for (i=0; i<100; i++) {
            VS_SDI_JAS_Buffer(zeroes, sizeof(zeroes) );
        }
        vPortFree(tmp);
        vPortFree(rp);

//   uint8_t ikip[] = {192,168,1,1};
        xprintf("done playback\r\n");
    }
#define ARROW_HACK 1
#if ARROW_HACK
    host_ip[0] = 81;
    host_ip[1] = 173;
    host_ip[2] = 3;
    host_ip[3] = 132 ; /* hack ivm ARROW */
#endif
    rv = connect(2,host_ip,80);
    xprintf("con=%d\r\n",rv);

    const uint16_t bufsize = 8192; // size of buf in Wiznet voor deze socket

    uint8_t *buf = pvPortMalloc(bufsize); // pak buf van systeem
    i = sprintf(buf, // "GET /radio1-sb-aac HTTP/1.1\r\n" /* radio1-sb-aac  radio2-sb-aac 3 4 etc */
                "GET / HTTP/1.1\r\n"
                //   "GET /live128 HTTP/1.1\r\n"
                //"Host: icecast.omroep.nl\r\n"
                //"User-Agent: VLC/2.0.8 LibVLC/2.0.8\r\n"
                //"Range: bytes=0-\r\n"
                "Connection: close\r\n"
                "Icy-MetaData: 0\r\n\r\n");

    rv= send(2,buf,i);
    xprintf("send=%d\r\n",rv);
    // skip header TODO TODO

    int32_t got_bytes;
    do {
        got_bytes = recv(2,buf,bufsize);
        // xprintf("b=%d\r\n",got_bytes);
        SERIAL_puts(".");
        if (got_bytes > 0) {
            VS_SDI_JAS_Buffer(buf, got_bytes );
        } else if (got_bytes < 0) {
            // error
            SERIAL_puts("error\r\n");

        } else {
            // 0 bytes
            // end of data
            SERIAL_puts("end\r\n");

        }
    } while (got_bytes > 0);
    vPortFree(buf); // en terug aan systeem
    close(2);


    while (1) {
        vTaskDelay(10);
    }

}

int main(void)
{
    QUEUE_INIT(my_TX_queue);
    QUEUE_INIT(my_RX_queue);
    QUEUE_INIT(my_KEYS_queue);

    GPIO_Init_edwin();
    VLSI_SPI_GPIO_Init();
    init_USART2(115200);
    init_USART1(115200*4); // moet hoger worden
    SERIAL_puts("Boot\r\n");
    adc_configure();

    xSemaphoreSPI2 = xSemaphoreCreateMutex();
    xSemaphoreWRAM = xSemaphoreCreateMutex();
    xSemaphoreWIZCHIP = xSemaphoreCreateMutex();
    if (xSemaphoreSPI2 == NULL || xSemaphoreWRAM == NULL || xSemaphoreWIZCHIP == NULL) {
        SERIAL_puts("Out of memory. xSemaphoreCreateMutex\r\n");
        while (1) {
            // we do not get here
        }

    }

    /* maak mutexen en event groups etc */
    xEventBits = xEventGroupCreate();
    if (xEventBits == NULL ) {
        // fatal error, no memory, hang here
        SERIAL_puts("Out of memory. xEventGroupCreate\r\n");
        while (1) {
            // we do not get here
        }
    }

    /* we maken een timer, deze wordt iedere seconde aangeroepen, handig voor timeouts */
    /* dns en dhcp gebruiken globale variabelen die timeouts weergeven per seconde */
    TimerHandle_t xSecondenTimer;
    /* maak de timer */
    xSecondenTimer =  xTimerCreate("TimerSec", 1000, pdTRUE, (void *) 1, vTimerCallback);
    xTimerStart(xSecondenTimer,0);

    /* Create Start thread */
    xTaskCreate(vTask1, "Ben_Thread", 256, NULL, 0, NULL);
    xTaskCreate(vTask2, "Theo_Thread", 256, NULL, 0, NULL);
    xTaskCreate(vTask5, "App", 256, NULL, 0, NULL);

    xTaskCreate(vTaskDHCP, "DHCP", 256, NULL, 0, NULL); /* stack must be large or crash will happen */
    // assert_failed("Test 1235", 8888);
    vTaskStartScheduler();
    /* Infinite loop */

    while (1) {
        // we do not get here
    }

}

/* called when stack is about to crash */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
    for(;;);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */

void assert_failed(uint8_t *file, uint32_t line)
{
    taskDISABLE_INTERRUPTS();
    //    SERIAL_puts((uint8_t*)"\r\nAssert: ");
    char buf[6];
    char *s = (char *) file;
    USART_ITConfig(USART2, USART_IT_TXE | USART_IT_RXNE, DISABLE);
    while (*s) {
        USART_SendData(USART2, *s);
        s++;
    }

    snprintf((char *) buf, 20, " %" PRIu32, line);
    s = buf;
    while (*s) {
        USART_SendData(USART2, *s);
        s++;
    }
    // SERIAL_puts(buf);
    while (1) {

    }
}

#endif
