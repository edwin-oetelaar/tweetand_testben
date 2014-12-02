/*
 * NADA internet radio receiver (radio) and transmitter (encoder) using Mp3 or Ogg-Vorbis
 * Circuit Cellar Challenge : wiznet 2014
 * Edwin en Ben 10 aug 2014 bitbucket
 */

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
#include "ringbuffer.h"
#include "wizchip_conf.h"

#define min(a, b) (((a)<(b))?(a):(b))
static const char zeroes[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


typedef enum {
    pm_listening = 0, pm_sending = 1, pm_intercom = 2
} playermode_t;

typedef struct {
    const char *text;
    /* text on lcd display */
    const char *host;
    /* hostname on internet */
    const uint8_t ip[4];
    /* bare ip when not using hostname, set host=NULL */
    const uint32_t port;
    /* port number of service */
    const char *mount;
    /* mount point */
    const char *passw;
    /* password for ice cast */
    const playermode_t mode; /* listen or send */
} channel_t;

static const channel_t channels[] = {
    {
        .text = "Radio Arrow 1",
        .host = NULL,
        .ip =
        {91,221,151,156},
        .port = 80,
        .mount = "",
        .mode = pm_listening
    },
    {
        .text = "Radio Arrow 2",
        .host = NULL,
        .ip =
        {91,221,151,178},
        .port = 9109,
        .mount = "",
        .mode = pm_listening
    },
    {
        .text = "Test Channel1 RX",  /* luisteren naar eigen uitzendingen */
        .host = "s1.streamsolution.nl",
        .ip = {0, 0, 0, 0},
        .port = 8000,
        .mount = "test",
        .mode = pm_listening
    },
    {
        .text = "Test Channel2 RX",  /* luisteren naar eigen uitzendingen */
        .host = "s1.streamsolution.nl",
        .ip = {0, 0, 0, 0},
        .port = 8000,
        .mount = "test2",
        .mode = pm_listening
    },
    {
        .text = "Trans Chan Test ",
        .host = "s1.vergadering-gemist.nl",
        .port = 8000,
        .mount = "test",
        .passw = "test",
        .mode = pm_sending
    },
    {
        .text = "Trans Chan Test2",
        .host = "s1.vergadering-gemist.nl",
        .port = 8000,
        .mount = "test2",
        .passw = "test",
        .mode = pm_sending
    },
    {
        .text = "Radio 1 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio1-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 2 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio2-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 3 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "3fm-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 4 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio4-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 5 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio5-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "Radio 6 AAC 32kb",
        .host = "icecast.omroep.nl",
        .port = 80,
        .mount = "radio6-sb-aac",
        .mode = pm_listening
    },
    {
        .text = "BNR nieuwsradio",
        .host = "icecast-bnr.cdp.triple-it.nl",
        .port = 80,
        .mount = "bnr_aac_32_04",
        .mode = pm_listening
    },
    {
        .text = "Absolute Radio UK",
        .host = "aacplus-ac-32.timlradio.co.uk",
        .port = 80,
        .mount = "/",
        .mode = pm_listening
    }
};

const uint32_t max_channels = sizeof(channels) / sizeof(channels[0]);

uint8_t WIZ_SPI_ReadByte(void);

uint8_t WIZ_SPI_SendByte(uint8_t byte);

uint8_t do_http_get(uint8_t sn, const char *url, void (writefunc)(const char *buf, uint32_t len));
uint8_t get_stream_from_server(uint8_t sn, const char *host, const uint8_t *ip, const uint16_t port, const char *mntpnt, const char *password, volatile uint32_t *status);

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

char AssertStringBuffer[16];

lcd_context_t LCD; // make a context

jack_ringbuffer_t *streambuffer; // used for audio encoding, and playout, IRQ handler will put stuff there
volatile uint32_t recorder_active_flag = 0; // put stuff in ringbuffer flag
volatile uint32_t change_status = 0; // put a 1 here and the device will check for a new role to play, after accepting it will set this flag to 0
volatile uint32_t active_channel = 0; // index of active channel
static uint32_t menu_channel = 0; // index of channel menu is showing
volatile uint32_t player_active_flag = 0; // the player can consume stuff now
volatile uint32_t player_running = 0; // the player is running, do not destroy the ringbuffer

static uint8_t playout_volume = 64;
/*default volume */


EventGroupHandle_t xEventBits; // set by dhcp task when network is up and running, bit 0x01 is NETWORK-OK
SemaphoreHandle_t xSemaphoreSPI2;
SemaphoreHandle_t xSemaphoreWRAM; // WRAM is mem inside VS1063, must be protected, needs many read writes to registers, may not be interrupted
SemaphoreHandle_t xSemaphoreWIZCHIP;

//extern uint8_t use_230400;

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

/* read audio bytes from the encoder over USART1, ALL USART1 interrupts
 * maximum freq about 25000 Hz called, 256kbps stream
 */
void USART1_IRQHandler(void)
{
    /* if Receive Register not empty then get byte */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        /* get byte */
        char ch = USART1->DR;
        /* if we want to capture data store in buffer */
        if (recorder_active_flag) {
            /* lock free jack_ringbuffer is very good */
            jack_ringbuffer_write(streambuffer, &ch, 1);
        } else {
            /* discard, TODO:  SET OVERRUN FLAG?
             * data from Encoder but transmitter already switched off, not a problem
             */
        }
    }
}

// this is the interrupt request handler (IRQ) for ALL USART2 interrupts
void USART2_IRQHandler(void)
{
    /* if Receive Register not empty then get byte */
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART2->DR;
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

static void handle_menu_channels(uint8_t key)
{
    int n = menu_channel; // get global var
    /* kanaal selectie */
    switch (key) {

    case 'D' :
        if (n < max_channels - 1) {
            n++;
        } else {
            n = 0;
        }
        break;
    case 'U' :
        if (n > 0) {
            n--;
        } else {
            n = max_channels - 1;
        }
        break;

    case 'S' : /* select */
        /* set channel */
        active_channel = n;
        /* set flag */
        change_status = 1;
        break;
    }
    menu_channel = n; // update global var

    const channel_t *p = channels + n; // magic

    lcd_set_cursor_position(&LCD, 0, 0);
    lcd_write(&LCD, p->text, strlen(p->text));

    if (p->host != NULL) {
        xprintf("channel= %s : hostname: %s\n", p->text, p->host);
        lcd_set_cursor_position(&LCD, 1, 0);
        lcd_write(&LCD, p->host, strlen(p->host));
    } else {
        lcd_set_cursor_position(&LCD, 1, 0);
        char buf[20];
        sprintf(buf, "ip:%d.%d.%d.%d", p->ip[0], p->ip[1], p->ip[2], p->ip[3]);
        lcd_write(&LCD, buf, strlen(buf));

        xprintf("channel= %s : use ip  : %d.%d.%d.%d\n", p->text, p->ip[0], p->ip[1], p->ip[2], p->ip[3]);
    }
}

static void handle_menu_volume(uint8_t key)
{
    /* volume */
    uint8_t n = VS_Read_SCI(SCI_VOL) & 0xFF;

    switch (key) {

    case 'D' :
        if (n < 254) {
            n++;
        }
        break;
    case 'U' :
        if (n > 0) {
            n--;
        }
        break;
    }
    playout_volume = n;

    VS_Volume_Set((playout_volume << 8) | playout_volume);

    lcd_set_cursor_position(&LCD, 0, 0);
    //              1234567890123456
    lcd_write(&LCD, "## Set Volume ##", 20);
    lcd_set_cursor_position(&LCD, 1, 0);
    char buf[20];
    sprintf(buf, "%03d          ", 255 - playout_volume);
    lcd_write(&LCD, buf, 16);

}


static void handle_menu_key(uint8_t key)
{
    //static n = 0;
    static uint8_t kolom = 0;
    /* kolom 0 = channel selectie,
     * kolom 1 = volume control
     */
    /* left right is kolom */
    switch (key) {
    case 'L' :
        if (kolom == 1) {
            kolom = 0;
        } else {
            kolom = 1;
        }
        break;
    case 'R' :
        if (kolom == 0) {
            kolom = 1;
        } else {
            kolom = 0;
        }
        break;
    }


    switch (kolom) {
    case 0 :
        /* channel select */
        handle_menu_channels(key);
        break;
    case 1:
        /* volume instellen */
        handle_menu_volume(key);
        break;

    }

}

static void vTaskUserInterface(void *arg)
{
    lcd_init_context(&LCD); // put values in the context
    lcd_init_gpio(&LCD); // init the gpio
    lcd_set_cursor_position(&LCD, 0, 0);
    lcd_write(&LCD, "*Team Tweetand* ", 16); // write a string
    int x = 10;
    while (x--) {
        lcd_set_backlight(&LCD, 0);
        vTaskDelay(100);
        lcd_set_backlight(&LCD, 1);
        vTaskDelay(200);
    }

    uint32_t keystate = 0; // 0=not pressed 1=pressed, 2=waiting for release
    uint32_t next_state = 0; // next state

    uint8_t prev_key = 0;
    uint8_t slowkey = 1; // double check ADC value
    handle_menu_key('-'); // initial screen

    for (; ;) {
        uint16_t val = adc_convert();
        uint8_t key = val2key(val); // lookup de ADC naar key
        //xprintf("%c %d %d\r\n",key,prev_key,keystate,next_state);
        switch (keystate) {
        case 0:

            if (key != '-') {
                /* a key was pressed*/
                prev_key = key;
                if (slowkey) {
                    next_state = 1; // fast keys to to state 2, for slow keys go to state 1
                } else {
                    SERIAL_write(&key, 1);
                    handle_menu_key(key);
                    next_state = 2; // go to state 2, skip confirmation on ADC
                }
            } else {
                next_state = 0; // wait for key again
            }
            break;

        case 1:
            if (key == prev_key) {
                /* same value 2nd time, accept key */
                SERIAL_write(&key, 1);
                handle_menu_key(key);
                next_state = 2;
            } else {
                // not same key
                next_state = 0;
            }
            break;

        case 2:
            if (key == prev_key) {
                next_state = 2; /* hang here until other value appears */
            } else {
                /* wait for keypress again */
                next_state = 0;
            }
            break;
        }

        keystate = next_state;

        vTaskDelay(20);
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
    } else {
        /* dat werkte dus goed */
        wiz_NetTimeout xx = { .retry_cnt = 0x08, .time_100us = 0x07d0 };
        ctlnetwork( CN_SET_TIMEOUT, &xx);
    }
    return 0; // ok
}

/* USART1 connects to Vs1063, pins PB6 (not used) and PB7 */
void USART1_hardware_init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART1 initialization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

    /* enable Clocks for USART1 and GPIOB */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* defaults in struct */
    GPIO_StructInit(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alt function
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push pull
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed; // fast
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // pull needed, not sure

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7; // Pins 7 (RX) is used

    /* write into GPIO registers */
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    //Connect to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // usart1 TX
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // usart1 RX

    /* defaults in struct */
    USART_StructInit(&USART_InitStruct);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx; /* not used USART_Mode_Tx */
    /* USART_OverSampling8Cmd(USART1,1); not used allow for very high bitrates, set oversample to 8 instead of 16 */

    /* write values into USART registers*/
    USART_Init(USART1, &USART_InitStruct);

    /* enable the USART1 receive interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* disable Transmit Data Register empty interrupt */
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    /* write into NVIC registers */
    NVIC_Init(&NVIC_InitStructure);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
}

/* debug info on USART 2 connected to USB */
void USART2_hardware_init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART2 initialization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

    /* init the struct with defaults */
    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);

    // enable Clocks for APB1 and GPIOA
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 2 (TX) and 3 (RX) are used
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    //Connect to AF
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2 tx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // PA3 rx

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


static void vTaskDHCP(void *arg)
{
    /* de netwerk config moet uit een eeprom komen, mogeljk later een 24c32 erbij bakken op de i2c bus */

    uint32_t netconfig = NETINFO_DHCP; /* NETINFO_STATIC */
//   uint8_t old_ip_address[4] = {0};   /* het IP van de PIC */
    uint8_t old_mac_address[6] = {0};  /* het MAC van de PIC */


    /* netwerk buffer sizes per socket */
    uint8_t buffsizes[2][8] = {
        // {2, 2, 2, 2, 2, 2, 2, 2},
        // {2, 2, 2, 2, 2, 2, 2, 2}
        {4, 4, 4, 4, 0, 0, 0, 0},
        {4, 4, 4, 4, 0, 0, 0, 0}

    };

    wiz_NetInfo ni = { // new network info, we copy the MAC from eeprom
        .mac = {0, 0, 0, 0, 0, 0},
        .ip =  {192, 168, 1, 3},
        .sn =  {255, 255, 255, 0},
        .gw =  {192, 168, 1, 1},
        .dns = {8, 8, 8, 8},
        .dhcp = 1 // 1=static 2=dhcp 0=undefined
    };

    /* hardware initialisatie */

    init_Wiz_SPI_GPIO();
    /* callbacks toekennen voor chipselect, zend en ontvang byte van SPI */
    reg_wizchip_cris_cbfunc(wizchip_lock, wizchip_unlock);
    reg_wizchip_cs_cbfunc(wiz_chip_select, wiz_chip_deselect);
    reg_wizchip_spi_cbfunc(WIZ_SPI_ReadByte, /* read single byte */
                           WIZ_SPI_SendByte /* send single byte */
                          );

    /* reset w550io hardware module, dit laadt de defaults */
    wiz_hardware_reset_chip();

    /* ik kan nu de data uit de chip lezen voor later gebruik */
    /* allocate memory for temp buffer */
    wiz_NetInfo *netinfo = pvPortMalloc(sizeof(wiz_NetInfo));
    if (netinfo) {
        ctlnetwork(CN_GET_NETINFO, netinfo); // GET info from chip

        int i;
        for (i = 0; i < 6; i++) {
            old_mac_address[i] = netinfo->mac[i]; // copy de gekregen MAC in de nieuwe structure
        }

        //    for (i=0; i<4; i++) {
        //        old_ip_address[i] = netinfo->ip[i]; // copy de gekregen IP
        //    }

        /* free buffer */
        vPortFree(netinfo);
    }

    /* PHY link status check, wacht tot link online is, na 2 sec een melding op serial port */

    uint32_t tmp = 0;
    TickType_t timeout = xTaskGetTickCount() + 2000;

    do {
        if (ctlwizchip(CW_GET_PHYLINK, &tmp) == -1) {
            SERIAL_puts("Unknown PHY Link status.\r\n");
        }

        vTaskDelay(1); // boring stuff nothing
        TickType_t t = xTaskGetTickCount();
        if (t > timeout) {
            SERIAL_puts("Check network cable\r\n");
            lcd_home(&LCD);
            lcd_set_cursor_position(&LCD, 0, 0);
            const char *xtxt = "Check Netw Cable";
            lcd_write(&LCD, xtxt, strlen(xtxt));
            timeout = t + 2000;
        }
    } while (tmp == PHY_LINK_OFF);

    /* als we hier komen zit er een stekker in de UTP met signalen */

    ctlwizchip(CW_INIT_WIZCHIP, buffsizes); // set up the buffer sizes inside the chip

    SERIAL_puts("=== before configure ==\r\n");
    dump_network_info(SERIAL_puts); // gebruik een callback voor de output

    /* ik wil nu of STATIC of DHCP doen */

    if (netconfig != NETINFO_DHCP) {
        /* haal de static settings op uit de eeprom en stop die in de chip, TODO, nog een eeprom */
        int i;
        for (i = 0; i < 6; i++) {
            ni.mac[i] = old_mac_address[i];
        }
        ctlnetwork(CN_SET_NETINFO, &ni);
        SERIAL_puts("\r\n=== after configure ==\r\n");
        dump_network_info(SERIAL_puts); // gebruik een callback voor de output

        SERIAL_puts("\r\n");
    } else {
        /* we doen DHCP */
        /* buffer maken */
        uint8_t *gDATABUF = pvPortMalloc(2048);  // dit moet niet zo groot zijn....
        DHCP_init(0, gDATABUF, 0x72345678 ^ xTaskGetTickCount()); // use socket 0 voor alle DHCP dingen
        /* Loop until we know DHCP is ok or failed */
        uint8_t dhcp_ret;

        while (1) {
            /* DHCP */
            char txt[20];
            /* DHCP IP allocation and check the DHCP lease time (for IP renewal) */
            dhcp_ret = DHCP_run(); // call state machine
            sprintf(txt, "dhcp=%d\r\n", dhcp_ret);
            SERIAL_puts(txt);
            /* normal value would be waiting for lease to expire, 4  */
            if (dhcp_ret == DHCP_IP_LEASED) {
                // dan doen we niks behalve slapen, 60 seconden en nog eens kijken
                // vTaskDelay(10000);
                vTaskDelay(60000);
            }
            /* in het begin zijn we bezig met de DHCP server, dan krijgen we 1 terug */
            if (dhcp_ret == DHCP_RUNNING) {
                // nog geen IP gekregen, wacht nu 100 ms en kijk of er een antwoord is
                vTaskDelay(100);
            }

            // indien succes of andere IP, dan dit toekennen, dit is slechts de eerste keer
            // en dan na een dag of week of zoiets
            if ((dhcp_ret == DHCP_IP_ASSIGN) || (dhcp_ret == DHCP_IP_CHANGED)) {
                /* IP etc van de DHCP server */
                getIPfromDHCP(ni.ip);
                getGWfromDHCP(ni.gw);
                getSNfromDHCP(ni.sn);
                getDNSfromDHCP(ni.dns);

                ni.dhcp = NETINFO_DHCP;
                /* de MAC moet ook gezet worden, gebruik de oude MAC */
                int i;
                for (i = 0; i < 6; i++) {
                    ni.mac[i] = old_mac_address[i];
                }

                /* zet nu alle netwerk info in de chip */
                ctlnetwork(CN_SET_NETINFO, &ni);

                // display_netinfo();

                SERIAL_puts("DHCP Leased Time : ");
                char txt[20];
                sprintf(txt, "%ld Sec\r\n", getDHCPLeasetime());
                SERIAL_puts(txt);
                dump_network_info(SERIAL_puts); // gebruik een callback voor de output

                SERIAL_puts("\r\n");
                // flag other waiting threads, network is up
                xEventGroupSetBits(xEventBits, 0x01); // set network bit TODO make CONST
            }

            if (dhcp_ret == DHCP_FAILED) {
                SERIAL_puts(">> DHCP Failed\r\n");
                /* we kunnen nu de eeprom gebruiken of de opgeslagen IP of we kunnen UPNP gaan proberen TODO later */
                // User's parts : DHCP failed
                // ===== Example pseudo code =====
                // netconfig = NETINFO_STATIC;
                // set_netinfo_default();
                xEventGroupSetBits(xEventBits, 0x01); // set network bit TODO make CONST
            }
        } // while
        // free memory
        vPortFree(gDATABUF);
    }
}

void vTimerCallback(void *ptr)
{
    DHCP_time_handler();
    DNS_time_handler();
    /* add music player timers here too ? */
}

uint8_t VS_Get_Serial_Byte(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {
        /* spin here */
    };
    uint8_t t = USART_ReceiveData(USART1) & 0xFF;

    return (t);
}

uint8_t stream_to_test_server(uint8_t sn, const char *host, uint16_t port, const char *mntpnt, const char *password, volatile uint32_t *status)
{
    uint8_t host_ip[] = {0, 0, 0, 0};
    //uint8_t mijn_dns[] = { 0,0,0,0 };
    uint8_t mijn_dns[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};

    int rv = 0;
    // get DNS server ip
    getDNSfromDHCP(mijn_dns[0]);
    // get DNS2 server ip
    getDNS2fromDHCP(mijn_dns[1]);

    uint8_t *buf_dns = pvPortMalloc(MAX_DNS_BUF_SIZE);
    uint8_t dns_counter = 0;
    while (dns_counter < 2) {
        // init dns request
        DNS_init(1, buf_dns, xTaskGetTickCount() & 0xFFFF); // gebruik voor DNS socket 1, 0 is in gebruik voor dhcp die af en toe kan zenden en ontvangen
        // do the request on given server
        int8_t rvx = DNS_run(mijn_dns[dns_counter], host, host_ip);
        if (rvx == 0) {
            xprintf("ip=%3d.%3d.%3d.%3d\r\n", host_ip[0], host_ip[1], host_ip[2], host_ip[3]);
            // vPortFree(buf_dns);
            break;
        } else {
            //  vPortFree(buf_dns);
            xprintf("dns error %d\r\n", rvx);
            //  return -1;
        }
        dns_counter++;
    }
    vPortFree(buf_dns);
    if (dns_counter == 2) {
        xprintf("fatal DNS problem\r\n");
    }


    rv = socket(sn, Sn_MR_TCP, 32000 + xTaskGetTickCount(), SF_TCP_NODELAY); // must be random number
    xprintf("sock=%d\r\n", rv);
    rv = connect(sn, host_ip, port);
    xprintf("con=%d\r\n", rv);

    const uint16_t bufsize = 2048; // 8192; // size of buf in Wiznet voor deze socket

    char *buf = pvPortMalloc(bufsize); // pak buf van systeem
    char *auth1 = pvPortMalloc(256); // pak buf van systeem
    char *auth2 = pvPortMalloc(256); // pak buf van systeem

    if (buf == NULL || auth1 == NULL || auth2 == NULL) {
        // out of memory
        vPortFree(buf);
        vPortFree(auth1);
        vPortFree(auth2);
        xprintf("out of mem\r\n");
        return -1;
    }

    strcpy(auth1, "source:");
    strcat(auth1, password);
    base64encode(auth1, strlen(auth1), auth2, 256);
    int i;
    i = sprintf(buf, "SOURCE /%s ICE/1.0\r\n"
                "Content-Type: audio/mpeg\r\n"
                "Authorization: Basic %s\r\n"
                "User-Agent: WizStream1\r\n"
                "Ice-Public: 1\r\n"
                "Ice-Description: Streamsolution test channel 2\r\n"
                "Pragma: no-cache\r\n"
                "Cache-Control: no-cache, no-transform, private, no-store, proxy-revalidate\r\n\r\n", mntpnt, auth2
               );

    vPortFree(auth1);
    vPortFree(auth2);

    rv = send(sn, buf, i);

    /* wait for server response, can be 200 = OK, 401=Bad password, 501 server error, 403 mount busy */
    /* HTTP/1.0 200 */

    uint16_t len = getSn_RX_RSR(sn);
    uint32_t timeout = 5000;
    while ((len < 13) && (timeout != 0)) {
        len = getSn_RX_RSR(sn);
        vTaskDelay(1);
        timeout--;

        if (*status) {
            break;
        }
    }

    if (*status) {
        /* cleanup and exit */

    }

    if (timeout == 0) {
        return -2;
    }

    len = recv(sn, buf, bufsize);

    if (!strncmp(buf, "HTTP/1.0 200 ", 13)) {
        // ok
        // start sending data now
    } else if (!strncmp(buf, "HTTP/1.0 401 ", 13)) {
        // bad password
        xprintf("bad pasword\r\n");
        return -3;
    } else if (!strncmp(buf, "HTTP/1.0 403 ", 13)) {
        // mountpoint busy
        xprintf("mountpoint busy\r\n");
        return -4;
    }

    /* start data pump here */
    radio_player_t *rp = pvPortMalloc(sizeof(radio_player_t));
    streambuffer = jack_ringbuffer_create(32768); // global variable
    //xprintf("sb=%x",streambuffer);

    if (buf && rp && streambuffer) {
        /* zet volume zacht van de output */
        VS_Volume_Set(0x2020);
        xprintf("rec test serial %d\r\n", bufsize);
        memset(rp, 0, sizeof(radio_player_t));
        rp->devicemode = DevMode_TX_Ice_Serial;
        rp->encoding_preset = 0; // mp3
        xprintf("tx via serial\r\n");
        recorder_active_flag = 1; // we have a ringbuffer so start using it
        VS_Encoder_Init(rp);

        /* data starts flowing into usart 1 now */
        int n = 0;
        int nn = 0;
        while (1) {

            if (*status) {
                break;
            }

            if (jack_ringbuffer_read_space(streambuffer) > bufsize) {
                // we have data in the buffer for the network
                jack_ringbuffer_read(streambuffer, buf, bufsize);
                int32_t x = send(sn, buf, bufsize);
                if (x < 0) {
                    xprintf("transmission end\r\n");
                    break;
                }
                xprintf("x\r\n");

            } else {
                nn++;
                if (nn == 10) {
                    size_t inbuf = jack_ringbuffer_read_space(streambuffer);
                    size_t frbuf = jack_ringbuffer_write_space(streambuffer);
                    //int fullness = inbuf <<2  / bufsize << 2;
                    char tt[32];
                    sprintf(tt, "%5d %5d\r\n", inbuf, frbuf);
                    SERIAL_puts(tt);
                    nn = 0;
                }
                vTaskDelay(10); //
            }

        }
        /* set flag so irq not pushes data into buffer anymore */
        recorder_active_flag = 0;

        /* signal encoder to stop set cancel flag */
        /*spec page 57 */
        /*
        When you want to finish encoding a file, set bit SM_CANCEL in SCI_MODE.
        After a while (typically less than 100 ms), SM_CANCEL will clear.
        */
        VS_Write_SCI(SCI_MODE, VS_Read_SCI(SCI_MODE) | SM_CANCEL);
        xprintf("wait enc stop\r\n");
        /* wait until encoder really stopped */
        do {
            int x = VS_Read_SCI(SCI_RECWORDS);
            xprintf("x in encoder =%d\r\n", x);
        } while (VS_Read_SCI(SCI_MODE) & SM_CANCEL);

        xprintf("enc is stop\r\n");

        /* If using SCI for data transfers, read all remaining words using SCI_HDAT1/SCI_HDAT0. */
        int x = VS_Read_SCI(SCI_RECWORDS);
        xprintf("still in encoder =%d\r\n", x);
        while (x) {
            uint16_t w = VS_Read_SCI(SCI_RECDATA);
            //    *rbp++ = (uint8_t)(w >> 8);
            //    *rbp++ = (uint8_t)(w & 0xFF);
            x--;
        }
        x = VS_Read_SCI(SCI_RECWORDS);
        xprintf("yet still in encoder =%d\r\n", x);

        /*
        Then read parametric_x.endFillByte.
        If the most significant bit (bit 15) is set to 1, then the file is of an odd length
        and bits 7:0 contain the last byte that still should be written
        to the output file. Now write 0 to endFillByte.
        */
        uint16_t endfilbyte = VS_Read_Mem(PAR_END_FILL_BYTE); /* 0x1e06*/
        xprintf("endfil = %d\r\n", endfilbyte);
        // VS_Write_SDI(0);
        // VS_Write_SDI(0);

        /* When all samples have been transmitted, SM_ENCODE bit of SCI_MODE will be
        cleared by VS1063a, and SCI_HDAT1 and SCI_HDAT0 are cleared.
        */
        do {
            int x = VS_Read_SCI(SCI_RECWORDS);
            xprintf("x in encoder =%d\r\n", x);
        } while (VS_Read_SCI(SCI_MODE) & SM_ENCODE);


        /* free streambuffer memory */
        jack_ringbuffer_free(streambuffer);
        vPortFree(rp);
        vPortFree(buf);
    }
    return rv;
}


uint8_t send_stream(channel_t *p, volatile uint32_t *status)
{
    int8_t rx = stream_to_test_server(2, /* socket number */
                                      p->host,  /* hostname */
                                      p->port, /*port*/
                                      p->mount, /*mountpoint*/
                                      p->passw, /*password*/
                                      status /* check here to stop streaming */
                                     );
}

uint8_t receive_stream(channel_t *p, volatile uint32_t *status)
{
    int8_t rx = get_stream_from_server(2,p->host,p->ip,p->port,p->mount,p->passw,status);
}

uint8_t
get_stream_from_server(uint8_t sn, const char *host, const uint8_t *ip, const uint16_t port, const char *mntpnt, const char *password, volatile uint32_t *status)
{


    uint8_t host_ip[] = {0, 0, 0, 0};
//   uint8_t mijn_dns[] = { 0,0,0,0 };
    int rv = 0;
    // get DNS server ip
    //  getDNSfromDHCP(mijn_dns);

    //  uint8_t   *buf_dns = pvPortMalloc(MAX_DNS_BUF_SIZE);
    // init dns request
    //  DNS_init(1, buf_dns, xTaskGetTickCount() & 0xFFFF ); // gebruik voor DNS socket 1, 0 is in gebruik voor dhcp die af en toe kan zenden en ontvangen
    // do the request on given server

    if (host == NULL) {
        /* use bare IP */
        host_ip[0] = ip[0];
        host_ip[1] = ip[1];
        host_ip[2] = ip[2];
        host_ip[3] = ip[3];
    } else {

        uint8_t mijn_dns[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};

        int rv = 0;
        // get DNS server ip
        getDNSfromDHCP(mijn_dns[0]);
        // get DNS2 server ip
        getDNS2fromDHCP(mijn_dns[1]);

        uint8_t *buf_dns = pvPortMalloc(MAX_DNS_BUF_SIZE);
        uint8_t dns_counter = 0;
        while (dns_counter < 2) {
            // init dns request
            DNS_init(1, buf_dns, xTaskGetTickCount() & 0xFFFF); // gebruik voor DNS socket 1, 0 is in gebruik voor dhcp die af en toe kan zenden en ontvangen
            // do the request on given server
            int8_t rvx = DNS_run(mijn_dns[dns_counter], host, host_ip);
            if (rvx == 0) {
                xprintf("ip=%3d.%3d.%3d.%3d\r\n", host_ip[0], host_ip[1], host_ip[2], host_ip[3]);
                //vPortFree(buf_dns);
                break;
            } else {
                xprintf("dns error %d\r\n", rvx);
            }
            dns_counter++;
        }
        vPortFree(buf_dns);
        if (dns_counter == 2) {
            xprintf("dns fatal error\r\n");
            return -1;
        }



        /* do dns lookup */
        //      int8_t rvx = DNS_run(mijn_dns,p->host,host_ip);
        //      if (rvx == 0) {
        //          xprintf("ip=%3d.%3d.%3d.%3d\r\n",host_ip[0],host_ip[1],host_ip[2],host_ip[3]);
        //         vPortFree(buf_dns);
        //      } else {
        //         vPortFree(buf_dns);
        //         xprintf("dns error %d\r\n", rvx);
        //        return -1;
        //     }
    }


    rv = socket(sn, Sn_MR_TCP, (2048 + xTaskGetTickCount()) & 0xFFFF,/* SF_TCP_NODELAY*/ 0); // must be random number
    uint8_t param=1;
    setsockopt(sn,SO_KEEPALIVEAUTO,&param); // enable keepalive
    param=8;
    setsockopt(sn,SO_TOS,&param); // low delay
    xprintf("sock=%d\r\n", rv);
    rv = connect(sn, host_ip, port);
    xprintf("xcon=%d %s\r\n", rv, socket_error_to_string(rv));
    // rv = connect(2,host_ip,80);
    //  xprintf("con=%d\r\n",rv);

    const uint16_t bufsize = 2048; // size of buf in Wiznet voor deze socket

    char *buf = pvPortMalloc(bufsize); // pak buf van systeem
    if (buf == NULL) {
        goto cleanup;
    }

    // "GET /radio1-sb-aac HTTP/1.1\r\n" /* radio1-sb-aac  radio2-sb-aac 3 4 etc */
    int i;
    i = sprintf(buf, "GET /%s HTTP/1.1\r\n"
                //   "GET /live128 HTTP/1.1\r\n"
                "Host: %s\r\n"
                "User-Agent: VLC/2.0.8 LibVLC/2.0.8\r\n"
                "Range: bytes=0-\r\n"
                "Connection: close\r\n"
                "Icy-MetaData: 0\r\n\r\n", mntpnt, host);

    // xprintf("%s",buf);

    rv = send(sn, buf, i);
    xprintf("send=%d\r\n", rv);
    // skip header TODO TODO
    streambuffer = jack_ringbuffer_create(65535); // global variable
    // failed to create streambuffer
    if (streambuffer == NULL) {
        goto cleanup;
    }

    int32_t got_bytes;
    uint32_t error = 0;

    /* while we need to play this station continue */
    do {

        if (*status) {
            break;
        }

        // hoeveel data kunnen we handlen?
        // als ruimte in buffer dan verder

        uint32_t bf = jack_ringbuffer_write_space(streambuffer);
        uint32_t read_this_many;
        if (bufsize < bf) {
            // read max buffer size if we have space
            read_this_many = bufsize;
        } else {
            // read available space maximal
            read_this_many = bf;
        }

        if (read_this_many == 0) {
            // de buffer is vol, we moeten even niet lezen
            taskYIELD(); // geef de consumer ruimte
        } else {

            xprintf("still free: %u %u\r\n",jack_ringbuffer_write_space(streambuffer),read_this_many);
            got_bytes = recv(sn, buf, read_this_many);

            if (got_bytes > 0) {
                // put into ring buffer
                //
                if (jack_ringbuffer_write_space(streambuffer) >= got_bytes) {
                    uint32_t stuffed = jack_ringbuffer_write(streambuffer,buf,got_bytes);
                    if (stuffed < got_bytes) {
                        // corruption
                        xprintf("FATAL corruption\r\n");
                    }
                    player_active_flag = 1;
                    taskYIELD(); // geef de consumer ruimte
                } else {
                    // no space in buffer get out
                    taskYIELD();
                }
            } else {
                // network error or end of stream
                error=1;
            }
        }
    } while (error == 0);

    player_active_flag=0; // stop playout

    // wait for player to finish

    while (player_running == 1) {
        taskYIELD();
    }

cleanup:

    vPortFree(buf); // en terug aan systeem
    player_active_flag=0; // stop playout
    if (streambuffer != NULL) {
        jack_ringbuffer_free(streambuffer);
    }

    close(sn);
}

void vTaskPlayer(void *pvParameters)
{
    // wait for playout
    // uint32_t we_are_playing = 0;
    uint8_t *buf;
    const uint32_t bufsize = 2048;
    buf=pvPortMalloc(bufsize);
    if (buf==NULL) {
        xprintf("out of memory XXX");
        goto cleanup;
    }

    while (1) {

        if (player_active_flag) {
            // get stuff from ringbuffer and push it to the vs1063
            // we changed state
            if (player_running == 0) {
                // we started now

                // init the vs1063 if needed
                player_running = 1; // we own the ringbuffer now

            }
            // get stuff and push to chip
            uint32_t bc = jack_ringbuffer_read(streambuffer,buf,bufsize);
            VS_SDI_Write_Buffer(buf, bc);

        } else {
            if (player_running) {
                // cleanup
                player_running = 0;
                uint32_t tmp = jack_ringbuffer_read_space(streambuffer);
                jack_ringbuffer_read_advance(streambuffer,tmp);
                // flush zero's to vs1063
                // signal other tasks we are NOT using the ringbuffer
            }

            // sleep
            taskYIELD();
        }

    }

cleanup:
    xprintf("TaskPlayout DEAD\r\n");

}


void vTaskApplication(void *pvParameters)
{
    /* we gaan hier blocken tot dat de vlag van de Netwerk Bit 0x01 gezet is */

    EventBits_t xx = xEventGroupWaitBits(xEventBits, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);

    SERIAL_puts("Network flag is OK\r\n");
    VS_Hard_Reset();
    VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
    VS_Volume_Set((playout_volume << 8) | playout_volume);
    /* do sine test */
    VS_Test_Sine(1, 100); // sine on
    vTaskDelay(200); // pause
    VS_Test_Sine(0, 100); //  // sine off
    /* play network up sample */
    extern const uint8_t sample_ben[];
    VS_SDI_Write_Buffer((char *) sample_ben, 10421); // play sample 2
    VS_flush_buffers();

    VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
    VS_Volume_Set((playout_volume << 8) | playout_volume);

    while (1) {
        /* check nieuwe task */
        if (change_status) {
            change_status = 0;
            const channel_t *p = channels + active_channel;

            if (p->mode == pm_listening) {
                receive_stream(p, &change_status); // runs until change_status==1
                VS_cancel_stream();
                VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
                VS_Volume_Set((playout_volume << 8) | playout_volume);
            }

            if (p->mode == pm_sending) {
                send_stream(p, &change_status);
                VS_cancel_stream();
                VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
                VS_Volume_Set((playout_volume << 8) | playout_volume);
            }
        }
    }
}

int main(void)
{
    // RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    //  RNG_Cmd(ENABLE);

    QUEUE_INIT(my_TX_queue);
    QUEUE_INIT(my_RX_queue);
    QUEUE_INIT(my_KEYS_queue);


    USART2_hardware_init(115200);

    /* setup USART1 speed, only RX enabled */
    /* Please note, the Transmit from USART1 is disabled, the pin is used by the LCD */
    //  if (use_230400) {
    //      USART1_hardware_init(230400);    // moet hoger worden?
    //  } else {
    USART1_hardware_init(460800);
    //  }

//    uint16_t i;
//    for (i=0; i<255; i++) {
//        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty
//        USART_SendData(USART1, i); // send char
//    }


    SERIAL_puts("Boot\r\n");
    int i = 100;
    //  while (i) {
    //   while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    //           /* wait until new random number appears */
    //   }
    //     uint32_t rn = RNG_GetRandomNumber();
    //   xprintf("\r\n%d %ud",i,rn);


    //   i--;
    // }
    VS_SPI_GPIO_init();
    adc_configure();

    xSemaphoreSPI2 = xSemaphoreCreateMutex(); // SPI register protection
    xSemaphoreWRAM = xSemaphoreCreateMutex(); // internal ram of VS1063 protection
    xSemaphoreWIZCHIP = xSemaphoreCreateMutex(); // internal registers of W5500 protection

    if (xSemaphoreSPI2 == NULL || xSemaphoreWRAM == NULL || xSemaphoreWIZCHIP == NULL) {
        SERIAL_puts("Out of memory. xSemaphoreCreateMutex\r\n");
        while (1) {
            // we do not get here
        }

    }

    /* maak mutexen en event groups etc */
    xEventBits = xEventGroupCreate();
    if (xEventBits == NULL) {
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
    xSecondenTimer = xTimerCreate("TimerSec", 1000, pdTRUE, (void *) 1, vTimerCallback);
    xTimerStart(xSecondenTimer, 0);

    /* Create Start thread */
//   xTaskCreate(vTask1, "Ben_Thread", 256, NULL, 0, NULL);
    xTaskCreate(vTaskUserInterface, "Theo_Thread", 256, NULL, 0, NULL);
    xTaskCreate(vTaskApplication, "App", 512, NULL, 0, NULL);
    xTaskCreate(vTaskPlayer,"PlayOut",512,NULL,0,NULL);

    xTaskCreate(vTaskDHCP, "DHCP", 256, NULL, 0, NULL); /* stack must be large or crash will happen */
    // assert_failed("Test 1235", 8888);

    vTaskStartScheduler();
    /* Infinite loop */

    while (1) {
        // we do not get here
    }
}


/* called when stack is about to crash */
void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName)
{
    /* hang here so we can attach a debugger to find out what happened */
    for (; ;);
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

    char *s = (char *) file;
    USART_ITConfig(USART2, USART_IT_TXE | USART_IT_RXNE, DISABLE);
    while (*s) {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
            ;
        }
        USART_SendData(USART2, *s);
        s++;
    }

    snprintf( AssertStringBuffer, sizeof(AssertStringBuffer) , " %" PRIu32, line);
    s = AssertStringBuffer;
    while (*s) {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
            ;
        }
        USART_SendData(USART2, *s);
        s++;
    }

    while (1) {
        /* hang here so we can attach a debugger */
    }
}

#endif

