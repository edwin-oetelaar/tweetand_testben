/*
 * NADA internet radio receiver (radio) and transmitter (encoder) using Mp3 or Ogg-Vorbis
 * Circuit Cellar Challenge : wiznet 2014
 * Edwin en Ben 10 aug 2014 bitbucket (UPDATED nu, 2 december 20:20)
 * Ben Zijlstra voegt deze regel toe..
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
#include "kanalenlijst.h"
#include "vTaskDHCP.h"

#define min(a, b) (((a)<(b))?(a):(b))
static const char zeroes[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// extern channel_t channels[]; /* global channel list */

// const uint32_t max_channels = sizeof(channels) / sizeof(channels[0]);

uint8_t WIZ_SPI_ReadByte(void);
uint8_t WIZ_SPI_SendByte(uint8_t byte);
void vTimerCallback(void *ptr);

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

static uint8_t playout_volume = 30; //  not so loud
const uint8_t auto_start_first_channel = 1; // make music on startup

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

void SERIAL_write(const uint8_t *s, uint32_t len)
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
    uint32_t n = menu_channel; // get global var
    /* kanaal selectie */
    switch (key) {

    case 'D' :
        if (n < kl_get_count() - 1) {
            n++;
        } else {
            n = 0;
        }
        break;
    case 'U' :
        if (n > 0) {
            n--;
        } else {
            n = kl_get_count() - 1;
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

    const struct channel *p = kl_get_channel(n); // channels + n; // magic

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

static void handle_menu_pitch(uint8_t key)
{
    /* volume */
    // uint8_t n = VS_Read_SCI(SCI_VOL) & 0xFF;
    static uint8_t flag = 0;
    switch (key) {

    case 'D' :
        if (flag > 0) flag--;

        break;
    case 'U' :
        //VS_PitchControl_Set(0);
        if (flag < 13) flag++;
        break;
    }

    VS_PitchControl_Set(flag);

    lcd_set_cursor_position(&LCD, 0, 0);
    //              1234567890123456
    lcd_write(&LCD, "## Pitch ##", 20);
    lcd_set_cursor_position(&LCD, 1, 0);
    const char b1[] = ">>Enabled :";
    char buf[20] = {0,};
    strcat(buf,b1);
    int lx = strlen(b1);
    buf[lx] = '0' + flag;
    buf[lx+1] =0;

    const char b2[] = ">>Disabled<<    ";

    //char buf[20];
    //sprintf(buf, "%03d          ", 255 - playout_volume);
    const char *p ;
    if (flag) {
        p=buf;
    } else {
        p=b2;
    }
    lcd_write(&LCD, p, 16);
    // VS_Registers_Dump();
}

static void handle_menu_key(uint8_t key)
{
    //static n = 0;
    static uint8_t kolom = 0;
    const uint8_t colmax = 2; /* 0,1,2 */
    /* kolom 0 = channel selectie,
     * kolom 1 = volume control
     * kolom 2 = pitch control
     */
    /* left right is kolom */
    switch (key) {
    case 'L' :
        if (kolom > 0) {
            kolom--;
        } else {
            kolom=colmax;
        }


    case 'R' :
        if (kolom <colmax ) {
            kolom++;
        } else {
            kolom=0;
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
    case 2:
        /* volume instellen */
        handle_menu_pitch(key);
        break;

    }

}

static void vTaskUserInterface(void *arg)
{
    (void) arg;
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

    if (auto_start_first_channel == 1) {
        // fake the press select button
        SERIAL_write((uint8_t*)"Autostart\r\n",11);
        handle_menu_key('S'); // S is Select
    }

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

    len = recv(sn, buf, bufsize, 5000); // 5 sec timeout

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
        // int n = 0;
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
            // uint16_t w =
            VS_Read_SCI(SCI_RECDATA);
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


uint8_t parrot_stream(const struct channel *p, volatile uint32_t *status)
{
    // create buffer
    /* start data pump here */
    radio_player_t *rp = pvPortMalloc(sizeof(radio_player_t));
    streambuffer = jack_ringbuffer_create(32768); // global variable
    //xprintf("sb=%x",streambuffer);

    // VS_Hard_Reset();
    if ( rp && streambuffer) {

        int_fast8_t bezigblijven = 1;
        int_fast8_t listening = 0;
        int_fast8_t nn = 0;

        while (bezigblijven) {
            VS_Registers_Init();
            /* zet volume zacht van de output */
            VS_Volume_Set(0x4040);
            // xprintf("rec test serial %d\r\n", bufsize);
            memset(rp, 0, sizeof(radio_player_t));
            rp->devicemode = DevMode_TX_Ice_Serial;
            rp->use_line_input = 0; /* set to 1 if needed */
            rp->encoding_preset = 0; // mp3, low Q
            xprintf("tx via serial\r\n");
            recorder_active_flag = 1; // we have a ringbuffer so start using it
            VS_Encoder_Init(rp);

            /* data starts flowing into usart 1 now */
            // int n = 0;
            // int nn = 0;

            while (1) {

                if (*status) {
                    bezigblijven = 0; /* break parrot loop */
                    break;
                }

                /* we wachten tot er voldoende volume is gezien, we discarden alle data */
                /* check level of the recording */
                uint32_t lrmax = VS_Read_mem32(PAR_ENC_CHANNEL_MAX);
                uint16_t level = (uint16_t) lrmax & 0xFFFF;
//                xprintf("max = %u %u\r\n", (uint16_t) lrmax & 0xFFFF );
                VS_Write_mem32(PAR_ENC_CHANNEL_MAX, 0x00000000);

                if (level < 2000 && listening == 0) {
                    /* level niet gezien, alle data weggooien van de opname */
                    jack_ringbuffer_read_advance(streambuffer,jack_ringbuffer_read_space (streambuffer));
                    // xprintf("silence\r\n");
                    vTaskDelay(10);
                } else {
                    /* level gezien, opname starten */
                    listening = 1;
                    break;
                }
            }


            while (listening) {

                size_t inbuf = jack_ringbuffer_read_space(streambuffer);
                size_t frbuf = jack_ringbuffer_write_space(streambuffer);
                //int fullness = inbuf <<2  / bufsize << 2;
                char tt[32];
                sprintf(tt, "%5d %5d\r\n", inbuf, frbuf);
                SERIAL_puts(tt);

                vTaskDelay(10); //

                // nu kijken of er een maximum van < 2000 in buffer zit, dan ook stoppen met opname
                /* check level of the recording */
                uint32_t lrmax = VS_Read_mem32(PAR_ENC_CHANNEL_MAX);
                uint16_t level = (uint16_t) lrmax & 0xFFFF;
//                xprintf("max = %u %u\r\n", (uint16_t) lrmax & 0xFFFF );
                VS_Write_mem32(PAR_ENC_CHANNEL_MAX, 0x00000000);

                if (level < 2000 && listening == 1) {
                    nn++;
                    if (nn > 100) {
                        listening = 0; /* niet meer luisteren */
                        nn=0;
                        xprintf("silence quit rec\r\n");
                        break;
                    }
                }

                frbuf = jack_ringbuffer_write_space(streambuffer);
                if (frbuf == 0) {
                    xprintf("buffer vol\n");
                    listening = 0; /* niet meer luisteren */
                    break;
                }
            }


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
                // uint16_t w =
                VS_Read_SCI(SCI_RECDATA);
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

            /* set flag so irq not pushes data into buffer anymore */
            vTaskDelay(20);
            recorder_active_flag = 0;
            /* nu de playback zooi */
            // VS_cancel_stream();
            VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
            VS_Volume_Set((playout_volume << 8) | playout_volume);
            VS_PitchControl_Set( 6 /*10*/);
            player_active_flag=1;
            while (1) {
                taskYIELD();
                size_t inbuf = jack_ringbuffer_read_space(streambuffer);
                if (inbuf == 0) {
                    xprintf("WTF hh %d\r\n",inbuf);
                    break;
                }
            }
            player_active_flag=0;

            while (player_running) {
                /* wait until player flushed everything */
                taskYIELD();
            }

        }
        /* free streambuffer memory */
        jack_ringbuffer_free(streambuffer);
        vPortFree(rp);
        // VS_PitchControl_Set(0);
        // vPortFree(buf);
        // put stuff in buffer until full
        // play buffer until empty
    } else {
        /* no memory for parrot */
    }

    return 0;
}

uint8_t send_stream(const struct channel *p, volatile uint32_t *status)
{
    int8_t rx = stream_to_test_server(2, /* socket number */
                                      p->host,  /* hostname */
                                      p->port, /*port*/
                                      p->mount, /*mountpoint*/
                                      p->passw, /*password*/
                                      status /* check here to stop streaming */
                                     );
    return rx;

}

uint8_t receive_stream(const struct channel *p, volatile uint32_t *status)
{
    int8_t rx = get_stream_from_server(2,p->host,p->ip,p->port,p->mount,p->passw,status);
    return rx;
}

uint8_t
get_stream_from_server(uint8_t sn, const char *host,
                       const uint8_t *ip, const uint16_t port,
                       const char *mntpnt,
                       const char *password,
                       volatile uint32_t *status)
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

        //int rv = 0;
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

    if (rv != sn) return -1;

    uint8_t param=1;
    setsockopt(sn,SO_KEEPALIVEAUTO,&param); // enable keepalive
    param=8;
    setsockopt(sn,SO_TOS,&param); // low delay
    xprintf("sock=%d\r\n", rv);


    rv = connect(sn, host_ip, port);
    xprintf("xcon=%d %s\r\n", rv, socket_error_to_string(rv));

    if (rv < 0) return -2;

    // rv = connect(2,host_ip,80);
    //  xprintf("con=%d\r\n",rv);

    const uint16_t bufsize = 2048; // size of buf in Wiznet voor deze socket

    char *buf = pvPortMalloc(bufsize); // pak buf van systeem
    if (buf == NULL) {
        goto cleanup;
    }

    // "GET /radio1-sb-aac HTTP/1.1\r\n" /* radio1-sb-aac  radio2-sb-aac 3 4 etc */
    int i;
    buf[0] = 0;
    strcat(buf,"GET /");
    strcat(buf,mntpnt);
    strcat(buf," HTTP/1.1\r\nHost: ");
    strcat(buf,host);
    /* TODO hier password username toevoegen */
    (void) password;
    strcat(buf,"\r\n"
           "User-Agent: VLC/2.0.8 LibVLC/2.0.8\r\n"
           "Range: bytes=0-\r\n"
           "Connection: close\r\n"
           "Icy-MetaData: 0\r\n\r\n");
    /*   i = sprintf(buf, "GET /%s HTTP/1.1\r\n"
                   //   "GET /live128 HTTP/1.1\r\n"
                   "Host: %s\r\n"
                   "User-Agent: VLC/2.0.8 LibVLC/2.0.8\r\n"
                   "Range: bytes=0-\r\n"
                   "Connection: close\r\n"
                   "Icy-MetaData: 0\r\n\r\n", mntpnt, host);
    */
    xprintf("%s",buf);
    i = strlen(buf);
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

            xprintf("still free: %d %" PRIu32 "\r\n",jack_ringbuffer_write_space(streambuffer),read_this_many);
            got_bytes = recv(sn, buf, read_this_many, 5000); // 5 sec timeout

            if (got_bytes > 0) {
                // put into ring buffer
                //
                if (jack_ringbuffer_write_space(streambuffer) >= (size_t) got_bytes) {
                    size_t stuffed = jack_ringbuffer_write(streambuffer,buf,(size_t)got_bytes);
                    if (stuffed < (size_t) got_bytes) {
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
    return 0;
}

void vTaskPlayer(void *pvParameters)
{
    (void) pvParameters;
    // wait for playout
    // uint32_t we_are_playing = 0;
    char *buf;
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

                uint32_t tmp = jack_ringbuffer_read_space(streambuffer);
                jack_ringbuffer_read_advance(streambuffer,tmp);
                // flush zero's to vs1063
                // signal other tasks we are NOT using the ringbuffer
                VS_flush_buffers();
                player_running = 0;
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
    (void)pvParameters;
    /* we gaan hier blocken tot dat de vlag van de Netwerk Bit 0x01 gezet is */

    //EventBits_t xx = xEventGroupWaitBits(xEventBits, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    //(void) xx;

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
    //VS_PitchControl_Set(1); /* set pitch up == on */

    while (1) {
        /* check nieuwe task */
        if (change_status) {
            change_status = 0;
            const struct channel *p = kl_get_channel(active_channel);// channels + active_channel;

            if (p->mode == pm_listening) {
                int8_t rv = receive_stream(p, &change_status); // runs until change_status==1

                if ( rv < 0) {
                    xprintf("rv<0 on receive_stream what now?\n");
                }

                VS_cancel_stream();
                VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
                VS_Volume_Set((playout_volume << 8) | playout_volume);
                // VS_PitchControl_Set(1); /* set pitch up == on */
            }

            if (p->mode == pm_sending) {
                send_stream(p, &change_status);
                VS_cancel_stream();
                VS_Registers_Init(); // set alles op defaults, incl clocks en sound level
                VS_Volume_Set((playout_volume << 8) | playout_volume);
                // VS_PitchControl_Set(1); /* set pitch up == on */
            }

            if (p->mode == pm_parrot) {
                // detect level ??

                // record into buffer
                int8_t rv = parrot_stream(p, &change_status); // runs until change_status==1

                if ( rv < 0) {
                    xprintf("rv<0 on parrot_stream what now?\n");
                }
                // play out buffer
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
    //int i = 100;
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
    xTaskCreate(vTaskApplication, "App", 1024, NULL, 0, NULL);
    xTaskCreate(vTaskPlayer,"PlayOut",512,NULL,0,NULL);

    xTaskCreate(vTaskDHCP, "DHCP", 256, NULL, 0, NULL); /* stack must be large or crash will happen was 256*/
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
    (void) xTask;
    (void) pcTaskName;
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


void vTimerCallback(void *ptr)
{
    (void) ptr; /*unused*/
    DHCP_time_handler();
    DNS_time_handler();
    /* add music player timers here too ? */
}
