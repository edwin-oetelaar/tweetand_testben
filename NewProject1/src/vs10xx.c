/*
 * vs10xx.c
 *
 *  Created on: Sep 12, 2011
 *      Author: oetelaar
 * modified 30/7/2014 Edwin
 */

/* Scheduler includes */
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "term_io.h"
#include "vs10xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
//#include "semphr.h"
#define _VSDEBUG_ 1
#include "data_blobs.h" // plugin and samples

extern const unsigned short plugin[]; /* zit nu in data_blobs.c */
extern xSemaphoreHandle xSemaphoreSPI2;
extern xSemaphoreHandle xSemaphoreWRAM;
const uint8_t use_230400 = 0 ; /* 460800 otherwise when 0 */

/* User application code loading tables for VS10xx */
void LoadUserCode(void)
{
    size_t i = 0;
    while (i < PLUGIN_SIZE) {
        unsigned short addr, n, val;
        addr = plugin[i++];
        n = plugin[i++];
        if (n & 0x8000U) { /* RLE run, replicate n samples */
            n &= 0x7FFF;
            val = plugin[i++];
            while (n--) {
                VS_Write_SCI(addr, val);
            }
        } else { /* Copy run, copy n samples */
            while (n--) {
                val = plugin[i++];
                VS_Write_SCI(addr, val);
            }
        }
    }
    /* check that DREQ shows the chip is ready to work, timeout 100ms */
    if (VS_Dreq_Wait(100)) {
        xprintf("could not load plugin\r\n");
    }
}


/* LEES application note AN2548
 * Je kan niet zomaar channels en DMA kanalen kiezen
 * bijv
 * SPI1_RX zit op DMA1_channel2
 * SPI1_TX zit op DMA1_channel3
 * SPI2_RX zit op DMA1_channel4
 * SPI2_TX zit op DMA1_channel5
 * */

void EXTI15_10_IRQHandler(void)
{
#if 0
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xprintf("INT11\n");
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        /* Toggle PC6 pin */
        //GPIO_WriteBit(GPIOC, GPIO_Pin_6, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_6))));
        /* Clear the EXTI line 9 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line11);
        xSemaphoreGiveFromISR(xSemaphoreDREQ,&xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE) {
            xprintf("PP\n");
            // We can force a context switch here.  Context switching from an
            // ISR uses port specific syntax.  Check the demo task for your port
            // to find the syntax required.
        }

    }
#endif
}

void DMA1_Channel5_IRQHandler(void)
{
#if 0
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    FlagStatus f = DMA_GetFlagStatus(DMA1_FLAG_TC5);
    FlagStatus g = DMA_GetFlagStatus(DMA1_FLAG_GL5);
    FlagStatus h = DMA_GetFlagStatus(DMA1_FLAG_HT5);
    FlagStatus i = DMA_GetFlagStatus(DMA1_FLAG_TE5);

    //xprintf("IRQ %d %d %d %d\n", f, g, h, i);

    if (f) {
        DMA_ClearITPendingBit(DMA1_FLAG_TC5);
        xSemaphoreGiveFromISR(xSemaphoreDMA1,& xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE) {

            //		xprintf("EE\n");
            // We can force a context switch here.  Context switching from an
            // ISR uses port specific syntax.  Check the demo task for your port
            // to find the syntax required.
            vPortYieldFromISR();
        }
    }
    // vPortYieldFromISR();
#endif
}

static void DMA_MemToSPI2(const char *buff, uint32_t btr)
{ // op de stm32f401ret zit spi2_tx channel0 op stream4 van dma1
//http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00046011.pdf
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA1_Stream4); // want we hebben spi2-tx, zet de DMA uit

    /* clean the init struct fill sane values */
    DMA_StructInit(&DMA_InitStruct);
    /* shared DMA configuration values */
    DMA_InitStruct.DMA_Channel=DMA_Channel_0; // SPI2 Tx DAM is DMA1/Stream4/Channel0
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) (&(SPI2->DR)); //
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // per byte
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // per byte
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // no inc on SPI address
    DMA_InitStruct.DMA_BufferSize = btr; // number of bytes to transfer
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal; // normal mode, not circular
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;

    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) buff;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral; // DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; // auto increment memory address
    DMA_InitStruct.DMA_FIFOMode  = DMA_FIFOMode_Disable; //Operate in 'direct mode' without FIFO

    /* set the DMA registers */
    DMA_Init(DMA1_Stream4, &DMA_InitStruct);

    /* Enable SPI TX request */
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx /*| SPI_I2S_DMAReq_Rx */, ENABLE);
    /* just make sure SPI2 is enabled*/
    SPI_Cmd(SPI2, ENABLE);
    /* excute DMA transfer command */
    DMA_Cmd(DMA1_Stream4, ENABLE);
    /* Wait until DMA1_Channel 4 Transfer Complete */
    while (DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET) {
        // vTaskDelay(0);
        taskYIELD();
    };

    /*!< Disable DMA TX Channel */
    DMA_Cmd(DMA1_Stream4, DISABLE);
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);
    /*!< oh boy, the dma is done, but the spi is still working on the last byte */
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
    /*!< Read out DR register to clear it */
    int temp=(SPI2->DR);
}

#if 0
void VS_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = VS_xCS | VS_xRESET;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, VS_xRESET); // not reset
    GPIO_SetBits(GPIOB, VS_xCS); // chip not selected

    // set VS_DREQ als input pin with pull up
    GPIO_InitStructure.GPIO_Pin = VS_DREQ; // PinB.11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#if 0
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* now some magic to get external interrupts working */
    /* Connect EXTI Line9 to PB.11  */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

    /* Configure EXTI Line11 to generate an interrupt on rising edge */
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* IRQ op IO pin nodig voor DREQ op pin 11 van Port B */
    /* Enable the EXTI9_5 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* end of the magic interrupt stuff */
#endif

    // SPI 2
    //   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); TODO WHAT IS THIS
    /* Configure SPIy pins: SCK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Pin = VS_MISO | VS_MOSI | VS_SCLK;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // yes also for MISO
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}
#endif

void VS_SPI_GPIO_init(void)
{
    // ok check op 31/7/2014
    // de VS1063 zit op SPI 2

    // en de CS zit op    PB12 (out)
    // de DREQ Int zit op PC5 (in)
    // de Reset zit op    PC4 (out)

    /**SPI2 GPIO Configuration
        PB13     ------> SPI2_SCK
        PB14     ------> SPI2_MISO
        PB15     ------> SPI2_MOSI
     */
    /**
        We gebruiken dus IO poort B en poort C
    */
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO
    SPI_InitTypeDef SPI_InitStruct;

    // enable Clocks for APB1 and GPIOB en C
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    // de pinnen van de SPI-2 instellen
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    // map de pin naar de SPI functie
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); //
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); //
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); //
    // de VS_xCS pin op PB12 is output
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_12; // Pins 12 CS
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    // de VS_xRESET zit op PC4
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_4; // Pins 4
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOC, &GPIO_InitStruct); // dit is GPIO C ok

    // de VS_DREQ zit op PC5
    // de inputs
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 ; // Pins 5 : Int
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // ok pull dit up dank u
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // ok dit is input
    GPIO_Init(GPIOC, &GPIO_InitStruct); // ok dit is GPIO C

    // enable the SPI clocks

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); // for SPI2, let op dit is APB1 niet 2 ok
    // ook de DMA clock enable zetten
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
    // set the SPI parameters to sane defaults
    SPI_StructInit(&SPI_InitStruct);
    // fill in the blanks
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master; // transmit in master mode
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // clock is low when idle
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; // data sampled at first edge
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS to software
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 56; // SPI frequency is
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB; // data is transmitted MSB first
    SPI_Init(SPI2, &SPI_InitStruct);

    SPI_Cmd(SPI2, ENABLE); // enable SPI2

}

#if 0

void VS_SPI_init_hardware(uint8_t slow)
{
    // setup up SPI2 hardware, slow or fast
    // pre: the global xSemaphoreSPI2 must be created
    // post: SPI2 is set to bytes, mode0, master, duplex

    SPI_InitTypeDef SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; /* we are master */
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; /* bytes, see spec */
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; /* mode0 see spec */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; /* mode0 */
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; /* yes we control NSS ( \chip_select) */
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; /* yes, see spec */
    SPI_InitStructure.SPI_CRCPolynomial = 7; /* dont know */
    // slow or fast
    if (slow) {
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;    /* go max speed not SPI_BaudRatePrescaler_4; */
    } else {
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;    /* go max speed not SPI_BaudRatePrescaler_4; */
    }

    SPI_Init(SPI2, &SPI_InitStructure);
    // enable SPI interface
    SPI_Cmd(SPI2, ENABLE);

    /* now do some magic on DMA interrupt */
    /* Enable the DMA1_Channel_IRQn Interrupt */
    //	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    //	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //	NVIC_Init(&NVIC_InitStructure);
}

#endif

// Jas music byte voor byte in de VS1063


void VS_Registers_Init(void)
{
    VS_Soft_Reset(DEFAULT_DIVIDER);
    // mode should be  0x4c20
    uint16_t regval = SM_SDINEW | // use SPI protocol
                      SM_TESTS | // enable tests via SDI
                      SM_LAYER12 | // enable mp3 layers
                      //  SM_LINE1   // use line input instead of mic */
                      SM_SDISHARE; // share xCS with xDCS

    // do a dummy read, make sure xCS etc are defined from example code..
    VS_Read_SCI(SCI_MODE); //
    // write regiser
    VS_Write_SCI(SCI_MODE, regval);
    // set sane volume
    VS_Volume_Set(0x0202);// some value
    VS_Write_SCI(SCI_BASS, 0x00);

}

void VS_Volume_Set(uint16_t vol)
{
    VS_Write_SCI(SCI_VOL, vol);
}

/*
 * reset vs10xx between track or bitstream errors
 * pre :
 * - IO ports must be set up
 * - timer must be running systick
 * - SPI must be configured
 * post :
 * - chip was reset using SPI commands return 0
 * - OR chip reset failed, return error code
 * */
uint8_t VS_Soft_Reset(uint16_t _div_reg)
{
    uint8_t rv = 0; // assume succes

    VS_Write_SCI(SCI_CLOCKF, _div_reg);
    vTaskDelay(5); // wait 5 ms
    if (!VS_Dreq_Wait(100)) {
        /* power down analog part, prevent plopping on reset */
        VS_Write_SCI(SCI_VOL, 0xFFFF);
        uint16_t regval = SM_SDINEW | SM_TESTS | SM_LAYER12 | SM_RESET
                          | SM_SDINEW /* | SM_LINE1 */ | SM_SDISHARE;

        // set bit 2 of mode reg to 1 for reset
        VS_Write_SCI(SCI_MODE, regval);
        vTaskDelay(5); // wait 5 ms

        //  wait some time for DREQ to come up
        rv = VS_Dreq_Wait(100);
        xprintf("plugin loading..");
        LoadUserCode();
    } else {
        // xprintf("DREQ timeout\n");
        rv = 1;
    }

#if _VSDEBUG_
    if (rv) {
        xprintf("Timeout in %s\r\n", __FUNCTION__);
    }
#endif
    return rv;

}

uint8_t VS_Dreq_Wait(const uint32_t timeout_ms)
{
    // short circuit
    uint8_t b = GPIO_ReadInputDataBit(VS_DREQ_PRT, VS_DREQ);
    if (b) {
        return 0;    // no timeout, done
    }
    // start with waiting
    uint32_t start_time = xTaskGetTickCount();
    uint32_t maxtime = start_time + timeout_ms; // max 1 second
    uint8_t wrapped = 0;
    if (start_time > maxtime) {
        // maxtime wrapped arround
        wrapped = 1;
    }

    uint8_t rv = 0;
    while (1) {
        b = GPIO_ReadInputDataBit(VS_DREQ_PRT, VS_DREQ);
        if (b) {
            // DREQ is high, good
            break;
        } else {
            if (wrapped) {
                // DREQ is still low, and maxtime is wrapped
                uint32_t now = xTaskGetTickCount();
                // timeout when time passed and start_time in future
                if ((maxtime < now) && (start_time > now)) {
                    // still low after maxtime, timeout
                    rv = 1;
                    break;
                }
            } else {
                // DREQ is still low
                if (maxtime < xTaskGetTickCount()) {
                    // still low after maxtime, timeout
                    rv = 1;
                    break;
                }
            }
        }
        vTaskDelay(0);
    }
    return rv;
}

/*
 * VS_Hard_Reset
 * pre :
 * - IO ports must be set up
 * - timer must be running, systick
 * post :
 * - the chip was reset
 * - a timeout happened when chip could not be reset
 * return :
 * 0 == success
 * else error code
 */
uint8_t VS_Hard_Reset(void)
{
    uint8_t rv = 0; // assume succesful reset
    const uint8_t timeout = 100; // timeout after 100ms
    // deselect chip
    GPIO_SetBits(VS_xCS_PRT, VS_xCS);
    // reset
    GPIO_ResetBits(VS_xRESET_PRT, VS_xRESET);
    // wait
    vTaskDelay(100);
    // end reset
    GPIO_SetBits(VS_xRESET_PRT, VS_xRESET);
    // wait on DREQ, timeout if takes too long
    uint32_t now = xTaskGetTickCount(); // just for benchmarking
    // wait here
    rv = VS_Dreq_Wait(timeout);
    // report wait time or problem

    /* op dit moment is de clock van de Vs1063 traag, de SPI moet nu ook langzaam */
#if _VSDEBUG_
    if (rv) {
        xprintf("Timeout in %s\r\n", __FUNCTION__);
        while (1)
            ;
    } else {
        xprintf("%s took %d ms\r\n", __FUNCTION__, (int) (xTaskGetTickCount()
                - now));
    }
#endif
    VS_Registers_Init(); // set some registers, incl speed
    /* de SPI mag nu snel TODO */

    // default to loading plugin
    xprintf("plugin loading..");
    LoadUserCode();
    xprintf("done\n");
    return rv;
}

// write single byte into data stream of VS chip
// when not writing registers (SCI) the control lines are in SDI mode by default
void VS_Write_SDI(uint8_t b)
{
    if (xSemaphoreTake(xSemaphoreSPI2,5000)) {
        //    GPIO_SetBits(VS_xCS_PRT, VS_xCS); // note, this is inverse of write Sci
        VS_SPI_SendByte(b);
        xSemaphoreGive(xSemaphoreSPI2);
    } else {
        xprintf("err sema spi2 wr sdi\n");
    }
}
/* write a single word to a (control) register inside the VS chip */
void VS_Write_SCI(uint8_t reg, uint16_t w_data)
{
    // grab VS_xCS
    if (xSemaphoreTake(xSemaphoreSPI2,5000)) {
        GPIO_ResetBits(VS_xCS_PRT, VS_xCS);
        VS_SPI_SendByte(VS_WRITE_COMMAND);
        VS_SPI_SendByte(reg);
        VS_SPI_SendByte((w_data >> 8) & 0xff);
        VS_SPI_SendByte(w_data & 0xff);
        // release VS_xCS
        GPIO_SetBits(VS_xCS_PRT, VS_xCS);
        xSemaphoreGive(xSemaphoreSPI2);
    } else {
        xprintf("err sema spi2 wr sci\n");
    }
}

/**
 * VS_Read_SCI(reg)
 * pre:
 * - IO porst must be set up
 * - SPI must be configured
 * - vs10xx chip must be reset
 * post:
 * - read value of vs10xx chip register using SPI
 * - return value of register uint16_t
 */
uint16_t VS_Read_SCI(uint8_t reg)
{
    if (xSemaphoreTake(xSemaphoreSPI2,5000)) {
        // grab VS_xCS
        GPIO_ResetBits(VS_xCS_PRT, VS_xCS);
        // send read command
        VS_SPI_SendByte(VS_READ_COMMAND);
        // send register index
        VS_SPI_SendByte(reg);
        // read first byte MSB
        uint16_t data = VS_SPI_SendByte(0x00) << 8;
        // second is LSB
        data |= VS_SPI_SendByte(0x00);
        // release VS_xCS
        GPIO_SetBits(VS_xCS_PRT, VS_xCS);
        xSemaphoreGive(xSemaphoreSPI2);
        return data;
    } else {
        xprintf("err sema spi2 wr sci\n");
        return 0xFFFF;
    }
}


/*
  Read 16-bit value from memory address.
*/
uint16_t VS_Read_Mem(uint16_t addr)
{
    uint32_t rv = 0xFFFF;
    if (xSemaphoreTake(xSemaphoreWRAM,100)) { // GRAB
        VS_Write_SCI(SCI_WRAMADDR, addr);
        rv = VS_Read_SCI(SCI_WRAM);
        xSemaphoreGive(xSemaphoreWRAM); // RELEASE
    } else {
        xprintf("sema err WRAM rm");
    }
    return rv;
}

/*
  Read 32-bit increasing counter value from addr.
  Because the 32-bit value can change while reading it,
  read MSB's twice and decide which is the correct one.
*/
uint32_t VS_Read_mem32_counter(uint16_t addr)
{
    uint16_t msbV1, lsb, msbV2;
    uint32_t res= 0xFFFF;
    if (xSemaphoreTake(xSemaphoreWRAM,100)) { // GRAB
        VS_Write_SCI(SCI_WRAMADDR, addr+1);
        msbV1 = VS_Read_SCI(SCI_WRAM);
        VS_Write_SCI(SCI_WRAMADDR, addr);
        lsb = VS_Read_SCI(SCI_WRAM);
        msbV2 = VS_Read_SCI(SCI_WRAM);
        xSemaphoreGive(xSemaphoreWRAM); // RELEASE
        if (lsb < 0x8000U) {
            msbV1 = msbV2;
        }
        res = ((uint32_t)msbV1 << 16) | lsb;
    } else {
        xprintf("sema err WRAM rm32cntr");
    }
    return res;
}

/*
 Read 32-bit non-changing value from addr under mutex protection
*/
uint32_t VS_Read_mem32(uint16_t addr)
{
    uint16_t lsb;
    uint32_t rv = 0xFFFF;
    if (xSemaphoreTake(xSemaphoreWRAM,100)) { // GRAB
        VS_Write_SCI(SCI_WRAMADDR, addr);
        lsb = VS_Read_SCI(SCI_WRAM);
        rv = lsb | ((uint32_t)VS_Read_SCI(SCI_WRAM) << 16);
        xSemaphoreGive(xSemaphoreWRAM); // RELEASE
    } else {
        xprintf("sema err WRAM rm32");
    }
    return rv;
}


/*
  Write 16-bit value to given VS10xx memory address
*/
void VS_Write_mem(uint16_t addr, uint16_t data)
{
    if (xSemaphoreTake(xSemaphoreWRAM,100)) { // GRAB
        VS_Write_SCI(SCI_WRAMADDR, addr);
        VS_Write_SCI(SCI_WRAM, data);
        xSemaphoreGive(xSemaphoreWRAM); // RELEASE
    } else {
        xprintf("sema err WRAM wr");
    }
}

/*
  Write 32-bit value to given VS10xx memory address
*/

void VS_Write_mem32(uint16_t addr, uint32_t data)
{
    if (xSemaphoreTake(xSemaphoreWRAM,100)) { // GRAB
        VS_Write_SCI(SCI_WRAMADDR, addr);
        VS_Write_SCI(SCI_WRAM, (uint16_t)data);
        VS_Write_SCI(SCI_WRAM, (uint16_t)(data>>16));
        xSemaphoreGive(xSemaphoreWRAM); // RELEASE
    } else {
        xprintf("sema err WRAM wr32");
    }
}

/* */
void VS_Test_Sine(uint8_t onoff, uint8_t freq)
{
#ifdef _VSDEBUG_
    xprintf("\r\nSine %s %d", onoff ? "ON" : "OFF", freq);
#endif
    uint8_t rv = VS_Dreq_Wait(100);
    if (!rv) {
        /* release xCS, in shared mode enabled SDI input, this is default */

        if (xSemaphoreTake(xSemaphoreSPI2,5000)) {
            GPIO_SetBits(VS_xCS_PRT, VS_xCS); // select CS
            if (onoff) {
                VS_SPI_SendByte(0x53);
                VS_SPI_SendByte(0xEF);
                VS_SPI_SendByte(0x6E);
                VS_SPI_SendByte(freq);
                SPI2_SendZeroBytes(0x10); // fill with 4 zero bytes
            } else {
                VS_SPI_SendByte(0x45);
                VS_SPI_SendByte(0x78);
                VS_SPI_SendByte(0x69);
                VS_SPI_SendByte(0x74);
                SPI2_SendZeroBytes(0x09);
            }
            GPIO_ResetBits(VS_xCS_PRT, VS_xCS); // De- select CS // MOET DIT ECHT Edwin?
            xSemaphoreGive(xSemaphoreSPI2);
        } else {
            xprintf("err sema spi2 sine\n");
        }
    } else {
#ifdef _VSDEBUG_
        xprintf("%s failed, dreq timeout\n", __FUNCTION__);
#endif
    }
}

void SPI2_SendZeroBytes(uint8_t count)
{
    while (count) {
        VS_SPI_SendByte(0x00);
        count--;
    }
}

/*

*/

uint8_t VS_SDI_Write_Buffer(const char *ptr, uint16_t len)
{
    uint16_t atonce = 32;
    uint16_t sdiFree=0;
    //	uint16_t audioFill;
    while (len) {
        /* new logic, use sdiFree instead of DREQ with default 32 byte value */
        // audioFill = VS_Read_SCI(SCI_WRAM);
        // xprintf("\r\nsdiFree=%u audioFill=%u ",sdiFree,audioFill);
        uint8_t b = GPIO_ReadInputDataBit(VS_DREQ_PRT, VS_DREQ);
        if (b) {
            /* Read sdiFree register */
            sdiFree = VS_Read_Mem(0xc0df); // set page 67 of datasheet ref 10.11.2
            sdiFree <<= 1; // multiply to get byte count
            atonce = (sdiFree < len) ? sdiFree : len;
            if (atonce) {
                if (xSemaphoreTake(xSemaphoreSPI2,1000)) {
                    GPIO_SetBits(VS_xCS_PRT, VS_xCS); // SDI
                    // xprintf("a\n");
                    DMA_MemToSPI2(ptr, atonce);
                    // xprintf("b\n");
                    ptr += atonce;
                    len -= atonce;
                    GPIO_ResetBits(VS_xCS_PRT, VS_xCS); // not SDI
                    xSemaphoreGive(xSemaphoreSPI2);
                } else {
                    // we could not get the semaphore for the SPI2 bus
                    // this means that there is a hangup in the SPI2 bus, someone has not released it
                    // normally the Semaphore is taken in about 0 ticks because we
                    // are the only task using serious bandwidth of the SPI2
                    xprintf("Err, SPI2 sema timeout\n");
                }
            } else {
                xprintf("Err, 0 byte SPI2 req\n");
            }
        } else {
            // chip busy
            // vTaskDelay(0); // yield
            taskYIELD();
        }
    }
    return 0;
}

uint8_t VS_SDI_JAS_Buffer(const char *ptr, uint16_t len)
{
    uint16_t atonce;
    uint16_t sdiFree;
    //	uint16_t audioFill;
    while (len) {
        /* new logic, use sdiFree instead of DREQ with default 32 byte value */
        // audioFill = VS_Read_SCI(SCI_WRAM);
        // xprintf("\r\nsdiFree=%u audioFill=%u ",sdiFree,audioFill);
        uint8_t b = GPIO_ReadInputDataBit(VS_DREQ_PRT, VS_DREQ);
        if (b) {
            /* read number of free WORDS in VS chip */
            sdiFree = VS_Read_Mem(0xc0df); // see page 67 of datasheet ref 10.11.2, reads 0x1E1F etc
            sdiFree <<= 1; // multiply to get byte count
            atonce = (sdiFree < len) ? sdiFree : len;
            if (atonce) {
                if (xSemaphoreTake(xSemaphoreSPI2,1000)) {
                    GPIO_SetBits(VS_xCS_PRT, VS_xCS); // SDI
                    #if 0
                    int i;
                    for (i=0; i<atonce; i++) { VS_SPI_SendByte(*(ptr+i)); }
                    #else
                    // we use DMA to handle pushing data to spi
                    DMA_MemToSPI2(ptr, atonce);
                    #endif
                    ptr += atonce;
                    len -= atonce;
                    GPIO_ResetBits(VS_xCS_PRT, VS_xCS); // not SDI
                    xSemaphoreGive(xSemaphoreSPI2);
                } else {
                    // we could not get the semaphore for the SPI2 bus
                    // this means that there is a hangup in the SPI2 bus, someone has not released it
                    // normally the Semaphore is taken in about 0 ticks because we
                    // are the only task using serious bandwidth of the SPI2
                    xprintf("Err, SPI2 sema timeout\n");
                }
            } else {
                xprintf("Err, 0 byte SPI2 req\n");
            }
        } else {
            // chip busy
            // vTaskDelay(0); // yield
            taskYIELD();
        }
    }
    return 0;
}
//
// print some info to serial
// pre: make sure that xSemaphoreWRAM is taken
// or nothing can interrupt it
void VS_Registers_Dump(void)
{
    /* read registers, dump contents  */
    // dit moet onder de WRAM mutex gebeuren TODO add mutex
    xprintf("\e[5;1H");
    uint8_t cnt;
    for (cnt = 0; cnt < 15; cnt++) {
        uint16_t value = VS_Read_SCI(cnt);
        xprintf("reg %d = %x\n", cnt, value);
    }
    /* read X memory stuff, dump contents */
    VS_Write_SCI(SCI_WRAMADDR, PARAMETER_BASEADDR);
    // lees chip id
    uint16_t hb = VS_Read_SCI(SCI_WRAM);
    uint16_t lb = VS_Read_SCI(SCI_WRAM);
    int32_t tmp = (hb << 16) + lb;
    xprintf("chipid = %" PRIu32 "  \n", tmp);
    lb = VS_Read_SCI(SCI_WRAM);
    xprintf("version %d  \n", lb);
    lb = VS_Read_SCI(SCI_WRAM);
    xprintf("config1 %d  \n", lb);
    lb = VS_Read_SCI(SCI_WRAM);
    xprintf("playspeed %d  \n", lb);
    lb = VS_Read_SCI(SCI_WRAM);
    xprintf("bitRatePer100 %d  \n", lb);
    lb = VS_Read_SCI(SCI_WRAM);
    xprintf("endFillByte %d  \n", lb);
    /* TODO goede 32 bit read functie gebruiken uit VS voorbeeld code */
    hb = VS_Read_SCI(SCI_WRAM);
    lb = VS_Read_SCI(SCI_WRAM);
    tmp = (hb << 16) + lb;
    xprintf("rateTune = %" PRId32 "   \n", tmp);
    //	endFillByte
    VS1063_parametric *par;
    //uint8_t *p;

    //	p = pvPortMalloc(sizeof(VS1063_parametric));

    //for (uint8_t cnt = 0; cnt < sizeof(VS1063_parametric); cnt += 2) {
    //		uint16_t value = VS_Read_SCI(SCI_WRAM);

    //		*(p + cnt) = (uint8_t) (value >> 8) & 0xFF;
    //		*(p + cnt + 1) = (uint8_t) value & 0xFF;
    //		xprintf("xmem %d = %x\n", cnt, value);
    //	}
    //	par = (VS1063_parametric*) p;
    //	xprintf("bitrate = %d\n", par->bitRatePer100);
    //	par->
    //	xprintf("chipid = %d\n", p->chipID);
    //	xprintf("version = %d\n", p->version);

    //	vPortFree(p);
}

uint8_t VS_SPI_SendByte(uint8_t const byte)
{
    // send a byte to SPI2
    // pre: xSemaphoreSPI2 is taken
    // pre: the Chip Selects are set
    // post: data is clocked out, and received over SPI2

    /* wait for Transmit empty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {
        ; /* spin here */
    }
    SPI_I2S_SendData(SPI2, byte);

    /* wait for receive not empty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {
        ; /* spin here */
    }
    /* get value from register */
    uint16_t tmp = SPI_I2S_ReceiveData(SPI2);

    return (uint8_t) tmp;
}

/* set the vs1063a into encoder mode, parameters will come later from rp
 * Example encoder samplerates,
 * XTALI = 12.288 MHz
 * Requested fs, Actual fs, Error, Note
 *
 * 48000 Hz   48000 Hz   0.00 %
 * 44100 Hz   44201 Hz  +0.23 % Not recommended for streaming applications.
 * 32000 Hz   32000 Hz   0.00 %
 * 24000 Hz   24000 Hz   0.00 %
 * 22050 Hz   22101 Hz  +0.23 % Not recommended for streaming applications.
 * 16000 Hz   16000
 * 12000 Hz   12000
 * 11025 Hz   11070 Hz  +0.41 % Not recommended for streaming applications.
 *  8000 Hz    8000
 */

static const enc_preset_t enc_presets[] = { {
        .samplerate= 24000U, /*samplerate 24000*/
        .rec_volume = 0, /* agc on */
        .max_agc_gain = 1024 *16, /* max gain agc maximum */
        .rec_format = RM_63_FORMAT_MP3 | RM_63_ADC_MODE_MONO, /* mono mp3 */
        .mode_bitrate = RQ_MODE_CBR | RQ_MULT_1000 | 32  // 0xE010, /* CBR MP3 16Kbps */
    }, {
        .samplerate=48000U, /* 48k */
        .rec_volume=0x00,/* agc on */
        .max_agc_gain =  4096U,/* max gain agc 4x */
        .rec_format = 0x0060, /* stereo mp3*/
        .mode_bitrate = 0xE080, /* CBR MP3 128 Kbps */
    }, {
        .samplerate=  48000U,
        .rec_volume =  0,
        .max_agc_gain = 4096U,
        .rec_format = 0x0050 /* stereo ogg */,
        .mode_bitrate = 5 /* Q mode 5 */

    }
};

#if 0
/* not used, */
unsigned char VScomm_get(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET) {
        //	xprintf("fl %x\n",USART2->SR);
        ;
    }
    if (USART_GetFlagStatus(USART2, USART_FLAG_FE)) {
        xprintf("FrameError\n");
    }
    return (unsigned char) USART_ReceiveData(USART2);
}

void VScomm_init(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USART2 configuration ------------------------------------------------------*/
    /* USART configured as follow:
     - BaudRate = 460800 baud (4x 115200 )
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 115200 << 2; /* 4x standard */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl
        =USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;

    /* Disable USART Receive and Transmit interrupts */
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

    /* Configure USART2*/
    USART_Init(USART2, &USART_InitStructure);
    USART_LINCmd(USART2,DISABLE);
    /* Enable the USART1 */
    USART_Cmd(USART2, ENABLE);
}

#endif

uint8_t VS_Encoder_Init(radio_player_t *rp)
{
    uint8_t rv = 1; // OK
    xprintf("encoding preset=%d\n", rp->encoding_preset);
    int i = rp->encoding_preset; /* number of the preset, niet de preset zelf */
    const enc_preset_t *myp = &(enc_presets[i]);
    LoadUserCode(); // just be sure the plugin is loaded
    // wait a little while for DREQ
//     WriteVS10xxMem32(PAR_ENC_SERIAL_NUMBER, 0x87654321 ); /* write RANDOM number info register for OGG serial number, optional */
    VS_Write_SCI(SCI_RECRATE, myp->samplerate); //sample rate
    // vTaskDelay(1);
    VS_Write_SCI(SCI_RECGAIN, myp->rec_volume); // AGC on
    // vTaskDelay(1);
    VS_Write_SCI( SCI_RECMAXAUTO, myp->max_agc_gain); // AGC max gain /1000
//   vTaskDelay(1);

    /* we doen de uart test */
    // zet de uart speed van de VS1063 op 460800
    // D1 = 12 D2=9
    // register is 0xC02B
    // D1 is high nibble (12 = 0x0C)
    // D2 is low nibble (9 = 0x09)
    //
    //  VS_Write_SCI(SCI_WRAMADDR, /* 0xC02B */0x1E2A);
    //  xprintf("0x1E2A=%x\n", VS_Read_SCI(SCI_WRAM));
    // prints 0x1E2A=420A
////    VS_Write_SCI(SCI_WRAMADDR, /* 0xC02B */0x1E2A);
//	VS_Write_SCI(SCI_WRAM, 0x090C); // deler =120 55.296/120=460800
//	VS_Write_SCI(SCI_WRAM, 0x00);
//	VS_Write_SCI(SCI_WRAM, 0x00);
    // alternative way to set baud rate
////    VS_Write_SCI(SCI_WRAM, 0x00);
////    VS_Write_SCI(SCI_WRAM, 11360U << 1); // 11520 << 2;
////    VS_Write_SCI(SCI_WRAM, 0x0000U);


    if (use_230400) {
        VS_Write_mem(PAR_ENC_TX_UART_DIV,0); /* do not use manual dividers for VS uart */
        VS_Write_mem(PAR_ENC_TX_UART_BYTE_SPEED,11360U << 1); /* use this Byte speed to calculate internal speed */
        VS_Write_mem(PAR_ENC_TX_PAUSE_GPIO,0); /* disable all flow control using IO pins of VS chip */
    } else {
        /* use 460800 */
        VS_Write_mem(PAR_ENC_TX_UART_DIV,0); // 0x1407); /* do found in vs1063an_EAC pdf  */
        VS_Write_mem(PAR_ENC_TX_UART_BYTE_SPEED,11360U << 2); /* use this Byte speed to calculate internal speed */
        VS_Write_mem(PAR_ENC_TX_PAUSE_GPIO,0); /* disable all flow control using IO pins of VS chip */
    }
    // Disable all interrupts except SCI
    // Write1053Sci(SCI_WRAMADDR, VS1053_INT_ENABLE);
    // Write1053Sci(SCI_WRAM, 0x2);

    if (rp->devicemode==DevMode_TX_Ice_Serial) {
        VS_Write_SCI(SCI_RECMODE, myp->rec_format | ENC_UART_TX_ENABLE  );
    } else {
        VS_Write_SCI(SCI_RECMODE, myp->rec_format);
    }
    vTaskDelay(1);
    // VS_Write_SCI(SCI_WRAMADDR, ENC_BR_MODE_VBR | ENC_BITR_1000| 128U); // bitratemode|multiplier|bitrate
    VS_Write_SCI(SCI_RECQUALITY, myp->mode_bitrate); // bitratemode|multiplier|bitrate

    vTaskDelay(1);
    uint16_t m = VS_Read_SCI(SCI_MODE);
    vTaskDelay(1);
    m |= SM_ENCODE;
    if (rp->use_line_input) {
        m |= SM_LINE1;
    }
    vTaskDelay(1);
    VS_Write_SCI(SCI_MODE, m); // set encode mode
    vTaskDelay(1);
    xprintf("encode :mode write is %4x\n", m);
    vTaskDelay(1);
    VS_Write_SCI(SCI_AIADDR, 0x50); // go start encoding now

    return rv;
}
