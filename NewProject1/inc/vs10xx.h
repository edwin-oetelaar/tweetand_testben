/*
 * vs10xx.h
 *
 *  Created on: Sep 12, 2011
 *      Author: oetelaar
 * re-hacked op 29/7/2014 ivm wiznet challenge
 */

#ifndef VS10XX_H_
#define VS10XX_H_

/* new : hardware configuratie van de VS1063 ook hier
 * dit moet de enige plek zijn waar mapping van pinnen en poorten zit
 * eens kijken of dat werkt ...
 */

#define ON	(1)
#define OFF	(0)
/**/
#define HIGH	(1)
#define LOW		(0)
/**/
#define VS_xRESET       GPIO_Pin_4 /* PC4 ok */
#define VS_xRESET_PRT   GPIOC  /* ok */
/**/
#define VS_xCS			GPIO_Pin_12 /* PB12 ok */
#define VS_xCS_PRT		GPIOB /* ok */
/**/
#define VS_DREQ			GPIO_Pin_5	/* PC5 ok */
#define VS_DREQ_PRT     GPIOC /* ok*/
/**/
#define VS_SCLK			GPIO_Pin_13 /* PB13 ok */
#define VS_SCLK_PRT     GPIOB /* ok */
/**/
#define VS_MISO			GPIO_Pin_14 /* PB14 ok */
#define VS_MISO_PRT     GPIOB /* ok */
/**/
#define VS_MOSI			GPIO_Pin_15 /* PB15 ok*/
#define VS_MOSI_PRT     GPIOB /* ok */

/* ===  eind van de hardware config === */

// #include "app_config.h"
#include "menu.h"
#define DEFAULT_DIVIDER 0xE000 /* 9.8.4 SCI_CLOCKF (RW) 0xe000 is maximaal intern 61MHz*/ /* 0xC000 */ /* 0x9800 */

typedef struct {
    uint8_t profile; /* 1=voice 2=voice-high-mono 3=voice-wide-stereo 4=hifi-voice-mono 5=music-stereo */
    uint8_t quality; /* 0..10 */
    uint8_t samplerate; /* x1000 : 8->8000*/
    uint16_t bitrate; /* x1000 : 148-> 148000 bps */
} ogg_bitrate_t;

static const ogg_bitrate_t oggbitrates[] = { //
    // page 32 van vs1063a datasheet
    { 1, 0, 8, 6 }, //
    { 1, 1, 8, 7 },//
    { 1, 2, 8, 9 },//
    { 1, 3, 8, 10 },//
    { 1, 4, 8, 12 },//
    { 1, 5, 8, 13 },//
    { 1, 6, 8, 16 },//
    { 1, 7, 8, 19 },//
    { 1, 8, 8, 22 },//
    { 1, 9, 8, 25 },//
    { 1, 10, 8, 28 },//
    //
    { 2, 0, 16, 7 }, //
    { 2, 1, 16, 11 },//
    { 2, 2, 16, 14 },//
    { 2, 3, 16, 18 },//
    { 2, 4, 16, 21 },//
    { 2, 5, 16, 25 },//
    { 2, 6, 16, 31 },//
    { 2, 7, 16, 37 },//
    { 2, 8, 16, 43 },//
    { 2, 9, 16, 49 },//
    { 2, 10, 16, 55 }, //
    //
    { 3, 0, 16, 10 }, //
    { 3, 1, 16, 18 },//
    { 3, 2, 16, 26 },//
    { 3, 3, 16, 34 },//
    { 3, 4, 16, 42 },//
    { 3, 5, 16, 50 },//
    { 3, 6, 16, 65 },//
    { 3, 7, 16, 81 },//
    { 3, 8, 16, 96 },//
    { 3, 9, 16, 112 },//
    { 3, 10, 16, 127 },
    //
    { 4, 0, 48, 37 }, //
    { 4, 1, 48, 47 },//
    { 4, 2, 48, 57 },//
    { 4, 3, 48, 68 },//
    { 4, 4, 48, 78 },//
    { 4, 5, 48, 88 },//
    { 4, 6, 48, 99 },//
    { 4, 7, 48, 110 },//
    { 4, 8, 48, 122 },//
    { 4, 9, 48, 133 },//
    { 4, 10, 48, 144 },
    //
    { 5, 0, 48, 53 }, //
    { 5, 1, 48, 72 },//
    { 5, 2, 48, 91 },//
    { 5, 3, 48, 110 },//
    { 5, 4, 48, 129 },//
    { 5, 5, 48, 148 },//
    { 5, 6, 48, 185 },//
    { 5, 7, 48, 222 },//
    { 5, 8, 48, 259 },//
    { 5, 9, 48, 296 },//
    { 5, 10, 48, 333 }
};

/* outputs */
//#define VS_MOSI		P1_0 /**< OUTP microproc output slave input */
//#define VS_SCLK 	P1_1 /**< OUTP spi clock */
//#define VS_xCS  	P1_2 /**< OUTP not Chip Select */
//#define VS_xBSYNC 	P1_3 /**< OUTP not Chip Select Registers */
//#define VS_xRESET 	P1_4 /**< OUTP not Reset */

/* inputs */
//#define VS_MISO		P1_5 /**< INP microproc input slave output */
//#define VS_DREQ		P1_6 /**< INP Dreq, slave can handle more data */

/** VS10xx SCI Write Command byte is 0x02 */
#define VS_WRITE_COMMAND 0x02

/** VS10xx SCI Read Command byte is 0x03 */
#define VS_READ_COMMAND 0x03

#define SCI_MODE		0x00   /**< VS10xx register */
#define SCI_STATUS		0x01   /**< VS10xx register */
#define SCI_BASS		0x02   /**< VS10xx register */
#define SCI_CLOCKF		0x03   /**< VS10xx register */
#define SCI_DECODE_TIME	0x04   /**< VS10xx register */
#define SCI_AUDATA		0x05   /**< VS10xx register */
#define SCI_WRAM		0x06   /**< VS10xx register */
#define SCI_WRAMADDR	0x07   /**< VS10xx register */
#define SCI_HDAT0		0x08   /**< VS10xx register */
#define SCI_HDAT1		0x09   /**< VS10xx register */


/* SCI register recording aliases */

#define SCI_RECQUALITY 0x07 /* (WRAMADDR) VS1063 */
#define SCI_RECDATA    0x08 /* (HDAT0)    VS1063 */
#define SCI_RECWORDS   0x09 /* (HDAT1)    VS1063 */
#define SCI_RECRATE    0x0C /* (AICTRL0)  VS1063, VS1053 */
#define SCI_RECDIV     0x0C /* (AICTRL0)  VS1033, VS1003 */
#define SCI_RECGAIN    0x0D /* (AICTRL1)  VS1063, VS1053, VS1033, VS1003 */
#define SCI_RECMAXAUTO 0x0E /* (AICTRL2)  VS1063, VS1053, VS1033 */
#define SCI_RECMODE    0x0F /* (AICTRL3)  VS1063, VS1053 */



#define SCI_AIADDR		0x0a   /**< VS10xx register */
#define SCI_VOL			0x0b   /**< VS10xx register */
#define SCI_AICTRL0		0x0c   /**< VS10xx register */
#define SCI_AICTRL1		0x0d   /**< VS10xx register */
#define SCI_AICTRL2		0x0e   /**< VS10xx register */
#define SCI_AICTRL3		0x0f   /**< VS10xx register */

#define SM_DIFF				0x0001   /**< SCI_MODE bits */
#define SM_LAYER12			0x0002   /**< SCI_MODE bits */
#define SM_RESET			0x0004   /**< SCI_MODE bits */
#define SM_CANCEL			0x0008   /**< SCI_MODE bits */
//#define SM_EARSPEAKER_LO	0x0010   /**< SCI_MODE bits */
#define SM_TESTS			0x0020   /**< SCI_MODE bits */
//#define SM_STREAM			0x0040   /**< SCI_MODE bits */
//#define SM_EARSPEAKER_HI	0x0080   /**< SCI_MODE bits */
#define SM_DACT				0x0100   /**< SCI_MODE bits */
#define SM_SDIORD			0x0200   /**< SCI_MODE bits */
#define SM_SDISHARE			0x0400   /**< SCI_MODE bits */
#define SM_SDINEW			0x0800   /**< SCI_MODE bits */
#define SM_ENCODE        	0x1000   /**< SCI_MODE bits */
// #define SM_ADPCM_HP     	0x2000   /**< SCI_MODE bits */
#define SM_LINE1     		0x4000   /**< SCI_MODE bits */
#define SM_CLK_RANGE     	0x8000   /**< SCI_MODE bits */

// #define PARAMETRIC_VERSION 0x0003
#define PARAMETER_BASEADDR 0x1E00 /* for 1053b chip, change for other chip TODO */

// modes mono/stereo AGC set in SCI_AICTRL3
#define ADC_MODE_0 		0x0000 /* joint stereo common AGC*/
#define ADC_MODE_1 		0x0001 /* dual channel separate AGC */
#define ADC_MODE_2 		0x0002 /* LEFT channel */
#define ADC_MODE_3 		0x0003 /* RIGHT channel */
#define ADC_MODE_4 		0x0004 /* mono downmix */

// formats set in SCI_AICTRL3
#define ENC_FORMAT_IMA 			0x0000
#define ENC_FORMAT_PCM 			0x0010
#define ENC_FORMAT_ULAW 		0x0020
#define ENC_FORMAT_ALAW			0x0030
#define ENC_FORMAT_ADPCM 		0x0040
#define ENC_FORMAT_OGG 			0x0050
#define ENC_FORMAT_MP3 			0x0060

// flags set in SCI_AICTRL3
#define ENC_NO_RIFFWAV 			0x0400
#define ENC_PAUSE_ENABLE 		0x0800
#define ENC_UART_TX_ENABLE 		0x2000
#define ENC_ECHO_CANCEL_ENABLE 	0x4000
#define ENC_CODEC_MODE_ENABLE 	0x8000

/* flags for into WRAM_ADDR of encoding*/
#define ENC_BITR_10    		0x0000
#define ENC_BITR_100   		0x1000
#define ENC_BITR_1000  		0x2000
#define ENC_BITR_10000 		0x3000

#define ENC_BR_MODE_QUA 	0x0000
#define ENC_BR_MODE_VBR 	0x4000
#define ENC_BR_MODE_ABR 	0x8000
#define ENC_BR_MODE_CBR 	0xC000

#define CFG1_NOWMA 				(1<<15)
#define CFG1_NOAAC 				(1<<14)
#define CFG1_NOMP3 				(1<<13)
#define CFG1_NOFLAC 			(1<<12) /* To allow more memory for the user */
#define CFG1_PSNORMAL 			(0<<6)
#define CFG1_PSDOWNSAMPLED 		(1<<6) /* PS in downsampled mode */
#define CFG1_PSOFF 				(3<<6) /* no PS */
#define CFG1_SBRNORMAL 			(0<<4)
#define CFG1_SBRNOIMPLICIT 		(1<<4) /* default */
#define CFG1_SBRDOWNSAMPLED 	(2<<4) /* never upsample */
#define CFG1_SBROFF 			(3<<4) /* no SBR or PS */
#define CFG1_MP3_NOCRC 			(1<<8) /* turn off checking mp3 crc */
#define CFG1_REVERB 			(1<<0) /* not used on 1063 midi reverb */
#define AAC_SBR_PRESENT 		1
#define AAC_UPSAMPLE_ACTIVE 	2
#define AAC_PS_PRESENT 			4
#define AAC_PS_ACTIVE 			8

/*
 The following parametric structure is in X memory at address 0x1e00 and can be used to set
 extra parameters or get useful information. SCI_WRAMADDR addresses 0xc0c0 to 0xc0ff
 are translated automatically to parametric structure addresses 0x1e00..0x1e3f. Also, when an
 address from 0xc0c0 to 0xc0ff is written, sdiFree and audioFill are updated.
 */
#define PARAMETRIC_VERSION 0x0004
typedef struct {
    /* configs are not cleared between files */
    uint32_t chipID;
    /*0x1e00/01 Initialized at reset for your convenience*/
    uint16_t version;
    /*0x1e02 - structure version */
    uint16_t config1;
    /*0x1e03 wamf ---C ppss RRRR */
    int16_t playSpeed;
    /*0x1e04 0,1 = normal speed, 2 = twice, etc. */
    uint16_t bitRatePer100; /*0x1e05 average bitrate divided by 100 */
    uint16_t endFillByte;
    /*0x1e06 which byte value to send after file */
    int32_t rateTune;
    /*0x1e07..8 samplerate tune in +-1ppm steps. V4*/
    uint16_t playMode;
    /*0x1e09 play and processing enables V4 */
    int32_t sampleCounter; /*0x1e0a..b sample counter. V4*/
    uint16_t vuMeter;
    /*0x1e0c VU meter result V4*/
    uint16_t adMixerGain;
    /*0x1e0d AD mixer attenuation in 3dB steps -3..-31*/
    uint16_t adMixerConfig; /*0x1e0e AD mixer config, bits 5-4=rate, 7-6=mode */
    uint16_t pcmMixerRate; /*0x1e0f PCM mixer sample rate (read when enabled)*/
    uint16_t pcmMixerFree; /*0x1e10 PCM mixer FIFO free state */
    uint16_t pcmMixerVol;
    /*0x1e11 PCM mixer volume 0..191 (-0.5dB steps) */
    uint16_t eq5Params[10]; /*0x1e12..0x1e1b 5-channel EQ parameters */
    uint16_t eq5Updated;
    /*0x1e1c write as non-zero to recalculate filters.*/
    uint16_t speedShifter; /*0x1e1d Speed shifter speed 0x4000 == 1.0x V4 */
    uint16_t earSpeakerLevel; /*0x1e1e EarSpeaker level, 0 = off. V4*/
    uint16_t sdiFree;
    /*0x1e1f SDI FIFO free in words. V4*/
    uint16_t audioFill;
    /*0x1e20 Audio buffer fill in stereo samples. V4*/
    uint16_t reserved[4];
    /*0x1e21..24 */
    uint32_t latestSOF;
    /*0x1e25/1e26 latest start of frame V4 */
    uint32_t positionMsec; /*0x1e27-28 play position if known. V3*/
    int16_t resync;
    /*0x1e29 > 0 for automatic m4a, ADIF, WMA resyncs*/
    /* 42 words */
    union {
        /* 22 available -- these are not cleared at software reset! */
        uint16_t generic[22]; /*1e2a*/
        struct {
            int16_t txUartDiv;
            /*1e2a direct set of UART divider*/
            int16_t txUartByteSpeed;
            /*1e2b set UART byte speed (txUartDiv=0)*/
            uint16_t txPauseGpio;
            /*1e2c mask: a high level pauses tx*/
            int16_t aecAdaptMultiplier; /* 2 for default */
            int16_t reserved[14];
            uint16_t channelMax[2]; /*1e3c,1e3d for record level monitoring*/
            uint32_t serialNumber; /*1e3e,1e3f for Ogg Vorbis if enabled in WRAMADDR(11)*/
        } encoding;
        struct {
            uint32_t curPacketSize;
            uint32_t packetSize;
        } wma; /* 4*/
        struct {
            uint16_t sceFoundMask; /*1e2a single-channel-el. found since last clr*/
            uint16_t cpeFoundMask; /*1e2b channel-pair-el. found since last clr*/
            uint16_t lfeFoundMask; /*1e2c low-frequency-el. found since last clr*/
            uint16_t playSelect;
            /*1e2d 0 = first any, initialized at aac init */
            int16_t dynCompress; /*1e2e -8192=1.0, initialized at aac init */
            int16_t dynBoost;
            /*1e2f 8192=1.0, initialized at aac init */
            /* playSelect: 0 = first sce or cpe or lfe
             xxxx0001 first sce
             xxxx0010 first cpe
             xxxx0011 first lfe
             eeee0101 sce eeee
             eeee0110 cpe eeee
             eeee0111 lfe eeee */
            uint16_t sbrAndPsStatus; /*0x1e30 V3 gotSBR/upsampling/gotPS/PSactive*/
            uint16_t sbrPsFlags;
            /*0x1e31 V4*/
        } aac; /* 3*/
        struct {
            int16_t gain; /* 0x1e2a proposed gain offset, default = -12 */
        } vorbis;
    } i;
} VS1063_parametric;

#if 0
struct parametric {
    /* configs are not cleared between files */
    uint32_t chipID; /*1e00/01 Initialized at reset for your convenience */
    uint16_t version; /*1e02 - structure version */
    uint16_t config1; /*1e03 ---- ---- ppss RRRR PS mode, SBR mode, Reverb */
    uint16_t playSpeed; /*1e04 0,1 = normal speed, 2 = twice, 3 = three times etc. */
    uint16_t byteRate; /*1e05 average byterate */
    uint16_t endFillByte; /*1e06 byte value to send after file sent */
    uint16_t reserved[16]; /*1e07..15 file byte offsets */
    uint32_t jumpPoints[8]; /*1e16..25 file byte offsets */
    uint16_t latestJump; /*1e26 index to lastly updated jumpPoint */
    uint32_t positionMsec; /*1e27-28 play position, if known (WMA, Ogg Vorbis) */
    int16 resync; /*1e29 > 0 for automatic m4a, ADIF, WMA resyncs */
    union {
        struct {
            uint32_t curPacketSize;
            uint32_t packetSize;
        } wma;
        struct {
            uint16_t sceFoundMask; /*1e2a SCEs found since last clear */
            uint16_t cpeFoundMask; /*1e2b CPEs found since last clear */
            uint16_t lfeFoundMask; /*1e2c LFEs found since last clear */
            uint16_t playSelect; /*1e2d 0 = first any, initialized at aac init */
            int16 dynCompress; /*1e2e -8192=1.0, initialized at aac init */
            int16 dynBoost; /*1e2f 8192=1.0, initialized at aac init */
            uint16_t sbrAndPsStatus; /*0x1e30 1=SBR, 2=upsample, 4=PS, 8=PS active */
        } aac;
        struct {
            uint32_t bytesLeft;
        } midi;
        struct {
            int16 gain; /* 0x1e2a proposed gain offset in 0.5dB steps, default = -12 */
        } vorbis;
    } i;
};
#endif

//void VS_GPIO_Init(void); // init the chip hardware
void VLSI_SPI_GPIO_Init(void); // init spi en gpio van de m4 chip

void VS_SPI2_Init(uint8_t slow); // init the spi hardware port
uint8_t VS_Hard_Reset(void); // reset chip, can timeout
uint8_t VS_Soft_Reset(uint16_t clockregister); // soft reset
void VS_Registers_Init(void);//
void VS_Volume_Set(uint16_t vol);
uint16_t VS_Read_SCI(uint8_t reg); // read a register in vs10xx chip
void VS_Write_SCI(uint8_t reg, uint16_t val); // write register inside vs10xx chip
void VS_Registers_Dump(void); // test internal check vs10xx all register dump to screen
uint8_t SPI2_SendByte(uint8_t const byte); // tijdelijke fix
uint8_t VS_Dreq_Wait(uint32_t timeout); // timeout in ms
void SPI2_SendZeroBytes(uint8_t count); // send count 0x00 bytes to spi port 2
uint8_t VS_SDI_Write_Buffer(const uint8_t *buf, uint16_t len);
uint8_t VS_SDI_JAS_Buffer(const uint8_t *ptr, uint16_t len); // zonder DMA data erin jassen Ben Goed oplossing
void VS_Test_Sine(uint8_t onoff, uint8_t freq); // sine on/off 0x65 as freq works
uint8_t VS_Encoder_Init(radio_player_t *rp);
#endif /* VS10XX_H_ */
