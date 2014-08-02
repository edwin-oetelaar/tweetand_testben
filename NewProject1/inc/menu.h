/*
 * menu.h
 *
 *  Created on: Oct 3, 2011
 *      Author: oetelaar
 */

#ifndef MENU_H_
#define MENU_H_

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define KEY_ENTER  10
#define KEY_RIGHT  67
#define KEY_LEFT  68
#define KEY_DOWN  66
#define KEY_UP  65

#include "spliturl.h"
//#include "ipsilog.h" /* nodig voor ipsilog SPI SRAM module */

typedef struct EDIT_CONTEXT {
    const char *title;
    uint8_t (* handler)(struct EDIT_CONTEXT *, char); // uint8_t ip_edit(context_t *c, char key)
    void *ptrData; // ptr to 4 bytes or other data
    uint8_t init_done;
    uint8_t viewmode;
    uint8_t activechar; // inverted char
} context_t;

typedef struct {
    uint8_t *val;
    const char *txt[10]; // selections
    uint8_t maxval; // number of items-1
    char buf[20];
} select_t;

typedef struct {
    char *str;
    //	uint8_t len;
    uint8_t bufsize;
    const char *validchars;
} string_t;

typedef struct {
    uint32_t *val;
    uint32_t min;
    uint32_t max;
    char buf[20];
} number_t;

typedef struct {
    uint8_t *bb;
    char buf[20];
    const char *template;
} ip_t;

typedef struct PLAYLIST_ITEM {
    char *title; // truncate to 16 chars
    char *url;
    struct PLAYLIST_ITEM *next;
    struct PLAYLIST_ITEM *prev;
} pl_item_t;

typedef struct PLAYLIST_CONTEXT {
    uint16_t itemcount;
    uint8_t timeout;

    pl_item_t *fist_item;
    pl_item_t *current_item;

    uint8_t auto_next;

} playlist_t;

/*
 * VS_Write_SCI(SCI_AICTRL0, 24000U); //sample rate
	vTaskDelay(1);
	VS_Write_SCI(SCI_AICTRL1, 0x00); // AGC on
	vTaskDelay(1);
	VS_Write_SCI(SCI_AICTRL2, 4096U); // AGC max gain /1000
	vTaskDelay(1);
	VS_Write_SCI(SCI_AICTRL3, ADC_MODE_4 | ENC_FORMAT_MP3 ); // mp3 mono
	vTaskDelay(1);
	// VS_Write_SCI(SCI_WRAMADDR, ENC_BR_MODE_VBR | ENC_BITR_1000| 128U); // bitratemode|multiplier|bitrate
	VS_Write_SCI(SCI_WRAMADDR, 0xE010); // bitratemode|multiplier|bitrate
 *
 * */


typedef struct ENCODER_PRESET {
    uint16_t samplerate;
    uint16_t rec_volume;
    uint16_t max_agc_gain;
    uint16_t rec_format;
    uint16_t mode_bitrate;
} enc_preset_t;

typedef enum {
    DevMode_Player = 0,
    DevMode_RX_Ice = 1,
    DevMode_TX_Ice = 2,
    DevMode_TX_Ice_Serial = 3

} devicemode_t;

typedef struct PLAYER_CONTEXT {
    devicemode_t devicemode; // 0=player 1=RX ice 2=TX mp3 3=TX-serial
    uint8_t encoding_preset; // 0=mp3/24000Hz/16kbps/mono
    uint8_t state; // current state http parser
    uint8_t nextstate; // moore machine
    uint8_t playlist_state; // for playlist parser
    uint32_t playlist_number; // for playlist parser
    playlist_t *playlist; // data of the playlist
    //	uint32_t buffersize;
    //	uint32_t bufferfill;
    //	uint32_t secondsplayed;
    //	uint8_t seconds; // time of day
    //	uint8_t day; // time of day
    //	uint8_t month; // time of day
    //	uint8_t year; // 2000==0, 2010==10
    uint8_t socket; // use this socket
    uint8_t dns1[4];
    uint8_t dns2[4];
    // uint8_t ip[4]; // ip of host we try to connect
    // char *buf;
    http_req_t *req;
    uint32_t timeout; // for counting seconds
    // uint8_t is_icy; // response was ICY not HTTP
    uint16_t cstate; /**< content type that we think it is CS_PLAYLIST | CS_PLS etc */
    uint16_t response_code; // 200 or 304 or whatever, 0 means invalid
    uint16_t meta_interval;
    uint16_t meta_dc; // meta downcounter
    char *strm_buf; // about 1500 bytes of buf to read data from network
    uint16_t strm_buf_size; // size of strm_buf
    uint16_t strm_buf_index;
 /*    ringbuffer_t *rb;  TODO dit eruit halen zit in IPSILOG code */
    volatile uint8_t consumer_run; // flag to start VS playing the ringbuffer, player sets this on buf high level
    volatile uint8_t consumer_playout; // set when stream ended, not changed from Player
    portTickType ttStartOfBuffering; // timetick when buffering started
} radio_player_t;

#endif /* MENU_H_ */
