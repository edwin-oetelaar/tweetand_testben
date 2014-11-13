#ifndef _SPLIT_URL_H
#define _SPLIT_URL_H

#include <stdint.h>
// Edwin van den Oetelaar
// types needed by my Radio application

typedef struct {
    char *authority; // user:password@server.domain.tld
    char *hostname; // server.domain.tld
    char *abs_path; // /cgi-bin/aap.php
    char *query; // ?aap=noot
    char *cookie; // a session cookie
    char *parameter; // key
    char *pvalue; // value
    char *username; // auth user url user:pass@
    char *password; // auth pass url
    char *fragment; // fragment #fragment
    int8_t scheme; // 0=http 1=ftp 2=rtp 3=file,  -1=>invalid
    uint16_t port; // 80
    uint8_t ip[4]; // ip number of host
    uint8_t nocache; // prevent caching
    uint32_t device_serial; // eg. 12641 => X-Serial: 12641
    uint8_t useragent; // 0=I2R 1=Lukas2 2=SIR80 3=SIR100 4=KWR ==> User-Agent: KWR/1.15.285c (id=KWR12641; ethernet; build=SIKN)
    char versionstring[16]; // eg. "X-Version:" => "1.15.285c"


} http_req_t;

typedef struct {
    uint8_t s; /**< socket id 0..7 */
    uint8_t SERVERIP[4]; /**< server IP */
    uint16_t server_port; /**< server when not 0, portnumber */
    //	uint16 divider; /**< clock divider to compensate for streaming */
    uint16_t http_response; /**< 200 or 404 or 302 etc integer*/
    uint8_t *meta_string; /**< buffer for meta data, len is 16*32 max */
    uint8_t meta_string_len; /**< length of current meta data */
    uint32_t bytes_read; /**< bytes read count integer, used for content-length counter, and sending length ogg encoder */
    uint8_t meta_interval; /**< if >0 then means the meta interval is used, was set in header response */
    uint16_t meta_downcounter; /**< count down meta data, when 0 next byte is metalength (/16) */
    uint8_t hstate; /**< http connection state */
    uint16_t cstate; /**< content type that we think it is CS_PLAYLIST | CS_PLS etc */
    uint8_t playstate; /**< state of play, 0 wait for first data, 1 playing, 2 pause? 3=skipt? etc */
    int32_t speedindicator; /**< are we slow or fast, if positive we need to go faster, if negative go slower */
    uint16_t i; /**< counter used in reading header lines, index in header line string */
    uint8_t playlistseen; /**< state of playlist parsing 0=init 1=[playlist] gezien 2=[notplaylist] */
    /*	bit is_pls;
     bit is_m3u=0;
     bit is_playlist=0;
     bit is_data=0;
     bit is_mp3=0;
     bit is_aac=0;
     bit is_wma=0;
     bit is_ogg=0;
     bit is_icy=0;
     bit is_http=0; */
} conn_state_t;

// function declarations

// char spliturl(const char *baseurl, request_t *request);
http_req_t * http_req_init(void);
void http_req_destroy(http_req_t *r);
uint8_t http_req_from_url(const char *url, http_req_t *req);
int base64encode(const void* data_buf, // input buf
                 size_t dataLength, // input size
                 char* result, // output buf
                 size_t resultSize // output size
                );
size_t base64decode(const char *input, size_t inputlen, uint8_t *output,
                    size_t outputlen);
//uint8_t
//		parse_header_line(conn_state_t *cs, const char *line, const uint8_t len);
#endif
