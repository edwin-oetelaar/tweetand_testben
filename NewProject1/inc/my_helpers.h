/* Wat helper functies 6 juli 2014 */

#ifndef __HELPERS_H__
#define __HELPERS_H__
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "socket.h"

#define WIZNET_SPI          SPI1 /* hier zit de wiznet chip wiz550io */

const char *socket_error_to_string(const int v);
uint8_t dump_mac(uint8_t *mac, void(*putsfunc)(const char *s));
uint8_t dump_ip(uint8_t *ip, void(*putsfunc)(const char *s));
uint8_t dump_network_info(void(*putsfunc)(const char *s));
uint8_t do_http_get(uint8_t sn , const char *url, void (writefunc)(const char *, uint32_t ));

#endif
