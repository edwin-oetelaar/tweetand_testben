#ifndef VTASKDHCP_H_INCLUDED
#define VTASKDHCP_H_INCLUDED

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
#include <inttypes.h>
#include "wizchip_conf.h"
#include "dhcp.h"
#include "my_queue.h"

extern void SERIAL_puts(const char *s);

void vTaskDHCP(void *arg);


#endif /* VTASKDHCP_H_INCLUDED */
