#include "my_helpers.h"

uint8_t dump_mac(uint8_t *mac, void(*putsfunc)(const char *s))
{
    char *buf = pvPortMalloc(32); // we proberen zo min mogelijk stack te gebruiken, dus heap allocatie
    if (buf == NULL) {
        return -1;    // out of memory
    }
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", *mac, *(mac+1), *(mac+2), *(mac+3), *(mac+4), *(mac+5));
    putsfunc(buf);
    vPortFree(buf);
    return 0;
}

uint8_t dump_ip(uint8_t *ip, void(*putsfunc)(const char *s))
{
    char *buf = pvPortMalloc(32); // we proberen zo min mogelijk stack te gebruiken, dus heap allocatie
    if (buf == NULL) {
        return -1;    // out of memory
    }
    sprintf(buf, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    putsfunc(buf);
    vPortFree(buf);
    return 0;
}

uint8_t dump_network_info(void(*putsfunc)(const char *s))
{

    wiz_NetInfo *ni = pvPortMalloc(sizeof (wiz_NetInfo));
    if (ni == NULL) {
        goto cleanup;
    }

    uint8_t rv = ctlnetwork(CN_GET_NETINFO, ni);

    if (rv) {
        putsfunc("Error CN_GET_NETINFO");
        goto cleanup;
    }

    char tmpstr[10] = {0}; // 10 bytes op de stack
    rv = ctlwizchip(CW_GET_ID, tmpstr); // tmpstr should contain "W5500"
    if (rv) {
        putsfunc("Error CW_GET_ID");
        goto cleanup;
    }

    putsfunc("DEV : ");
    putsfunc(tmpstr);
    putsfunc("\r\nMAC : ");
    rv = dump_mac(ni->mac, putsfunc);
    if (rv) {
        goto cleanup;
    }
    putsfunc("\r\n IP : ");
    rv = dump_ip(ni->ip, putsfunc);
    if (rv) {
        goto cleanup;
    }
    putsfunc("\r\nMSK : ");
    rv = dump_ip(ni->sn, putsfunc);
    if (rv) {
        goto cleanup;
    }
    putsfunc("\r\n GW : ");
    rv = dump_ip(ni->gw, putsfunc);
    if (rv) {
        goto cleanup;
    }
    putsfunc("\r\nDNS : ");
    rv = dump_ip(ni->dns, putsfunc);
    if (rv) {
        goto cleanup;
    }
    putsfunc("\r\nCONF: ");
    switch (ni->dhcp) {
    case 1:
        putsfunc("STATIC");
        break;
    case 2:
        putsfunc("DHCP");
        break;
    default:
        putsfunc("Unknown");
    }
    return 0; // ok
cleanup:
    vPortFree(ni); // safe on NULL pointer by definition
    return -1; // error
}

uint8_t do_http_get(uint8_t sn , const char *url, void (writefunc)(const char *, uint32_t ))
{
// de socket is open naar de server, nu de http request erin
    TickType_t timeout = 1000; // timeout value 1 second

    char *buf = pvPortMalloc(128);
    if (buf == NULL) {
        goto cleanup;
    }
    send(sn,"GET ",4);
    send(sn,url,strlen(url));
    send(sn," HTTP/1.0\r\n\r\n",13);
    // we verwachten antwoord nu

    // we geven het 1 seconde om de header te laden
    TickType_t tstart = xTaskGetTickCount(); //  + 1000;
    uint32_t nbuf = 0;
    do {
        taskYIELD();
        //TickType_t tnow = xTaskGetTickCount();
        if ((xTaskGetTickCount() - tstart) > timeout) {
            break;
        }
        nbuf = getSn_RX_RSR(sn);
    } while ((nbuf < 13));

    if (nbuf < 13) {
        writefunc("timeout\r\n",9);

    } else {
        int32_t len=0;
        do {

            int32_t total=0;
            len=recv(sn,buf,128,8000); // 8 sec timeout
            if (len>0) {
                writefunc(buf,len);
                total+=len;
            }
        } while (len>0);
    }
    vPortFree(buf);
    return 0;
cleanup:
    return -1;
}



const char *socket_error_to_string(const int v)
{
    const char *rv;
    switch (v) {
    case SOCK_OK:
        rv="SOCK_OK";
        break;
    case SOCKERR_SOCKNUM:
        rv = "Invalid socket number";
        break;
    case SOCKERR_SOCKMODE:
        rv = "Invalid socket mode";
        break;
    case SOCKERR_SOCKINIT:
        rv = "Socket is not initialized";
        break;
    case SOCKERR_IPINVALID:
        rv = "Wrong server IP address";
        break;
    case SOCKERR_PORTZERO:
        rv = "Server port zero";
        break;
    case SOCKERR_TIMEOUT:
        rv = "Timeout occurred during request connection";
        break;
    case SOCK_BUSY:
        rv = "In non-block io mode, it returned immediately";
        break;
    case  SOCKERR_SOCKOPT:
        rv = "Invalid socket option";
        break;
    case  SOCKERR_SOCKCLOSED:
        rv = "Socket unexpectedly closed";
        break;
    case  SOCKERR_SOCKFLAG:
        rv = "Invalid socket flag";
        break;
    case  SOCKERR_SOCKSTATUS:
        rv = "Invalid socket status for socket operation";
        break;
    case  SOCKERR_ARG:
        rv = "Invalid argument";
        break;
    case  SOCKERR_DATALEN:
        rv = "Invalid data length for buffer";
        break;
    case  SOCKERR_BUFFER:
        rv = "Socket buffer is not enough";
        break;
    default:
        rv="unknown error";
    }
    return rv;
}
/* not working on stm32f401RET
//Lua: string = read_unique_device_id()
static int cpu_read_unique_device_id( uint32_t *L )
{
    uint32_t *id_addr = (uint32_t *)0x1fff7a10;

    int i;
    for( i = 0; i < 3; ++i ) {
        *L = *id_addr;
        L++;
        id_addr++;
    }
    return 1;
}
*/
