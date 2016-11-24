#include "vTaskDHCP.h"
#include "wizchip_hardware.h"
#include "my_helpers.h"

extern EventGroupHandle_t xEventBits; // global bitset

void vTaskDHCP(void *arg)
{
    (void) arg; /* unused */
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
          //  lcd_home(&LCD);
          //  lcd_set_cursor_position(&LCD, 0, 0);
          //  const char *xtxt = "Check Netw Cable";
          //  lcd_write(&LCD, xtxt, strlen(xtxt));
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
        if (gDATABUF == NULL) {
            SERIAL_puts("OOM: dhcp\n");
            // goto NoMemCleanup;
            return ;
        }

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
        // NoMemCleanup:
            // nothing
    }
}


