#ifndef WIZCHIP_HARDWARE_H_INCLUDED
#define WIZCHIP_HARDWARE_H_INCLUDED

#include <inttypes.h>

void init_Wiz_SPI_GPIO(void);
uint32_t wiz_hardware_reset_chip(void);
void wiz_chip_select(void);
void wiz_chip_deselect(void);
void wizchip_lock(void);
void wizchip_unlock(void);
uint8_t WIZ_SPI_ReadByte(void);
uint8_t WIZ_SPI_SendByte(uint8_t byte);

#endif /* WIZCHIP_HARDWARE_H_INCLUDED */
