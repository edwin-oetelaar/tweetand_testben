#ifndef LCDSHIELD_H_INCLUDED
#define LCDSHIELD_H_INCLUDED
#include <inttypes.h>
#include <stdlib.h>
#include "stm32f4xx_conf.h"
// mijn poging om het LCD shield te laten werken 23:50 uur op vrijdag

typedef struct {
    //enum {};
    GPIO_TypeDef* RS_port; // port for RS
    GPIO_TypeDef* E_port; // port for RS
    GPIO_TypeDef* D0_port; // port for RS
    GPIO_TypeDef* D1_port; // port for RS
    GPIO_TypeDef* D2_port; // port for RS
    GPIO_TypeDef* D3_port; // port for RS
    GPIO_TypeDef* BL_port; // port for RS
    uint16_t RS_pin ; // pin (16bit) for RS
    uint16_t E_pin ;  //
    uint16_t D0_pin ; //
    uint16_t D1_pin ; //
    uint16_t D2_pin ; //
    uint16_t D3_pin ; //
    uint16_t BL_pin ; //
    uint32_t _status;
} lcd_context_t;

ErrorStatus lcd_init_context(lcd_context_t *ctx); // set the default values for the display in the context
ErrorStatus lcd_init_gpio(lcd_context_t *ctx); // init de GPIO en het display
void lcd_write(lcd_context_t *ctx, const char *b, uint32_t len); // write string to display
void lcd_set_cursor_position(lcd_context_t *ctx, int line, int col); // change cursor position in display
void lcd_set_backlight(lcd_context_t *ctx, int on); // turn backlight on or off for display
void lcd_set_cursor(lcd_context_t *ctx, int on, int blink); // set cursor on/off blink on the display
void lcd_cls(lcd_context_t *ctx); // clear the display,
void lcd_home(lcd_context_t *ctx); // set cursor to home posistion


#endif /* LCDSHIELD_H_INCLUDED */
