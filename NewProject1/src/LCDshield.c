#include "LCDshield.h"
#include "FreeRTOS.h"
#include "task.h"
/* voor het Wiznet Challenge project
 * alles is GPL
 * door Edwin van den Oetelaar
 * NB de scheduler van FreeRTOS must be running, or else vTaksDelay will hang forever
 */

// freetronicsLCDShield lcdshield(D8, D9, D4, D5, D6, D7, D3, A0);
// RS  E   D0   D1  D2  D3   BL A0
// PA9 PC7 PB5 PB4 PB10 PA8 PB6

static void _lcd_write_byte(lcd_context_t *ctx, int byte);  // internal function
static void _lcd_write_data(lcd_context_t *ctx, int byte);  // internal function
static void _lcd_write_command(lcd_context_t *ctx, int byte); // internal function

ErrorStatus lcd_init_context(lcd_context_t *ctx)
{
    // omdat alle pinnen op diverse GPIO poorten zitten moet ik per pin ook de poort weten
    // dit is de enige plek waar dat nodig is om te definieeren
    ctx->RS_pin = GPIO_Pin_9;
    ctx->RS_port = GPIOA;
    ctx->E_pin = GPIO_Pin_7;
    ctx->E_port = GPIOC;
    ctx->D0_pin = GPIO_Pin_5;
    ctx->D0_port= GPIOB;
    ctx->D1_pin = GPIO_Pin_4;
    ctx->D1_port= GPIOB;
    ctx->D2_pin = GPIO_Pin_10;
    ctx->D2_port= GPIOB;
    ctx->D3_pin = GPIO_Pin_8;
    ctx->D3_port= GPIOA;
    ctx->BL_pin = GPIO_Pin_0; /* was GPIO_Pin_6; */
    ctx->BL_port= GPIOB;
    return SUCCESS;
};

ErrorStatus lcd_init_gpio(lcd_context_t *ctx)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure); // clear the struct met default data

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE); // enable gpio clocks A B C allemaal

    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       // OUTPUT
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // PUSH PULL
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //
    GPIO_InitStructure.GPIO_Pin = ctx->BL_pin;
    GPIO_Init(ctx->BL_port, &GPIO_InitStructure); // BL
    //
    GPIO_InitStructure.GPIO_Pin = ctx->RS_pin;
    GPIO_Init(ctx->RS_port, &GPIO_InitStructure); // RS
    //
    GPIO_InitStructure.GPIO_Pin = ctx->E_pin;
    GPIO_Init(ctx->E_port, &GPIO_InitStructure);  // E
    //
    GPIO_InitStructure.GPIO_Pin = ctx->D0_pin;
    GPIO_Init(ctx->D0_port, &GPIO_InitStructure); // D0
    //
    GPIO_InitStructure.GPIO_Pin = ctx->D1_pin;
    GPIO_Init(ctx->D1_port, &GPIO_InitStructure); // D1
    //
    GPIO_InitStructure.GPIO_Pin = ctx->D2_pin;
    GPIO_Init(ctx->D2_port, &GPIO_InitStructure); // D2
    //
    GPIO_InitStructure.GPIO_Pin = ctx->D3_pin;
    GPIO_Init(ctx->D3_port, &GPIO_InitStructure); // D3

    // init de LCD

    GPIO_WriteBit(ctx->E_port,ctx->E_pin,1); // E=1
    GPIO_WriteBit(ctx->RS_port,ctx->RS_pin,0);  // RS=0
    vTaskDelay(15); // 15 ms
    //
    int i;
    for (i = 0; i < 3; i++) {
        _lcd_write_byte(ctx,0x03);
        // this command takes 1.64ms, so wait for it
        vTaskDelay(2); // 2 ms
    }

    // 4-bit mode
    _lcd_write_byte(ctx,0x02);
    _lcd_write_byte(ctx,0x28);
    // Function set 001 BW N F - -
    _lcd_write_byte(ctx,0x0C);
    // Display on/off control 0000 1 D C B (D(isplay) On/Off C(ursor) On/Off B(link) On/Off
    _lcd_write_byte(ctx,0x06);
    // Cursor Direction and Display Shift : 0000 01 CD S (CD 0-left, 1-right S(hift) 0-no, 1-yes
    lcd_cls(ctx);

    return SUCCESS;
}

// string naar de display
void lcd_write(lcd_context_t *ctx, const char *b, uint32_t len)
{
    while (len) {
        _lcd_write_data(ctx,*b);
        b++;
        len--;
    }
}

void lcd_set_cursor_position(lcd_context_t *ctx, int line, int col)
{
    _lcd_write_command(ctx, 0x80 + (line * 0x40) + col);
}

void lcd_set_backlight(lcd_context_t *ctx, int on)
{
    GPIO_WriteBit(ctx->BL_port,ctx->BL_pin,!!on);
}

void lcd_set_cursor(lcd_context_t *ctx, int on, int blink)
{
    int tmp = 0;

    if (blink) tmp = 0x01;
    if (on) tmp |= 0x02;
    _lcd_write_command(ctx,0x0C + tmp);
}

void lcd_cls(lcd_context_t *ctx)
{
    _lcd_write_command(ctx,0x01);
}

void lcd_home(lcd_context_t *ctx)
{
    _lcd_write_command(ctx,0x02);
}
// de belangrijkste : schrijf byte naar IO poort en toggle de E pin
// de E flank van 1 naar 0 klokt de data in het register
static void _lcd_write_byte(lcd_context_t *ctx, int byte)
{

    uint8_t tmp = (byte >> 4) & 0x0F;
    GPIO_WriteBit(ctx->E_port,ctx->E_pin,1); // hoog maken
    GPIO_WriteBit(ctx->D0_port,ctx->D0_pin,!!(tmp & 0x01));
    GPIO_WriteBit(ctx->D1_port,ctx->D1_pin,!!(tmp & 0x02));
    GPIO_WriteBit(ctx->D2_port,ctx->D2_pin,!!(tmp & 0x04));
    GPIO_WriteBit(ctx->D3_port,ctx->D3_pin,!!(tmp & 0x08));

    GPIO_WriteBit(ctx->E_port,ctx->E_pin,0); // flank naar 0, dit is de klok
    // blijkbaar geen wachttijd nodig

    tmp = byte & 0x0F;
    GPIO_WriteBit(ctx->E_port,ctx->E_pin,1); // hoog maken van E
    GPIO_WriteBit(ctx->D0_port,ctx->D0_pin,!!(tmp & 0x01));
    GPIO_WriteBit(ctx->D1_port,ctx->D1_pin,!!(tmp & 0x02));
    GPIO_WriteBit(ctx->D2_port,ctx->D2_pin,!!(tmp & 0x04));
    GPIO_WriteBit(ctx->D3_port,ctx->D3_pin,!!(tmp & 0x08));

    GPIO_WriteBit(ctx->E_port,ctx->E_pin,0);  // flank naar 0, klok de data
    // wacht 40 us maar we moeten 1000 wachten ivm timer resolutie, dit is een task switch
    vTaskDelay(1);
}

static void _lcd_write_data(lcd_context_t *ctx, int byte)
{
    GPIO_WriteBit(ctx->RS_port,ctx->RS_pin,1);  // RS=1
    _lcd_write_byte(ctx,byte);
}

static void _lcd_write_command(lcd_context_t *ctx, int byte)
{
    GPIO_WriteBit(ctx->RS_port,ctx->RS_pin,0);  // RS=0
    _lcd_write_byte(ctx,byte);
}

/* end of file */
