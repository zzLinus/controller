#include "main.h"
#define LEFT        0x27
#define RIGHT       0x26
#define UP					0x29
#define DOWN				0x2A
#define ON					0xA7
#define OFF					0xA6

typedef enum
{
    SET_PIXEL = 0x01,
    RESET_PIXEL = 0X00,

}PixelStatus;

void OLED_FullyFill(uint8_t fill_Data);
void OLED_FullyClear(void);
void OLED_Init(void);
void GUI_PUT_CHINESE(uint8_t x,uint8_t y,uint8_t* buffer);
void OLED_DrawBMP(int16_t x0,int16_t y0,int16_t L,int16_t H,uint8_t BMP[]);
void OLED_SetPixel(int16_t x, int16_t y, uint8_t set_pixel);
void OLED_RefreshRAM();
void OLED_ClearRAM();

int print_HZK16_string(unsigned char *buf, int len);

uint8_t Find_GKB(uint8_t buf1,uint8_t buf2,uint8_t *buffer);