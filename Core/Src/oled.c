#include "main.h"
#include "oled.h"
#include "i2c.h"
#include "usart.h"
#include "stdio.h"

#define IICx hi2c2

uint16_t const displayWidth  = 128;
uint16_t const displayHeight = 64;

uint32_t _fontaddress = 0x000000;


static uint8_t OLED_RAM[8][140];

void HAL_I2C_WriteByte(uint8_t addr,uint8_t data)
{
    uint8_t TxData[2] = {addr,data};
    HAL_I2C_Master_Transmit(&IICx,0x78,(uint8_t*)TxData,2,10);
}

void WriteCmd(uint8_t IIC_Command)
{
    HAL_I2C_WriteByte(0x00,IIC_Command);
}

void WriteDat(uint8_t IIC_Data)
{
    HAL_I2C_WriteByte(0x40,IIC_Data);
}

void OLED_Init(void)
{
    HAL_Delay(500);

    WriteCmd(0xAE);//开显示
    WriteCmd(0x20);//设置内存寻址模式

    WriteCmd(0x10);//页面寻址模式
    WriteCmd(0xB0);//页面寻址模式开始地址
    WriteCmd(0xC8);//设置COM输出扫描方向
    WriteCmd(0x00);//设置低劣地址
    WriteCmd(0x10);//设置高列地址

    WriteCmd(0x40);//--设置起始行地址

    WriteCmd(0x81);//--set contrast control register

    WriteCmd(0xFF);//亮度调节 0x00~0xff
    WriteCmd(0xA1);//--设置段重新映射0到127
    WriteCmd(0xA6);//--设置正常显示
    WriteCmd(0xA8);//--设置复用比(1 ~ 64)
    WriteCmd(0x3F);

    WriteCmd(0xA4);//0xa4,输出遵循RAM内容;0xa5,Output忽略RAM内容

    WriteCmd(0xD3);//-设置显示抵消
    WriteCmd(0x00);//-not offset
    WriteCmd(0xD5);//--设置显示时钟分频/振荡器频率
    WriteCmd(0xF0);//--设置分率
    WriteCmd(0xD9);//--设置pre-charge时期
    WriteCmd(0x22);
    WriteCmd(0xDA);//--设置com大头针硬件配置
    WriteCmd(0x12);
    WriteCmd(0xDB);//--设置vcomh
    WriteCmd(0x20);//0x20,0.77xVcc

    WriteCmd(0x8D);//--设置DC-DC
    WriteCmd(0x14);

    WriteCmd(0xAF);//--打开OLED面板

    OLED_FullyClear();
}

void OLED_ON()
{
    WriteCmd(0x8D);  //设置电荷泵
    WriteCmd(0x14);  //开启电荷泵
    WriteCmd(0xAF);  //OLED唤醒
}

void OLED_OFF()
{
    WriteCmd(0x8D);  //设置电荷泵
    WriteCmd(0x10);  //关闭电荷泵
    WriteCmd(0xAE);  //OLED休眠
}

//全屏填充
void OLED_RefreshRAM()
{
    // 页寻址模式填充
    for(uint16_t m = 0; m < displayHeight/8; m++)
    {
        WriteCmd(0xB0+m);		//设置页地址b0~b7
        WriteCmd(0x00);		//设置显示位置—列低地址00-0f
        WriteCmd(0x10);		//设置显示位置—列高地址10-1f
        for(uint16_t n = 0; n < displayWidth+4; n++)
        {
            WriteDat(OLED_RAM[m][n]);
        }
    }
}

//清除数据缓冲区
void OLED_ClearRAM(void)
{
    for(uint16_t m = 0; m < displayHeight/8; m++)
    {
        for(uint16_t n = 0; n < displayWidth; n++)
        {
            OLED_RAM[m][n] = 0x00;
        }
    }
}

//全屏填充 0x00~0xff
void OLED_FullyFill(uint8_t fill_Data)
{
    for(uint16_t m = 0; m < displayHeight/8; m++)
    {
        for(uint16_t n = 0; n < displayWidth; n++)
        {
            OLED_RAM[m][n] = fill_Data;
        }
    }
    OLED_RefreshRAM();
}

//全屏清除
void OLED_FullyClear()
{
    OLED_FullyFill(RESET_PIXEL);
}

//设置坐标像素点数据
void OLED_SetPixel(int16_t x, int16_t y, uint8_t set_pixel)
{
    if (x >= 0 && x < displayWidth && y >= 0 && y < displayHeight)
    {
        if(set_pixel)
        {
            OLED_RAM[y/8][x] |= (0x01 << (y%8));
        }
        else
        {
            OLED_RAM[y/8][x] &= ~(0x01 << (y%8));
        }
    }
}

//获得坐标像素点数据
PixelStatus OLED_GetPixel(int16_t x, int16_t y)
{
    if(OLED_RAM[y/8][x] >> (y%8) & 0x01)
    {
        return SET_PIXEL;
    }
    return	RESET_PIXEL;
}

void OLED_SetCursor(uint8_t X,uint8_t Y)
{
    WriteCmd(0xB0|Y);
    WriteCmd(0x10|((X&0xF0)>>4));
    WriteCmd(0x00|(X&0x0F));
}

void GUI_PUT_CHINESE(uint8_t x,uint8_t y,uint8_t* buffer)
{
    if(x > 0 && x <= displayWidth/16 && y > 0 && y <= displayHeight/16)
    {
        OLED_SetCursor((x-1)*16,(y-1)*2);
        for(uint8_t n = 0;n < 16;n++)
        {
            WriteDat(buffer[n]);
        }
        OLED_SetCursor((x-1)*16,(y-1)*2);
        for(uint8_t a = 16;a < 32;a++)
        {
            WriteDat(buffer[a]);
        }
    }
}

void OLED_DrawBMP(int16_t x0,int16_t y0,int16_t L,int16_t H,uint8_t BMP[])
{
    if (x0 >= 0 && x0 < displayWidth && x0+L <= displayWidth &&\
		y0 >= 0 && y0 < displayHeight && y0+H <= displayHeight) {

        uint8_t *p = (uint8_t *)BMP;
        for(int16_t y = y0; y < y0+H; y+=8)
        {
            for(int16_t x = x0; x < x0+L; x++)
            {
                for(int16_t i = 0; i < 8; i++)
                {
//					OLED_SetPixel(x, y+i, ((*((uint8_t *)BMP+(x-x0)+L*((y-y0)/8))) >> i) & 0x01);
                    OLED_SetPixel(x, y+i, ((*p) >> i) & 0x01);
                }
                p++;
            }
        }
    }
}

int print_HZK16_string(unsigned char *buf, int len)
{
    int i, j, k, n, z;
    int flag;
    unsigned char key[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};

    if (len < 32 || 0 != (len % 32))
    {
        printf("len is not multiple of 32 \n");
        return -1;
    }

    n = 0;
    do{
        for(k=0; k < 16; k++)
        {
            for(j=0; j < 2; j++)
            {
                for(i=0; i < 8; i++)
                {
                    flag = buf[k*2+j]&key[i];
                    printf("%s", flag?"▮":"▯");
                }
            }
            printf("\n");
        }
        n++;
        buf += 32;
        printf(" \n");
        printf(" \n");
    }while(n < len / 32);

    return 0;
}

