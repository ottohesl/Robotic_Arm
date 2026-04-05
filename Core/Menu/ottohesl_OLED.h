#ifndef OTTOHESL_OLED_H
#define OTTOHESL_OLED_H
#include <stdbool.h>
#include "oledfont.h"
#include "stm32h7xx_hal.h"
#include "string.h"
#include "i2c.h"

// 硬件配置
#define OLED_I2C_HANDLE    &hi2c3          // I2C4句柄
#define OLED_I2C_ADDR      0x78            // OLED I2C地址(0x3C<<1)

// OLED坐标/尺寸
#define OLED_COLUMN 128       // 列数(x:0~127)
#define OLED_ROW    64        // 行数(y:0~63)
#define OLED_PAGE   (OLED_ROW / 8)  // 页数(8行/页)

// 显示模式
typedef enum {
    OLED_NORMAL = 0,    // 正常显示
    OLED_HIGHLIGHT = 1  // 反色显示
} OLED_DisplayMode;

// 显示方向
typedef enum {
    POSITIVE = 0,   // 正向
    REVERSE  = 1,   // 反向
} OLED_Display_Dir;

// 字体大小
typedef enum {
    FONT_8  =  8,    // 8号字体
    FONT_16 = 16,    // 16号字体
    FONT_24 = 24,    // 24号字体
    FONT_32 = 32     // 32号字体
} FONT_SIZE;

// 函数声明
void OLED_Send(uint8_t *data, uint8_t len);
void OLED_SendCmd(uint8_t cmd);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowFrame(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_Printf(uint8_t x, uint8_t y, FONT_SIZE font_size, OLED_DisplayMode mode, const char *fmt, ...);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(OLED_Display_Dir i);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_DisplayMode mode);
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], OLED_DisplayMode mode);

#endif //OTTOHESL_OLED_H