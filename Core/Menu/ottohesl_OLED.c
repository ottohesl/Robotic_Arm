#include "ottohesl_OLED.h"
#include <stdarg.h>
#include <stdio.h>

/**
 * @brief 显存缓冲区
 * @note 格式：[页(8行/页)][列] → 8页 × 128列，对应64行×128列的OLED屏幕
 */
static uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN]__attribute__((section(".ram"))) ;

/**
 * @brief 向OLED发送数据
 * @param data 待发送数据缓冲区
 * @param len 数据长度（字节数）
 * @retval None
 * @note 移植时需根据硬件平台修改I2C传输逻辑
 */
void OLED_Send(uint8_t *data, uint8_t len) {
    HAL_I2C_Master_Transmit(OLED_I2C_HANDLE, OLED_I2C_ADDR, data, len,50);
}

/**
 * @brief 向OLED发送指令
 * @param cmd 待发送的OLED指令码
 * @retval None
 * @note OLED I2C协议要求：指令传输前需发送0x00标志位
 */
void OLED_SendCmd(uint8_t cmd) {
    static uint8_t sendBuffer[2] = {0};
    sendBuffer[0] = 0x00;  // 指令传输标志位
    sendBuffer[1] = cmd;
    OLED_Send(sendBuffer, 2);
}

/**
 * @brief 清空OLED显存缓冲区
 * @retval None
 * @note 仅清空显存，需调用OLED_ShowFrame刷新到屏幕
 */
void OLED_Clear(void) {
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief OLED初始化函数
 * @retval None
 * @note 包含硬件初始化、参数配置、显存清空、显示开启等流程
 */
void OLED_Init(void) {
    HAL_Delay(100);
    OLED_SendCmd(0xAE);  // 关闭显示
    OLED_SendCmd(0x00);  // 设置低列地址
    OLED_SendCmd(0x10);  // 设置高列地址
    OLED_SendCmd(0x40);  // 设置显示起始行
    OLED_SendCmd(0x81);  // 对比度控制
    OLED_SendCmd(0xCF);  // 对比度值（0~255）
    OLED_SendCmd(0xA1);  // 段列映射（0xA1=正常，0xA0=反转）
    OLED_SendCmd(0xC8);  // COM行扫描方向（0xC8=正常，0xC0=反转）
    OLED_SendCmd(0xA6);  // 正常显示（0xA6=正常，0xA7=反色）
    OLED_SendCmd(0xA8);  // 多路复用率
    OLED_SendCmd(0x3f);  // 64行显示（0x3F=64）
    OLED_SendCmd(0xD3);  // 显示偏移
    OLED_SendCmd(0x00);  // 无偏移
    OLED_SendCmd(0xD5);  // 时钟分频
    OLED_SendCmd(0x80);  // 分频比
    OLED_SendCmd(0xD9);  // 预充电周期
    OLED_SendCmd(0xF1);  // 预充电时间
    OLED_SendCmd(0xDA);  // COM引脚配置
    OLED_SendCmd(0x12);  // 硬件配置
    OLED_SendCmd(0xDB);  // VCOMH电压
    OLED_SendCmd(0x30);  // VCOMH值
    OLED_SendCmd(0x20);  // 地址模式
    OLED_SendCmd(0x02);  // 页地址模式
    OLED_SendCmd(0x8D);  // 电荷泵
    OLED_SendCmd(0x14);  // 开启电荷泵
    OLED_Clear();        // 清空显存
    OLED_SendCmd(0xAF);  // 开启显示
    HAL_Delay(100);
}

/**
 * @brief 刷新显存到OLED屏幕
 * @retval None
 * @note 按页发送显存数据，每页包含128列×8行的像素信息
 */
void OLED_ShowFrame(void) {
    static uint8_t sendBuffer[OLED_COLUMN + 1];
    sendBuffer[0] = 0x40;  // 数据传输标志位
    for (uint8_t i = 0; i < OLED_PAGE; i++) {
        OLED_SendCmd(0xB0 + i); // 设置页地址（0~7）
        OLED_SendCmd(0x00);     // 设置列地址低4位
        OLED_SendCmd(0x10);     // 设置列地址高4位
        memcpy(sendBuffer + 1, OLED_GRAM[i], OLED_COLUMN);
        OLED_Send(sendBuffer, OLED_COLUMN + 1);
    }
}

/**
 * @brief 绘制单个像素点
 * @param x 列坐标（0~127）
 * @param y 行坐标（0~63）
 * @param t 显示状态：1-点亮，0-熄灭
 * @retval None
 * @note 超出坐标范围的点会被忽略
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t) {
    if(x >= OLED_COLUMN || y >= OLED_ROW) return;

    uint8_t page = y / 8;    // y行转页（0~7页）
    uint8_t bit = y % 8;     // y行在页内的位（0~7）
    uint8_t mask = 1 << bit;

    if(t) {
        OLED_GRAM[page][x] |= mask;
    } else {
        OLED_GRAM[page][x] &= ~mask;
    }
}

/**
 * @brief 获取UTF8字符长度
 * @param str 待检测的字符串指针
 * @retval 1-ASCII字符，3-UTF8汉字，0-非法字符/空指针
 * @note 内部辅助函数，仅支持ASCII和3字节UTF8汉字
 */
static uint8_t OLED_GetUTF8Len(const char* str) {
    if (str == NULL || *str == '\0') return 0;
    uint8_t c = (uint8_t)*str;
    if ((c & 0x80) == 0)      return 1;    // ASCII字符
    else if ((c & 0xF0) == 0xE0) return 3; // UTF-8汉字（3字节）
    else return 0;                         // 非法字符
}

/**
 * @brief 查找汉字对应的字库数据
 * @param chinese_char UTF8编码的汉字（3字节）
 * @param font_size 字体大小（仅支持FONT_16/FONT_24/FONT_32）
 * @retval 成功-字库数据指针，失败-NULL
 * @note 内部辅助函数，需配合oledfont.h中的汉字库使用
 */
static const uint8_t* OLED_FindChineseFont(const char* chinese_char, FONT_SIZE font_size) {
    if (chinese_char == NULL || *chinese_char == '\0') return NULL;
    if (font_size != FONT_16 && font_size != FONT_24 && font_size != FONT_32) return NULL;

    uint8_t utf8_len = OLED_GetUTF8Len(chinese_char);
    if (utf8_len != 3) return NULL;

    for (uint8_t i = 0; i < OLED_ChineseFontTable_Len; i++) {
        const OLED_ChineseFontMap* item = &OLED_ChineseFontTable[i];
        if (memcmp(chinese_char, item->utf8_char, 3) == 0) {
            switch (font_size) {
                case FONT_16: return item->font16;
                case FONT_24: return item->font24;
                case FONT_32: return item->font32;
                default: return NULL;
            }
        }
    }
    return NULL;
}

/**
 * @brief OLED格式化打印函数
 * @param x 起始列坐标（0~127）
 * @param y 起始行坐标（0~63）
 * @param font_size 字体大小（FONT_8/FONT_16/FONT_24）
 * @param mode 显示模式（OLED_NORMAL/OLED_HIGHLIGHT）
 * @param fmt 格式化字符串
 * @param ... 可变参数（与printf格式一致）
 * @retval None
 * @note 支持ASCII字符、3字节UTF8汉字、换行符\n，超出屏幕范围自动换行/截断
 */
void OLED_Printf(uint8_t x, uint8_t y, FONT_SIZE font_size, OLED_DisplayMode mode, const char *fmt, ...) {
    // 边界判断：起始坐标+字体大小需在屏幕内
    if(x >= OLED_COLUMN || y >= OLED_ROW || fmt == NULL) return;
    if (font_size != FONT_8 && font_size != FONT_16 && font_size != FONT_24) return;

    // 1. 可变参数格式化
    char buf[128] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // 2. 核心参数定义（固定字符尺寸，避免绘制中动态修改基准坐标）
    uint8_t curr_col = x;  // 当前列（x轴）
    uint8_t curr_row = y;  // 当前行（y轴）
    const uint8_t ascii_w = (font_size == FONT_8) ? 6 : (font_size / 2);  // ASCII字符宽度
    const uint8_t chinese_w = font_size;                                 // 汉字宽度
    const uint8_t line_h = (font_size == FONT_8) ? 8 : font_size;         // 行高
    const uint8_t char_row = (font_size == FONT_8) ? 1 : (font_size / 8); // 字符占用页数

    const char *p = buf;
    while(*p != '\0' && curr_row + line_h <= OLED_ROW) {
        // 处理换行符
        if(*p == '\n') {
            curr_col = x;
            curr_row += line_h;
            p++;
            continue;
        }

        // 识别字符类型
        uint8_t utf8_len = OLED_GetUTF8Len(p);
        if(utf8_len == 1) { // ASCII字符
            uint8_t chr = *p - ' ';
            // 列不足则换行
            if(curr_col + ascii_w > OLED_COLUMN) {
                curr_col = x;
                curr_row += line_h;
                if(curr_row + line_h > OLED_ROW) break;
            }

            // 逐行/逐列绘制ASCII字符
            for(uint8_t row_idx = 0; row_idx < char_row; row_idx++) {
                for(uint8_t col_idx = 0; col_idx < ascii_w; col_idx++) {
                    uint8_t font_data = 0;
                    uint16_t font_idx = row_idx * ascii_w + col_idx;
                    if(font_size == FONT_8)      font_data = asc2_0806[chr][font_idx];
                    else if(font_size == FONT_16) font_data = asc2_1608[chr][font_idx];
                    else if(font_size == FONT_24) font_data = asc2_2412[chr][font_idx];

                    // 逐位绘制像素
                    for(uint8_t bit_idx = 0; bit_idx < 8; bit_idx++) {
                        uint8_t draw_row = curr_row + row_idx * 8 + bit_idx;
                        uint8_t draw_col = curr_col + col_idx;
                        if(draw_col >= OLED_COLUMN || draw_row >= OLED_ROW) continue;
                        uint8_t draw_flag = (font_data >> bit_idx) & 0x01;
                        if(mode == OLED_HIGHLIGHT) draw_flag = !draw_flag;
                        OLED_DrawPoint(draw_col, draw_row, draw_flag);
                    }
                }
            }
            curr_col += ascii_w;
            p++;
        }
        else if(utf8_len == 3) { // 汉字字符
            const uint8_t* font_data = OLED_FindChineseFont(p, font_size);
            if(font_data == NULL) {
                curr_col += ascii_w;
                p += 3;
                continue;
            }

            // 列不足则换行
            if(curr_col + chinese_w > OLED_COLUMN) {
                curr_col = x;
                curr_row += line_h;
                if(curr_row + line_h > OLED_ROW) break;
            }

            // 逐行/逐列绘制汉字
            for(uint8_t row_idx = 0; row_idx < char_row; row_idx++) {
                for(uint8_t col_idx = 0; col_idx < chinese_w; col_idx++) {
                    uint16_t font_idx = row_idx * chinese_w + col_idx;
                    uint8_t font_byte = font_data[font_idx];

                    // 逐位绘制像素
                    for(uint8_t bit_idx = 0; bit_idx < 8; bit_idx++) {
                        uint8_t draw_row = curr_row + row_idx * 8 + bit_idx;
                        uint8_t draw_col = curr_col + col_idx;
                        if(draw_col >= OLED_COLUMN || draw_row >= OLED_ROW) continue;
                        uint8_t draw_flag = (font_byte >> bit_idx) & 0x01;
                        if(mode == OLED_HIGHLIGHT) draw_flag = !draw_flag;
                        OLED_DrawPoint(draw_col, draw_row, draw_flag);
                    }
                }
            }
            curr_col += chinese_w;
            p += 3;
        }
        else { // 非法字符
            curr_col += ascii_w;
            p++;
        }
    }
}

/**
 * @brief OLED显示颜色反转
 * @param i 反转控制：0-正常显示，1-反色显示
 * @retval None
 * @note 立即生效，无需刷新显存
 */
void OLED_ColorTurn(uint8_t i) {
    if(i == 0) {
        OLED_SendCmd(0xA6);
    } else {
        OLED_SendCmd(0xA7);
    }
}

/**
 * @brief OLED显示方向反转
 * @param i 方向控制：POSITIVE-正向，REVERSE-反向
 * @retval None
 * @note 方向修改后建议调用OLED_Clear+OLED_ShowFrame刷新显示
 */
void OLED_DisplayTurn(OLED_Display_Dir i) {
    if(i == POSITIVE) {
        OLED_SendCmd(0xC8);
        OLED_SendCmd(0xA1);
    } else {
        OLED_SendCmd(0xC0);
        OLED_SendCmd(0xA0);
    }
}

/**
 * @brief 开启OLED显示
 * @retval None
 * @note 与OLED_DisPlay_Off配合使用，仅控制显示开关，不丢失显存数据
 */
void OLED_DisPlay_On(void) {
    OLED_SendCmd(0x8D);
    OLED_SendCmd(0x14);
    OLED_SendCmd(0xAF);
}

/**
 * @brief 关闭OLED显示
 * @retval None
 * @note 关闭后显存数据保留，调用OLED_DisPlay_On可恢复显示
 */
void OLED_DisPlay_Off(void) {
    OLED_SendCmd(0x8D);
    OLED_SendCmd(0x10);
    OLED_SendCmd(0xAE);
}

/**
 * @brief 绘制圆形（中点画圆算法）
 * @param x 圆心列坐标（0~127）
 * @param y 圆心行坐标（0~63）
 * @param r 圆半径（像素数）
 * @retval None
 * @note 超出屏幕范围的圆会被自动裁剪
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r) {
    if(x + r >= OLED_COLUMN || x - r < 0 || y + r >= OLED_ROW || y - r < 0) return;

    int a, b, num;
    a = 0;
    b = r;
    while(2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y + b, 1);
        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        OLED_DrawPoint(x - b, y + a, 1);

        a++;
        num = (a * a + b * b) - r*r;
        if(num > 0) {
            b--;
            a--;
        }
    }
}

/**
 * @brief 绘制直线（Bresenham算法）
 * @param x1 起点列坐标（0~127）
 * @param y1 起点行坐标（0~63）
 * @param x2 终点列坐标（0~127）
 * @param y2 终点行坐标（0~63）
 * @param mode 显示模式（OLED_NORMAL-点亮，OLED_HIGHLIGHT-熄灭）
 * @retval None
 * @note 起点/终点超出屏幕范围时直线不会绘制
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_DisplayMode mode) {
    if(x1 >= OLED_COLUMN || y1 >= OLED_ROW || x2 >= OLED_COLUMN || y2 >= OLED_ROW) return;

    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uCol, uRow;

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uCol = x1;
    uRow = y1;

    incx = (delta_x > 0) ? 1 : (delta_x < 0) ? -1 : 0;
    incy = (delta_y > 0) ? 1 : (delta_y < 0) ? -1 : 0;
    delta_x = delta_x > 0 ? delta_x : -delta_x;
    delta_y = delta_y > 0 ? delta_y : -delta_y;
    distance = (delta_x > delta_y) ? delta_x : delta_y;

    for(t = 0; t < distance + 1; t++) {
        if(uCol < OLED_COLUMN && uRow < OLED_ROW) {
            OLED_DrawPoint(uCol, uRow, (mode == OLED_NORMAL) ? 1 : 0);
        }
        xerr += delta_x;
        yerr += delta_y;

        if(xerr > distance) {
            xerr -= distance;
            uCol += incx;
        }
        if(yerr > distance) {
            yerr -= distance;
            uRow += incy;
        }
    }
}

/**
 * @brief 显示位图图片
 * @param x 起始列坐标（0~127）
 * @param y 起始行坐标（0~63）
 * @param sizex 图片宽度（列数）
 * @param sizey 图片高度（行数）
 * @param BMP 位图数据缓冲区（按行存储，8行合并为1字节）
 * @param mode 显示模式（OLED_NORMAL/OLED_HIGHLIGHT）
 * @retval None
 * @note 图片超出屏幕范围时会被自动裁剪
 */
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], OLED_DisplayMode mode) {
    // 边界检测：图片超出屏幕则直接返回
    if(x >= OLED_COLUMN || y >= OLED_ROW) return;
    // 修正：计算实际可显示的尺寸（避免超出屏幕）
    uint8_t show_w = (x + sizex > OLED_COLUMN) ? (OLED_COLUMN - x) : sizex;
    uint8_t show_h = (y + sizey > OLED_ROW) ? (OLED_ROW - y) : sizey;

    uint16_t j = 0;          // 图片数据索引
    uint8_t byte_h = show_h / 8 + ((show_h % 8) ? 1 : 0); // 高度对应的字节数（8行=1字节）

    // 外层循环：按字节行遍历（每字节对应8行）
    for(uint8_t n = 0; n < byte_h; n++) {
        // 中层循环：按列遍历（宽度）
        for(uint8_t i = 0; i < show_w; i++) {
            uint8_t temp = BMP[j++]; // 取当前字节的图片数据
            // 内层循环：遍历字节的8个位（对应8行）
            for(uint8_t m = 0; m < 8; m++) {
                // 计算当前绘制的坐标
                uint8_t draw_x = x + i;
                uint8_t draw_y = y + n * 8 + m;
                // 确保坐标在屏幕范围内
                if(draw_x < OLED_COLUMN && draw_y < OLED_ROW) {
                    uint8_t draw_flag = (temp & 0x01) ? 1 : 0; // 取最低位
                    if(mode == OLED_HIGHLIGHT) {
                        draw_flag = !draw_flag; // 反色显示
                    }
                    OLED_DrawPoint(draw_x, draw_y, draw_flag); // 绘制像素点
                }
                temp >>= 1; // 右移一位，处理下一个位
            }
        }
    }
}