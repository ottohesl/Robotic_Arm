#ifndef __OLEDFONT_H
#define __OLEDFONT_H

/************************** 基础说明 **************************
* 本文件是 OLED 显示屏的字体库头文件，仅包含声明
* 字体数组的实际定义请放在 oledfont.c 文件中
* 1. ASCII 字符集：支持 8x6/12x12/16x16/24x24 四种点阵规格
* 2. 汉字库：支持 16x16/24x24/32x32/64x64 四种点阵规格
* 点阵数据存储规则：
* - 每个字符的点阵数据按「行」存储，每个字节表示 8 个像素点（高位在上/左，低位在下/右）
* - 例如 8x6 点阵：每个字符占 6 字节，每字节对应 8 行的一列像素
* - 汉字点阵字节数计算：点阵宽×点阵高 / 8（如 16x16 汉字 = 16*16/8 = 32 字节）
*************************************************************/
#include <stdint.h>

/************************** ASCII 字符集 声明 **************************/
// 8x6 点阵 ASCII 字符集（8行6列），覆盖空格(~)共95个可打印字符
extern const unsigned char asc2_0806[][6];

// 12x12 点阵 ASCII 字符集（12行12列），覆盖空格(~)共95个可打印字符
extern const unsigned char asc2_1206[95][12];

/**
 * asc2_1608: ASCII字符16x8点阵数据
 * 数组维度：[字符数][16]，每个字符占用16字节
 * 点阵规格：16行8列（高度16像素，宽度8像素）
 * 字符范围：空格(0x20)~波浪号(0x7E)，共95个可见ASCII字符（索引0对应空格，索引94对应~）
 * 存储格式：每行1个字节，共16行，对应16x8的点阵像素
 */
extern const unsigned char asc2_1608[][16];

/**
 * asc2_2412: ASCII字符24x12点阵数据
 * 数组维度：[字符数][36]，每个字符占用36字节
 * 点阵规格：24行12列（高度24像素，宽度12像素）
 * 字符范围：空格(0x20)~波浪号(0x7E)，共95个可见ASCII字符（索引0对应空格，索引94对应~）
 * 存储格式：每行1.5字节（12位），共24行，合计36字节，对应24x12的点阵像素
 */
extern const unsigned char asc2_2412[][36];

/************************** 汉字库 声明 **************************/
// Hzk1：16×16点阵汉字库，每个字符占32字节（16行×2字节/行）
extern const unsigned char Hzk1[][32];

// Hzk2：24×24点阵汉字库，每个字符占72字节（24行×3字节/行）
extern const unsigned char Hzk2[][72];

// Hzk3：32×32点阵汉字库，每个字符占128字节（32行×4字节/行）
extern const unsigned char Hzk3[][128];

extern const uint8_t Robotic_picture_Data[];
// ----------------- 字库映射结构体 -----------------
typedef struct {
    const char utf8_char[4];        // UTF-8中文字符（3字节+结束符）
    const uint8_t* font16;          // 16点阵字模
    const uint8_t* font24;          // 24点阵字模
    const uint8_t* font32;          // 32点阵字模
    const uint8_t* font64;          // 64点阵字模
} OLED_ChineseFontMap;

extern const OLED_ChineseFontMap OLED_ChineseFontTable[];
extern const uint16_t OLED_ChineseFontTable_Len;

#endif // __OLEDFONT_H