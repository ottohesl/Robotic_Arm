#ifndef OLED_MENU_H
#define OLED_MENU_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include "main.h"
#include "ottohesl_OLED.h"
#include "6DOF_Control.h"
// -------------------------- 硬件定义 --------------------------
#define KEY_EN_GPIO_Port   GPIOA
#define KEY_EN_Pin         GPIO_PIN_2    // Enter键
#define KEY_NE_GPIO_Port   GPIOB
#define KEY_NE_Pin         GPIO_PIN_10   // Next键

// -------------------------- 枚举定义 --------------------------
// 菜单层级（简化为固定3级）
typedef enum {
    MENU_LEVEL_1,  // 一级菜单（Logo+二级菜单入口）
    MENU_LEVEL_2,  // 二级菜单（目录+三级菜单入口/back）
    MENU_LEVEL_3,   // 三级菜单（数据显示+back）
    MENU_LEVEL_4   // 具体数据显示
} Menu_Level;

// 按键事件
typedef enum {
    KEY_EVENT_NONE,       // 无按键
    KEY_EVENT_NEXT,       // 下选
    KEY_EVENT_ENTER       // 确认（进入子菜单/返回）
} Key_Event;

// -------------------------- 扩展的菜单结构体 --------------------------
typedef struct {
    Menu_Level current_level;  // 当前菜单层级
    uint8_t selected_idx;      // 当前层级选中项索引
    uint8_t idx_max;           // 当前层级最大索引值
    uint8_t level2_idx;        // 保存二级菜单选中的索引
    uint8_t level3_idx;        // 保存三级菜单选中的索引
} Menu_Handle;

// -------------------------- 全局声明 --------------------------
extern Menu_Handle current_menu;    // 菜单句柄（唯一实例）
extern uint32_t key_scan_time;     // 按键扫描时间戳

// 核心函数
Key_Event get_key();
void level_1(Menu_Handle* current_menu);
void OLED_Serial_Printf(uint8_t Ser, OLED_DisplayMode mode,const char *fmt, ...);
void level_2(Menu_Handle* current_menu);
void level_3(Menu_Handle* current_menu);
void level_4(Menu_Handle* current_menu);
void oled_loop();
#endif // OLED_MENU_H