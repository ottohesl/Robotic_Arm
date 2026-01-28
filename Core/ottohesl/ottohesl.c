/**
  ******************************************************************************
  * @file           : ottohesl.c
  * @author         : ottohesl
  * @date           : 2025/12/11
  * @brief          : 基于STM32 HAL库的串口格式化发送函数（轮询/DMA模式），支持调试输出
  * @attention      : 适配STM32 F1/F4/H7系列DMA内存地址限制，需配合链接脚本(LD)修改
  ******************************************************************************
  * @note
  * 1. 调试模式通过宏 debug_mode/LCD 控制，关闭后无冗余代码开销
  * 2. H7系列DMA无法访问0x20000000地址，需将缓存数组映射到0x24000000开始的SRAM2
  * 3. 依赖头文件 ottohesl.h（需定义 OTTOHESL_UART_BUFFER 缓存大小）
  * 4. 依赖LCD驱动函数（LCD_SetTextFont/LCD_SetColor等）、HAL库串口驱动
  ******************************************************************************
  *@更新日志              版本                      更新内容
  *2025/12/11           v1.0       封装串口、dma串口发送的集成函数，并配置调试模式
  *2025/12/23           v1.1
  *
 */
#include "ottohesl.h"

/**
 * @defgroup UART_DEBUG_CONFIG 串口调试配置宏
 * @brief 串口/LCD调试模式开关，编译阶段通过条件编译裁剪功能
 * @{
 */
#define debug_mode  0       /**< 串口调试模式开关：1=开启（调用uart_debugger），0=关闭 */
#define LCD         0       /**< LCD调试模式开关：1=开启（显示HAL状态到1.54寸LCD），0=关闭 */
#define open_freertos  0    /**< FREERTOS：1=开启(互斥锁)，0=关闭 */
/** @} */

/**
  * @brief  格式化串口发送函数（轮询模式：sprintf + HAL_UART_Transmit）
  * @param  huart: 串口句柄指针（如 &huart1）
  * @param  fmt: 格式化字符串（支持%d/%s/%f等占位符，如 "temp: %.2f°C"）
  * @param  ...: 可变参数列表，匹配fmt中的占位符
  * @retval 无
  * @note   1. 静态缓存区映射到SRAM2（.ram段），避免H7系列DMA地址限制
  *         2. 自动在发送内容末尾添加换行符\n，提升串口打印可读性
  *         3. 缓存溢出时发送"length error\n"错误提示，超时时间100ms
  *         4. debug_mode=1时，调用uart_debugger输出发送状态
  */
void OTTO_uart(UART_HandleTypeDef *huart, const char *fmt, ...) {
#if open_freertos
    osStatus_t mutex_ret = osMutexAcquire(uartMutexHandle, osWaitForever);
#endif
    /* 静态缓存区：映射到链接脚本定义的.ram段（SRAM2，0x24000000开始），避免H7 DMA地址限制 */
    static char message[OTTOHESL_UART_BUFFER];
    va_list args;            /* 可变参数列表 */
    /* 串口发送状态 */

    /* 1. 解析可变参数，格式化字符串到缓存 */
    va_start(args, fmt);
    int len = vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    /* 2. 检查格式化结果：长度异常则发送错误提示 */
    if (len < 0 || len >= sizeof(message)) {
        HAL_UART_Transmit(huart, (uint8_t *)"length error\n", strlen("length error\n"), 100);
        return;
    }

    /* 3. 末尾添加换行符（确保缓存有剩余空间） */
    if (len + 1 < sizeof(message)) {
        message[len] = '\n';
        len++;  // 换行符计入最终发送长度
    }

    /* 4. 轮询模式发送格式化数据 */
    HAL_StatusTypeDef uart_tx_status = HAL_UART_Transmit(huart, (uint8_t *) message, len, 100);
#if open_freertos
    osMutexRelease(uartMutexHandle);
#endif
    /* 5. 调试模式：输出发送状态（串口/LCD） */
#if debug_mode
    uart_debugger(huart, uart_tx_status);
#endif
}

/**
  * @brief  格式化串口发送函数（DMA模式：sprintf + HAL_UART_Transmit_DMA）
  * @param  huart: 串口句柄指针（如 &huart1）
  * @param  fmt: 格式化字符串（支持%d/%s/%f等占位符）
  * @param  ...: 可变参数列表，匹配fmt中的占位符
  * @retval 无
  * @attention 关键注意事项（H7系列必看）：
  *           1. H7的DMA控制器无法直接访问0x20000000起始的SRAM1地址，需将缓存映射到0x24000000开始的SRAM2
  *           2. 需在LD链接脚本末尾添加以下段定义，将.ram段映射到SRAM2：
  *              @code
  *              .sram_msg :
  *              {
  *                  . = ALIGN(4);
  *                  *(.ram)           // 匹配代码中的__attribute__((section(".ram")))
  *              } > RAM             // RAM段需指向SRAM2（0x24000000）
  *              @endcode
  * @note   1. 功能逻辑与轮询模式一致，仅发送接口替换为DMA（非阻塞）
  *         2. debug_mode=1时，调用uart_debugger输出DMA发送状态
  *         3. 缓存溢出时仍使用轮询模式发送错误提示
  */
void OTTO_uart_dma(UART_HandleTypeDef *huart, const char *fmt, ...) {
    /* 静态缓存区：映射到SRAM2的.ram段，规避H7 DMA地址限制 */
    static char message[OTTOHESL_UART_BUFFER] __attribute__((section(".ram")));
    va_list args;            /* 可变参数列表 */
    /* DMA发送状态 */

    /* 1. 解析可变参数，格式化字符串到缓存 */
    va_start(args, fmt);
    int len = vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    /* 2. 检查格式化结果：长度异常则发送错误提示（轮询模式） */
    if (len < 0 || len >= sizeof(message)) {
        HAL_UART_Transmit(huart, (uint8_t *)"length error\n", strlen("length error\n"), 100);
        return;
    }

    /* 3. 末尾添加换行符 */
    if (len + 1 < sizeof(message)) {
        message[len] = '\n';
        len++;
    }

    /* 4. DMA模式发送格式化数据（非阻塞） */
    HAL_StatusTypeDef uart_tx_status = HAL_UART_Transmit_DMA(huart, (uint8_t *) message, len);

    /* 5. 调试模式：输出发送状态 */
#if debug_mode
    uart_debugger(huart, uart_tx_status);
#endif
}

/**
  * @brief  串口发送状态调试函数（串口+LCD双端提示）
  * @param  huart: 串口句柄指针（用于发送状态提示）
  * @param  status: HAL库串口发送返回状态（HAL_OK/HAL_ERROR等）
  * @retval 无
  * @note   1. 仅debug_mode=1时会被调用，关闭后代码不参与编译
  *         2. LCD=1时，在1.54寸LCD上显示状态（红色字体，24号中文字体）
  *         3. 串口输出状态字符串（OK/ERROR/BUSY/TIMEOUT），LCD按行显示
  */
void uart_debugger(UART_HandleTypeDef *huart, HAL_StatusTypeDef status) {
#if LCD
    /* LCD调试模式：初始化字体/颜色/清屏 */
    LCD_SetTextFont(&CH_Font24);    /* 设置24号中文字体 */
    LCD_SetColor(LIGHT_RED);        /* 设置字体颜色：亮红 */
    static const uint16_t font_h = 24; /* 字体高度，用于计算LCD显示行间距 */
    LCD_Clear();                    /* 清屏，避免重叠显示 */
#endif

    /* 根据HAL状态分支处理 */
    switch (status) {
        case HAL_OK:
            /* 串口输出OK */
            HAL_UART_Transmit(huart, (uint8_t *)"OK\n", strlen("OK\n"), 100);
#if LCD
            /* LCD第0行显示HAL_OK */
            LCD_DisplayText(10, font_h * 0, "HAL_OK");
#endif
            break;

        case HAL_ERROR:
            /* 串口输出ERROR */
            HAL_UART_Transmit(huart, (uint8_t *)"ERROR\n", strlen("ERROR\n"), 100);
#if LCD
            /* LCD第1行显示HAL_ERROR */
            LCD_DisplayText(10, font_h * 1, "HAL_ERROR");
#endif
            break;

        case HAL_BUSY:
            /* 串口输出BUSY */
            HAL_UART_Transmit(huart, (uint8_t *)"BUSY\n", strlen("BUSY\n"), 100);
#if LCD
            /* LCD第2行显示HAL_BUSY */
            LCD_DisplayText(10, font_h * 2, "HAL_BUSY");
#endif
            break;

        case HAL_TIMEOUT:
            /* 串口输出TIMEOUT */
            HAL_UART_Transmit(huart, (uint8_t *)"TIMEOUT\n", strlen("TIMEOUT\n"), 100);
#if LCD
            /* LCD第3行显示HAL_TIMEOUT */
            LCD_DisplayText(10, font_h * 3, "HAL_TIMEOUT");
#endif
            break;

        default:
            /* 未知状态（预留扩展） */
            HAL_UART_Transmit(huart, (uint8_t *)"UNKNOWN\n", strlen("UNKNOWN\n"), 100);
#if LCD
            LCD_DisplayText(10, font_h * 4, "UNKNOWN");
#endif
            break;
    }
}