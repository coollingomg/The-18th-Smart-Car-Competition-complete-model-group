/*********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  RX                 查看 zf_device_wireless_uart.h 中 WIRELESS_UART_RX_PINx 宏定义
*                  TX                 查看 zf_device_wireless_uart.h 中 WIRELESS_UART_TX_PINx 宏定义
*                  RTS                查看 zf_device_wireless_uart.h 中 WIRELESS_UART_RTS_PINx 宏定义
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_wireless_uart_h_
#define _zf_device_wireless_uart_h_

#include "zf_common_typedef.h"
//=================================================无线串口 驱动配置====================================================
#define WIRELESS_UART_INDEX         (UART_2)                                    // 无线串口对应使用的串口号
#define WIRELESS_UART_BUAD_RATE     (115200)                                    // 无线串口对应使用的串口波特率
#define WIRELESS_UART_TX_PIN        (UART2_RX_P10_6)                            // 无线串口对应模块的 TX 要接到单片机的 RX
#define WIRELESS_UART_RX_PIN        (UART2_TX_P10_5)                            // 无线串口对应模块的 RX 要接到单片机的 TX
#define WIRELESS_UART_RTS_PIN       (P10_2)                                     // 无线串口对应模块的 RTS 引脚
//=================================================无线串口 驱动配置====================================================

//================================================无线串口 自动波特率====================================================
// 注意事项1：无线转串口模块版本是V2.0以下的是无法开启自动波特率的。
// 注意事项2：开启自动波特率务必连接RTS引脚 否则会开启失败。
// 注意事项3：模块自动波特率失败的话 可以尝试断电重启

// 开启自动波特率务必阅读上面两条 注意事项
// 开启自动波特率务必阅读上面两条 注意事项
// 开启自动波特率务必阅读上面两条 注意事项

// 0：关闭自动波特率
// 1：开启自动波特率 自动波特率的作用是修改 WIRELESS_UART_BAUD 之后不需要对模块进行配置 模块会自动设置为对应的波特率

#define WIRELESS_UART_AUTO_BAUD_RATE    (0)
//================================================无线串口 自动波特率====================================================

#define WIRELESS_UART_BUFFER_SIZE       (64  )
#define WIRELESS_UART_TIMEOUT_COUNT     (0x64)

//=================================================无线串口 基础函数====================================================
uint32      wireless_uart_send_byte         (const uint8 data);
uint32      wireless_uart_send_buff         (const uint8 *buff, uint32 len);
uint32      wireless_uart_send_string       (const char *str);
void        wireless_uart_send_image        (const uint8 *image_addr, uint32 image_size);

uint32      wireless_uart_read_buff         (uint8 *buff, uint32 len);

void        wireless_uart_callback          (void);

uint8       wireless_uart_init              (void);
//=================================================无线串口 基础函数====================================================

#endif


