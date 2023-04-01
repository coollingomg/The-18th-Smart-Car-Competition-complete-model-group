#ifndef _zf_driver_uart_h_
#define _zf_driver_uart_h_

#include "ifxAsclin_Asc.h"
#include "zf_common_typedef.h"

#define UART0_TX_BUFFER_SIZE 256        // 定义串口0发送缓冲区大小
#define UART0_RX_BUFFER_SIZE 16         // 定义串口0接收缓冲区大小

#define UART1_TX_BUFFER_SIZE 256
#define UART1_RX_BUFFER_SIZE 16

#define UART2_TX_BUFFER_SIZE 256
#define UART2_RX_BUFFER_SIZE 16

#define UART3_TX_BUFFER_SIZE 256
#define UART3_RX_BUFFER_SIZE 16


typedef enum            // 枚举串口引脚 此枚举定义不允许用户修改
{
    UART0_TX_P14_0,     // 串口0 发送引脚可选范围
    UART0_TX_P14_1,
    UART0_TX_P15_2,
    UART0_TX_P15_3,

    UART1_TX_P02_2,     // 串口1 发送引脚可选范围
    UART1_TX_P11_12,
    UART1_TX_P15_0,
    UART1_TX_P15_1,
    UART1_TX_P15_4,
    UART1_TX_P15_5,
    UART1_TX_P20_10,
    UART1_TX_P33_12,
    UART1_TX_P33_13,

    UART2_TX_P02_0,     // 串口2 发送引脚可选范围
    UART2_TX_P10_5,
    UART2_TX_P14_2,
    UART2_TX_P14_3,
    UART2_TX_P33_8,
    UART2_TX_P33_9,

    UART3_TX_P00_0,     // 串口3 发送引脚可选范围
    UART3_TX_P00_1,
    UART3_TX_P15_6,
    UART3_TX_P15_7,
    UART3_TX_P20_0,
    UART3_TX_P20_3,
    UART3_TX_P21_7,
}uart_tx_pin_enum;


typedef enum            // 枚举串口引脚 此枚举定义不允许用户修改
{

    UART0_RX_P14_1,     // 串口0 接收引脚可选范围
    UART0_RX_P15_3,

    UART1_RX_P02_3,     // 串口1 接收引脚可选范围
    UART1_RX_P11_10,
    UART1_RX_P15_1,
    UART1_RX_P15_5,
    UART1_RX_P20_9,
    UART1_RX_P33_13,

    UART2_RX_P02_0,     // 串口2 接收引脚可选范围
    UART2_RX_P02_1,
    UART2_RX_P10_6,
    UART2_RX_P14_3,
    UART2_RX_P33_8,

    UART3_RX_P00_1,     // 串口3 接收引脚可选范围
    UART3_RX_P15_7,
    UART3_RX_P20_3,
    UART3_RX_P21_6,
}uart_rx_pin_enum;


typedef enum            // 枚举串口号 此枚举定义不允许用户修改
{
    UART_0,
    UART_1,
    UART_2,
    UART_3,
}uart_index_enum;

//创建串口handle变量
extern IfxAsclin_Asc uart0_handle;
extern IfxAsclin_Asc uart1_handle;
extern IfxAsclin_Asc uart2_handle;
extern IfxAsclin_Asc uart3_handle;

//====================================================串口 基础函数====================================================
void    uart_write_byte_wait                (uart_index_enum uart_n, const uint8 dat);
void    uart_write_byte                     (uart_index_enum uartn, const uint8 dat);
void    uart_write_buffer                   (uart_index_enum uartn, const uint8 *buff, uint32 len);
void    uart_write_string                   (uart_index_enum uartn, const char *str);

uint8   uart_read_byte                      (uart_index_enum uartn);
uint8   uart_query_byte                     (uart_index_enum uartn, uint8 *dat);

void    uart_tx_interrupt                   (uart_index_enum uartn, uint32 status);
void    uart_rx_interrupt                   (uart_index_enum uartn, uint32 status);

void    uart_init                           (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
//====================================================串口 基础函数====================================================

//=================================================兼容旧版本开源库接口名称=================================================
#ifdef  COMPATIBLE_WITH_OLDER_VERSIONS
#define uart_putchar(uart_n, dat)           (uart_write_byte((uart_n), (dat)))
#define uart_putbuff(uart_n, buff, len)     (uart_write_buffer((uart_n), (buff), (len)))
#define uart_putstr(uart_n, str)            (uart_write_string((uart_n), (str)))

#define uart_getchar(uart_n, dat)           (*(dat) = uart_read_byte((uart_n)))
#define uart_query(uart_n, dat)             (uart_query_byte((uart_n), (dat)))

#define uart_tx_irq(uart_n, status)         (uart_tx_interrupt((uart_n), (status)))
#define uart_rx_irq(uart_n, status)         (uart_rx_interrupt((uart_n), (status)))
#endif
//=================================================兼容旧版本开源库接口名称=================================================

#endif
