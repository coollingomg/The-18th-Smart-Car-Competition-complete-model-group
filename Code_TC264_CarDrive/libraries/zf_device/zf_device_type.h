#ifndef _zf_device_type_h_
#define _zf_device_type_h_

#include "zf_common_debug.h"

//==============================================定义 外设 枚举体==================================================
typedef enum
{
    NO_CAMERE = 0,                                                              // 无摄像头
    CAMERA_BIN_IIC,                                                             // 小钻风 IIC 版本
    CAMERA_BIN_UART,                                                            // 小钻风 UART 版本
    CAMERA_GRAYSCALE,                                                           // 总钻风
    CAMERA_COLOR,                                                               // 凌瞳
}camera_type_enum;

typedef enum
{
    NO_WIRELESS = 0,                                                            // 无设备
    WIRELESS_UART,                                                              // 无线串口
    BLUETOOTH_CH9141,                                                           // 蓝牙 CH9141
    GPS_TAU1201,
    WIFI_UART,
    WIFI_SPI,
}wireless_type_enum;
//==============================================定义 外设 枚举体==================================================

typedef void (*callback_function)(void);

//===========================================声明回调函数指针及外设类型==================================================
extern wireless_type_enum wireless_type;
extern callback_function wireless_module_uart_handler;                          // 无线串口接收中断函数指针，根据初始化时设置的函数进行跳转

extern camera_type_enum camera_type;
extern callback_function camera_dma_handler;                                    // 串口通讯中断函数指针，根据初始化时设置的函数进行跳转
extern callback_function camera_vsync_handler;                                  // 串口通讯中断函数指针，根据初始化时设置的函数进行跳转
extern callback_function camera_uart_handler;                                   // 串口通讯中断函数指针，根据初始化时设置的函数进行跳转
//===========================================声明回调函数指针及外设类型==================================================

//=============================================中断回调 基础函数===================================================
void   set_camera_type          (camera_type_enum type_set, callback_function vsync_callback, callback_function dma_callback, callback_function uart_callback);
void   set_wireless_type        (wireless_type_enum type_set, callback_function uart_callback);
//=============================================中断回调 基础函数===================================================

#endif
