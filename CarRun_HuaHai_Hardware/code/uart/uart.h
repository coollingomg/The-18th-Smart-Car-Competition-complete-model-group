/*
 * uart.h
 *
 *  Created on: 2023年3月18日
 *      Author: wzl
 */


#ifndef CODE_UART_UART_H_
#define CODE_UART_UART_H_


//包含头文件
#include <stdarg.h>
#include "cpu0_main.h"


//宏定义蓝牙串口、无线串口引脚（没有同时用无线串口和蓝牙）
#define uart_booluteeth_pin_tx      0       //UART0_TX_P14_0
#define uart_booluteeth_pin_rx      0       //UART0_RX_P14_1
#define uart_wireless_pin_tx        0       //UART0_TX_P14_0
#define uart_wireless_pin_rx        0       //UART0_RX_P14_1
//宏定义蓝牙、无线串口使用的串口（没有同时用无线串口和蓝牙，所以两个都是用的同一组串口，使用别的串口只需要改宏定义）
#define bluetooth_using_uart        0       //UART_0
#define wireless_using_uart         0       //UART_0
//宏定义蓝牙、无线串口使用串口的波特率
#define bluetooth_using_uart_baud   115200
#define wireless_using_uart_baud    115200


//宏定义与上位机串口通信引脚
#define uart_eb_pin_tx              14      //UART2_TX_P10_5
#define uart_eb_pin_rx              10      //UART2_RX_P10_6
//宏定义edgeboard使用的串口
#define eb_using_uart               2       //UART_2
//宏定义edgeboard使用串口的波特率
#define eb_using_uart_baud          460800


//定义帧头--蓝牙
#define DATA_PEAK_HEAD              0xa5
//定义帧尾--蓝牙
#define DATA_PEAK_TAIL              0x5a
//通信序列字节长度--蓝牙
#define UART_FRAME_LEN              36


//无线串口调试数据结构体
typedef struct
{
    bool  Flag_Wireless;                                    //数据发送定时标志位
}Wireless_data_recevie_Struct;


//串口--蓝牙调试数据结构体
typedef struct
{
    bool  receiveStart;                                     //数据接收开始
    uint8 receiveIndex;                                     //接收序列
    bool  receiveFinished;                                  //数据队列接收并校验完成
    bool  copyready;                                        //数据拷贝完成
    bool  Flag_Bluetooth;                                   //数据发送定时

    uint8 data_verity;                                      //数据校验位
    uint8 receiveBuff[UART_FRAME_LEN];                      //数据接收队列
    uint8 receiveBuffFinished[UART_FRAME_LEN];              //数据接收队列：校验成功

    bool  data_choice;                                      //开环、闭环标准位
    float motor_speed_loop_Kp;                              //电机速度环kp
    float motor_speed_loop_Ki;                              //电机速度环ki
    float motor_speed_loop_Kd;                              //电机速度环kd
    float motor_current_loop_Kp;                            //电机电流环kp
    float motor_current_loop_Ki;                            //电机电流环ki
    float motor_current_loop_Kd;                            //电机电流环kd
    float data_speed;                                       //数据->速度
    float data_angle;                                       //数据->角度
}BlueTooth_data_recevie_Struct;


//通信序列字节最长长度
#define USB_FRAME_LENMAX            32
//USB通信序列字节最短长度
#define USB_FRAME_LENMIN            4
//eb通信序列帧头
#define USB_FRAME_HEAD              0x42
//通信地址
#define USB_ADDR_HEART              0x00                //监测软件心跳
#define USB_ADDR_CONTROL            0x01                //智能车控制
#define USB_ADDR_SPEEDMODE          0x02                //速控模式
#define USB_ADDR_SERVOTHRESHOLD     0x03                //舵机阈值
#define USB_ADDR_BUZZER             0x04                //蜂鸣器音效
#define USB_ADDR_LIGHT              0x05                //LED灯效
#define USB_ADDR_KEYINPUT           0x06                //按键输入
#define USB_ADDR_BATTERY            0x07                //电池信息
#define USB_ADDR_PID_INFORMATION    0x10                //下位机PID参数信息
//仅限自检软件通信使用
#define USB_ADDR_INSPECTOR          0x0A                //智能车自检软件连接心跳
#define USB_ADDR_SELFCHECK          0x0B                //智能车自检开始
#define USB_ADDR_SPEEDBACK          0x08                //车速信息反馈：m/s


//eb通信结构体
typedef struct
{
    bool   receiveStart;                                     //数据接收开始
    uint8  receiveIndex;                                     //接收序列
    bool   receiveFinished;                                  //数据队列接收并校验完成
    uint8  receiveBuff[USB_FRAME_LENMAX];                    //USB接收队列：临时接收
    uint8  receiveBuffFinished[USB_FRAME_LENMAX];            //USB接收队列：校验成功
    uint16 counter;                                          //计数器
    uint16 counterDrop;                                      //掉线计数器
    uint16 counterSend_upper;                                //往上位机发送速度计时器

    uint8  data_verity;                                      //数据校验

    uint16 counterSend;                                      //自检数据发送计数器
    bool   connected;                                        //上位机通信连接状态
    bool   inspectorEnable;                                  //智能汽车自检软件连接使能
}UsbStruct;


//声明eb通信数据结构体
extern UsbStruct usbStr;
//声明蓝牙数据结构体
extern BlueTooth_data_recevie_Struct Bluetooth_data;
//声明无线串口数据结构体
extern Wireless_data_recevie_Struct Wireless_data;


//==================================================UART 基础函数==================================================
void USB_uart_init                  (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
void Wireless_uart_init             (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
void Bluetooth_uart_init            (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);

void use_bluetooth_Handle           (void);
void Bluetooth_Send                 (char *string, ...);

void Wireless_Uart_Send             (int32_t data1,int32_t data2,int32_t data3,int32_t data4);
void Wireless_Handle                (void);
void Wireless_Timer                 (void);

void USB_Edgeboard_Timr             (void);
void USB_Edgeboard_Handle           (void);
void USB_Edgeboard_TransmitKey      (uint16 time);

//智能车自检
void USB_Edgeboard_ServoThreshold   (uint8 chanel);
void USB_Edgeboard_CarSpeed         (void);
void senddata_to_upper              (uint8_t addr,float senddata);
//==================================================UART 基础函数==================================================


#endif
