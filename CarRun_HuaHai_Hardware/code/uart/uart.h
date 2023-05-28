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

//宏定义蓝牙串口引脚
#define uart_booluteeth_pin_tx      14      //UART2_TX_P10_5
#define uart_booluteeth_pin_rx      10      //UART2_RX_P10_6
//宏定义蓝牙使用的串口
#define bluetooth_using_uart        2       //UART_2
//宏定义蓝牙使用串口的波特率
#define bluetooth_using_uart_baud   115200

//宏定义eb使用的串口
#define eb_using_uart               2       //UART_2
//宏定义eb使用串口的波特率
#define eb_using_uart_baud          460800
//宏定义蓝牙串口引脚
#define uart_eb_pin_tx              14      //UART2_TX_P10_5
#define uart_eb_pin_rx              10      //UART2_RX_P10_6


//宏定义串口信息接收方式,1为使用蓝牙；0为使用上位机接收数据
#define USING_BLUETOOTH_OR_EGBOARD     0

//定义通信相关信息
#if USING_BLUETOOTH_OR_EGBOARD
    //定义帧头
    #define DATA_PEAK_HEAD              0xa5
    //定义帧尾
    #define DATA_PEAK_TAIL              0x5a
    //通信序列字节长度
    #define UART_FRAME_LEN              24

    //串口蓝牙调试数据结构体
    typedef struct
    {
        bool  receiveStart;                                     //数据接收开始
        uint8 receiveIndex;                                     //接收序列
        bool  receiveFinished;                                  //数据队列接收并校验完成
        bool  copyready;                                        //数据拷贝完成
        uint8 data_verity;                                      //数据校验位
        uint8 receiveBuff[UART_FRAME_LEN];                      //数据接收队列
        uint8 receiveBuffFinished[UART_FRAME_LEN];              //数据接收队列：校验成功

        bool  data_choice;                                      //数据->选择开环还是闭环
        float data_Kp;                                          //数据kp
        float data_Ki;                                          //数据ki
        float data_Kd;                                          //数据kd
        float data_speed;                                       //数据->速度
        float data_angle;                                       //数据->角度
    }BlueTooth_data_recevie_Struct;

    //声明蓝牙数据结构体
    extern BlueTooth_data_recevie_Struct Bluetooth_data;
#else
    //通信序列字节最长长度
    #define USB_FRAME_LENMAX            16
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
#endif


//==================================================UART 基础函数==================================================
void my_uart_init                   (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);

#if USING_BLUETOOTH_OR_EGBOARD
void use_bluetooth_Handle           (void);
void Bluetooth_Send                 (char *string, ...);
#endif


#if !USING_BLUETOOTH_OR_EGBOARD
void USB_Edgeboard_Timr             (void);
void USB_Edgeboard_TransmitKey      (uint16 time);
void USB_Edgeboard_Handle           (void);

//智能车自检
void USB_Edgeboard_ServoThreshold   (uint8 chanel);
void USB_Edgeboard_CarSpeed         (void);
#endif

void senddata_to_upper              (uint8_t addr,float senddata);
//==================================================UART 基础函数==================================================


#endif /* CODE_UART_UART_H_ */
