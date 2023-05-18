/*
 * uart.h
 *
 *  Created on: 2023��3��18��
 *      Author: wzl
 */
#ifndef CODE_UART_UART_H_
#define CODE_UART_UART_H_


//����ͷ�ļ�
#include <stdarg.h>
#include "cpu0_main.h"

//�궨��������������
#define uart_booluteeth_pin_tx      14      //UART2_TX_P10_5
#define uart_booluteeth_pin_rx      10      //UART2_RX_P10_6
//�궨������ʹ�õĴ���
#define bluetooth_using_uart        2       //UART_2
//�궨������ʹ�ô��ڵĲ�����
#define bluetooth_using_uart_baud   115200

//�궨��ebʹ�õĴ���
#define eb_using_uart               2       //UART_2
//�궨��ebʹ�ô��ڵĲ�����
#define eb_using_uart_baud          115200
//�궨��������������
#define uart_eb_pin_tx              14      //UART2_TX_P10_5
#define uart_eb_pin_rx              10      //UART2_RX_P10_6


//�궨�崮����Ϣ���շ�ʽ,1Ϊʹ��������0Ϊʹ����λ����������
#define USING_BLUETOOTH_OR_EGBOARD     0

//����ͨ�������Ϣ
#if USING_BLUETOOTH_OR_EGBOARD
    //����֡ͷ
    #define DATA_PEAK_HEAD              0xa5
    //����֡β
    #define DATA_PEAK_TAIL              0x5a
    //ͨ�������ֽڳ���
    #define UART_FRAME_LEN              24

    //���������������ݽṹ��
    typedef struct
    {
        bool  receiveStart;                                     //���ݽ��տ�ʼ
        uint8 receiveIndex;                                     //��������
        bool  receiveFinished;                                  //���ݶ��н��ղ�У�����
        bool  copyready;                                        //���ݿ������
        uint8 data_verity;                                      //����У��λ
        uint8 receiveBuff[UART_FRAME_LEN];                      //���ݽ��ն���
        uint8 receiveBuffFinished[UART_FRAME_LEN];              //���ݽ��ն��У�У��ɹ�

        bool  data_choice;                                      //����->ѡ�񿪻����Ǳջ�
        float data_Kp;                                          //����kp
        float data_Ki;                                          //����ki
        float data_Kd;                                          //����kd
        float data_speed;                                       //����->�ٶ�
        float data_angle;                                       //����->�Ƕ�
    }BlueTooth_data_recevie_Struct;

    //�����������ݽṹ��
    extern BlueTooth_data_recevie_Struct Bluetooth_data;
#else
    //ͨ�������ֽ������
    #define USB_FRAME_LENMAX            16
    //USBͨ�������ֽ���̳���
    #define USB_FRAME_LENMIN            4
    //ebͨ������֡ͷ
    #define USB_FRAME_HEAD              0x42
    //ͨ�ŵ�ַ
    #define USB_ADDR_HEART              0x00                //����������
    #define USB_ADDR_CONTROL            0x01                //���ܳ�����
    #define USB_ADDR_SPEEDMODE          0x02                //�ٿ�ģʽ
    #define USB_ADDR_SERVOTHRESHOLD     0x03                //�����ֵ
    #define USB_ADDR_BUZZER             0x04                //��������Ч
    #define USB_ADDR_LIGHT              0x05                //LED��Ч
    #define USB_ADDR_KEYINPUT           0x06                //��������
    #define USB_ADDR_BATTERY            0x07                //�����Ϣ
    #define USB_ADDR_PID_INFORMATION    0x10                //��λ��PID������Ϣ
    //�����Լ����ͨ��ʹ��
    #define USB_ADDR_INSPECTOR          0x0A                //���ܳ��Լ������������
    #define USB_ADDR_SELFCHECK          0x0B                //���ܳ��Լ쿪ʼ
    #define USB_ADDR_SPEEDBACK          0x08                //������Ϣ������m/s

    //ebͨ�Žṹ��
    typedef struct
    {
        bool   receiveStart;                                     //���ݽ��տ�ʼ
        uint8  receiveIndex;                                     //��������
        bool   receiveFinished;                                  //���ݶ��н��ղ�У�����
        uint8  receiveBuff[USB_FRAME_LENMAX];                    //USB���ն��У���ʱ����
        uint8  receiveBuffFinished[USB_FRAME_LENMAX];            //USB���ն��У�У��ɹ�
        uint16 counter;                                          //������
        uint16 counterDrop;                                      //���߼�����

        uint8  data_verity;                                      //����У��

        uint16 counterSend;                                      //�Լ����ݷ��ͼ�����
        bool   connected;                                        //��λ��ͨ������״̬
        bool   inspectorEnable;                                  //���������Լ��������ʹ��
    }UsbStruct;

    //����ebͨ�����ݽṹ��
    extern UsbStruct usbStr;
#endif


//==================================================UART ��������==================================================
void my_uart_init                   (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);

#if USING_BLUETOOTH_OR_EGBOARD
void use_bluetooth_Handle           (void);
void Bluetooth_Send                 (char *string, ...);
#endif


#if !USING_BLUETOOTH_OR_EGBOARD
void USB_Edgeboard_Timr             (void);
void USB_Edgeboard_TransmitKey      (uint16 time);
void USB_Edgeboard_Handle           (void);

//���ܳ��Լ�
void USB_Edgeboard_ServoThreshold   (uint8 chanel);
void USB_Edgeboard_CarSpeed         (void);
#endif

void senddata_to_upper              (float senddata);
//==================================================UART ��������==================================================


#endif /* CODE_UART_UART_H_ */
