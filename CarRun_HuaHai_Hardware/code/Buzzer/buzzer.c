/*
 * buzzer.c
 *
 *  Created on: 2023��4��27��
 *      Author: wzl
 */

//����ͷ�ļ�
#include "Buzzer/buzzer.h"

//�����ṹ��
BuzzerStruct buzzerStr;

/**
* @brief        ��������ʼ��
* @param        void
* @ref          void
* @author       wzl
**/
void Buzzer_Init(void)
{
    //��ʼ����������gpio�ڣ���ʼʱΪ�͵�ƽ
    gpio_init(P14_6, GPO, 0, GPO_PUSH_PULL);

    //��ؽṹ�������ʼ��
    buzzerStr.Counter = 0;
    buzzerStr.Cut = 0;
    buzzerStr.Enable = false;
    buzzerStr.Times = 0;
}

/**
* @brief        ���Ʒ������߳�
* @param        void
* @ref          void
* @author       wzl
**/
void Buzzer_Timer(void)
{
    //���յ������������ź�
    if(buzzerStr.Enable)
    {
        //����������
        buzzerStr.Counter++;
        //
        if(buzzerStr.Cut < buzzerStr.Counter)
            buzzerStr.Counter = buzzerStr.Cut;
    }
}

/**
* @brief        ���������ƺ���
* @param        void
* @ref          void
* @author       wzl
**/
void Buzzer_Handle(void)
{
    //���������ƣ�������������û��ֹͣ������
    if(buzzerStr.Enable && !buzzerStr.Silent)
    {
        if(buzzerStr.Times<=0)
        {
            //��û�����д���ʱ�����͵�ƽ
            gpio_low(P14_6);
            //ʧ�ܷ�����
            buzzerStr.Enable = false;
            return;
        }
        else if(buzzerStr.Cut<=buzzerStr.Counter)
        {
            //��ת���������ŵ������ƽ
            gpio_toggle_level(P14_6);
            //��ת����
            buzzerStr.Times--;
            //��תʱ��
            buzzerStr.Counter = 0;
        }
    }
    else
        //ֹͣ����������
        gpio_low(P14_6);
}


/**
* @brief        ������ʹ��
* @param        buzzer������������ģʽѡ��
* @ref          void
* @author       wzl
**/
void Buzzer_Enable(BuzzerEnum buzzer)
{
    //ѡ�����������ģʽ
    switch(buzzer)
    {
        case BuzzerOk:
            buzzerStr.Cut = 70;         //70ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 4;
            break;

        case BuzzerWarnning:
            buzzerStr.Cut = 100;        //100ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 10;
            break;

        case BuzzerSysStart:
            buzzerStr.Cut = 60;         //60ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 6;
            break;

        case BuzzerDing:
            buzzerStr.Cut = 30;         //30ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 2;
            break;

        case BuzzerFinish:
            buzzerStr.Cut = 200;        //200ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 6;
            break;

        case BuzzerVoltage:
            buzzerStr.Cut = 15;         //15ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 10;
            break;
    }

    //��ռ�����
    buzzerStr.Counter = 0;
}

