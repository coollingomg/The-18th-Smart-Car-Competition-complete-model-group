/*
 * buzzer.c
 *
 *  Created on: 2023年4月27日
 *      Author: wzl
 */

//包含头文件
#include "Buzzer/buzzer.h"

//声明结构体
BuzzerStruct buzzerStr;

/**
* @brief        蜂鸣器初始化
* @param        void
* @ref          void
* @author       wzl
**/
void Buzzer_Init(void)
{
    //初始化蜂鸣器的gpio口，初始时为低电平
    gpio_init(P14_6, GPO, 0, GPO_PUSH_PULL);

    //相关结构体参数初始化
    buzzerStr.Counter = 0;
    buzzerStr.Cut = 0;
    buzzerStr.Enable = false;
    buzzerStr.Times = 0;
}

/**
* @brief        控制蜂鸣器线程
* @param        void
* @ref          void
* @author       wzl
**/
void Buzzer_Timer(void)
{
    //接收到开启蜂鸣器信号
    if(buzzerStr.Enable)
    {
        //蜂鸣器计数
        buzzerStr.Counter++;
        //
        if(buzzerStr.Cut < buzzerStr.Counter)
            buzzerStr.Counter = buzzerStr.Cut;
    }
}

/**
* @brief        蜂鸣器控制函数
* @param        void
* @ref          void
* @author       wzl
**/
void Buzzer_Handle(void)
{
    //蜂鸣器控制，蜂鸣器开启且没有停止蜂鸣器
    if(buzzerStr.Enable && !buzzerStr.Silent)
    {
        if(buzzerStr.Times<=0)
        {
            //当没有鸣叫次数时，拉低电平
            gpio_low(P14_6);
            //失能蜂鸣器
            buzzerStr.Enable = false;
            return;
        }
        else if(buzzerStr.Cut<=buzzerStr.Counter)
        {
            //翻转蜂鸣器引脚的输出电平
            gpio_toggle_level(P14_6);
            //翻转次数
            buzzerStr.Times--;
            //翻转时长
            buzzerStr.Counter = 0;
        }
    }
    else
        //停止蜂鸣器鸣叫
        gpio_low(P14_6);
}


/**
* @brief        蜂鸣器使能
* @param        buzzer：蜂鸣器工作模式选择
* @ref          void
* @author       wzl
**/
void Buzzer_Enable(BuzzerEnum buzzer)
{
    //选择蜂鸣器工作模式
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

    //清空计数器
    buzzerStr.Counter = 0;
}

