/*
 * buzzer.h
 *
 *  Created on: 2023年4月27日
 *      Author: wzl
 */

#ifndef CODE_BUZZER_BUZZER_H_
#define CODE_BUZZER_BUZZER_H_

//包含头文件
#include "cpu0_main.h"

//定义蜂鸣器结构体
typedef struct
{
    bool Enable;                        //使能标志
    uint16_t Times;                     //鸣叫次数
    uint16_t Counter;                   //计数器
    uint16_t Cut;                       //间隔时间
    bool Silent;                        //是否禁用蜂鸣器
}BuzzerStruct;


//蜂鸣器音效
typedef enum
{
    BuzzerOk = 0,                       //确认提示音
    BuzzerWarnning,                     //报警提示音
    BuzzerSysStart,                     //开机提示音
    BuzzerDing,                         //叮叮叮一下
    BuzzerFinish,                       //结束提示音
}BuzzerEnum;


//声明蜂鸣器相关函数
void Buzzer_Init        (void);
void Buzzer_Timer       (void);
void Buzzer_Handle      (void);
void Buzzer_Enable      (BuzzerEnum buzzer);


//声明结构体
extern BuzzerStruct buzzerStr;


#endif /* CODE_BUZZER_BUZZER_H_ */
