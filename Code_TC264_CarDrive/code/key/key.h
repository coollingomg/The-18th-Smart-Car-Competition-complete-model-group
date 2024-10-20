/*
 * key.h
 *
 *  Created on: 2023年3月28日
 *      Author: wzl
 */

#ifndef CODE_KEY_KEY_H_
#define CODE_KEY_KEY_H_

//包含头文件
#include "cpu0_main.h"

//宏定义要初始化地按键
#define KEY_GPIO    484                 //p15_4

//定义按键结构体
typedef struct
{
    bool KeyPress;                      //按键输入
}GpioStruct;


//声明按键结构体
extern GpioStruct gpioStr;


//==================================================CAR_CONTROL 基础函数==================================================
void my_key_init            (void);
//==================================================CAR_CONTROL 基础函数==================================================

#endif
