/*
 * voltage_sampling.h
 *
 *  Created on: 2023年5月15日
 *      Author: wzl
 */


#ifndef CODE_VOLTAGE_SAMPLING_VOLTAGE_SAMPLING_H_
#define CODE_VOLTAGE_SAMPLING_VOLTAGE_SAMPLING_H_


#include "cpu0_main.h"
#include "zf_driver_adc.h"


//宏定义相关常量
#define VOLTAGE_VALUE   11.0
#define VOLTAGE_MIN     5.0
#define SAMPL_TIME      1000
#define CONVERSION_FAC  4095.0
#define MAX_VOLTAGE     3.3 * 4.0


//定义电压采样相关的结构体
typedef struct
{
    bool    if_adc_handle;     //adc处理标志位
    float   adc_getNum;        //adc采样值
    uint16  count;             //计数器
    bool    key_contral;       //按键控制电量提醒
}ADC_sampling;


//声明结构体
extern ADC_sampling adc_sampling;


//==================================================V_sampling 基础函数==================================================
void adc_Init       (void);
void adc_Timer      (void);
void adc_Handle     (void);
//==================================================V_sampling 基础函数==================================================

#endif
