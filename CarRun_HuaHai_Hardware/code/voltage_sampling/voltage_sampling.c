/*
 * voltage_sampling.c
 *
 *  Created on: 2023年5月15日
 *      Author: wzl
 */
#include "voltage_sampling/voltage_sampling.h"
#include "Buzzer/buzzer.h"

//定义adc采样的结构体
ADC_sampling adc_sampling;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       adc初始化
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void adc_Init(void)
{
    //电压采样初始化
    adc_init(ADC0_CH2_A2, ADC_12BIT);

    //相关参数初始化
    adc_sampling.if_adc_handle = false;        //初始默认失能
    adc_sampling.adc_getNum = 0.0;             //初始采样值为0
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       adc线程计数器函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void adc_Timer(void)
{
    //没进一次该函数，计数器自增
    adc_sampling.count++;

    //每10000ms处理一次adc采样数据
    if(adc_sampling.count >= SAMPL_TIME)
    {
        //使能adc采样处理标志位
        adc_sampling.if_adc_handle = true;
        //清零计数器
        adc_sampling.count = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       adc线程处理
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void adc_Handle(void)
{
    if(adc_sampling.if_adc_handle == true)
    {
        //定义变量来读取adc寄存器的值
        uint16 get_value = 0;
        //开始进行电压采样，采样5此，返回采样平均值
        get_value = adc_mean_filter_convert(ADC0_CH2_A2, 5);
        //将得到的值转换为电压值
        adc_sampling.adc_getNum = get_value / CONVERSION_FAC * MAX_VOLTAGE;
        //判断电压是否低于设置电压
        if(adc_sampling.adc_getNum < VOLTAGE_VALUE)
        {
            Buzzer_Enable(BuzzerVoltage);
        }
        //清理标志位
        adc_sampling.if_adc_handle = false;
    }
}

