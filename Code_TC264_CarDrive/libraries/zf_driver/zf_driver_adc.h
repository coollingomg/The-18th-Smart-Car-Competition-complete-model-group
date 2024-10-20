#ifndef _zf_driver_adc_h
#define _zf_driver_adc_h

#include "zf_common_typedef.h"

typedef enum    // 枚举ADC通道
{
    // ADC0可选引脚
    ADC0_CH0_A0   = 0*16 + 0,
    ADC0_CH1_A1,
    ADC0_CH2_A2,
    ADC0_CH3_A3,
    ADC0_CH4_A4,
    ADC0_CH5_A5,
    ADC0_CH6_A6,
    ADC0_CH7_A7,
    ADC0_CH8_A8,
    ADC0_CH10_A10 = 0*16 + 10,
    ADC0_CH11_A11,
    ADC0_CH12_A12,
    ADC0_CH13_A13,
	
	// ADC1可选引脚
	ADC1_CH0_A16  = 1*16 + 0,
	ADC1_CH1_A17  = 1*16 + 1,
	ADC1_CH4_A20  = 1*16 + 4,
	ADC1_CH5_A21  = 1*16 + 5,
	ADC1_CH8_A24  = 1*16 + 8,
	ADC1_CH9_A25  = 1*16 + 9,
	
	// ADC2可选引脚
	ADC2_CH3_A35  = 2*16 + 3,
	ADC2_CH4_A36,
	ADC2_CH5_A37,
	ADC2_CH6_A38,
	ADC2_CH7_A39,
	ADC2_CH10_A44 = 2*16 + 10,
	ADC2_CH11_A45,
	ADC2_CH12_A46,
	ADC2_CH13_A47,
	ADC2_CH14_A48,
	ADC2_CH15_A49,
}adc_channel_enum;

// 此枚举定义不允许用户修改
typedef enum        // 枚举ADC通道
{
    ADC_8BIT,       // 8位分辨率
    ADC_10BIT,      // 10位分辨率
    ADC_12BIT,      // 12位分辨率
}adc_resolution_enum;

//====================================================ADC 基础函数====================================================
uint16  adc_convert             (adc_channel_enum vadc_chn);                                    // ADC转换一次
uint16  adc_mean_filter_convert (adc_channel_enum vadc_chn, uint8 count);                       // ADC均值滤波
void    adc_init                (adc_channel_enum vadc_chn, adc_resolution_enum resolution);    // ADC初始化
//====================================================ADC 基础函数====================================================


#endif
