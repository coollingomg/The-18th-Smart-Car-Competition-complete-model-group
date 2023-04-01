#ifndef _zf_driver_exti_h_
#define _zf_driver_exti_h_

#include "IfxScuEru.h"
#include "zf_common_typedef.h"

typedef enum  // 枚举ERU通道
{
    // 一个通道只能选择其中一个引脚作为 外部中断的输入
    ERU_CH0_REQ0_P15_4   = 0*3 + 1,                                                 // 通道0可选引脚
    ERU_CH1_REQ10_P14_3  = 1*3 + 1,                                                 // 通道1可选引脚
    // 特别注意通道2 与 通道3都被摄像头占用
    // 特别注意通道2 与 通道3都被摄像头占用
    // 特别注意通道2 与 通道3都被摄像头占用
    ERU_CH2_REQ7_P00_4   = 2*3,  ERU_CH2_REQ14_P02_1, ERU_CH2_REQ2_P10_2,           // 通道2可选引脚
    ERU_CH3_REQ6_P02_0   = 3*3,  ERU_CH3_REQ3_P10_3,  ERU_CH3_REQ15_P14_1,          // 通道3可选引脚

    // 通道4与通道0 共用中断函数 在中断内通过判断标志位来识别是哪个通道触发的中断
    ERU_CH4_REQ13_P15_5  = 4*3,  ERU_CH4_REQ8_P33_7,                                // 通道4可选引脚
    // 通道5与通道1 共用中断函数
    ERU_CH5_REQ1_P15_8   = 5*3,                                                     // 通道5可选引脚
    // 通道6与通道2 共用中断函数
    ERU_CH6_REQ12_P11_10 = 6*3,  ERU_CH6_REQ9_P20_0,                                // 通道6可选引脚
    // 通道7与通道3 共用中断函数
    ERU_CH7_REQ16_P15_1  = 7*3,  ERU_CH7_REQ11_P20_9,                               // 通道7可选引脚
}exti_pin_enum;



typedef enum  // 枚举触发方式
{
    EXTI_TRIGGER_RISING,                                                            // 上升沿触发模式
    EXTI_TRIGGER_FALLING,                                                           // 下降沿触发模式
    EXTI_TRIGGER_BOTH,                                                              // 双边沿触发模式
}exti_trigger_enum;



// 中断标志位获取
#define exti_flag_get(eru_pin)      IfxScuEru_getEventFlagStatus((IfxScuEru_InputChannel)(eru_pin/3))
// 中断标志位清除
#define exti_flag_clear(eru_pin)    IfxScuEru_clearEventFlag((IfxScuEru_InputChannel)(eru_pin/3))

//====================================================EXIT 基础函数====================================================
void exti_all_close         (void);                                                 // EXTI 失能
void exti_enable            (exti_pin_enum eru_pin);                                // EXTI 中断使能
void exti_disable           (exti_pin_enum eru_pin);                                // EXTI 中断失能
void exti_init              (exti_pin_enum eru_pin, exti_trigger_enum trigger);     // EXTI 中断初始化
//====================================================EXIT 基础函数====================================================


#endif
