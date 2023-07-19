/*
 * timer.c
 *
 *  Created on: 2023年3月28日
 *      Author: wzl
 */
#include "timer/timer.h"
#include "motor/motor.h"
#include "isr_config.h"
#include "car_control/car_control.h"
#include "uart/uart.h"
#include "Buzzer/buzzer.h"
#include "servo/servo.h"
#include "voltage_sampling/voltage_sampling.h"
#include "icm20602_data_handle.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       定时中断初始化
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void timer_Init(void)
{
    //开启定时器计数中断,使用CCU61_CH0 ，定时1ms
    pit_ms_init(CCU61_CH0, 1);
//    //开启定时器计数中断,使用CCU60_CH1 ，定时1ms
//    pit_ms_init(CCU60_CH1, 1);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       定时中断服务函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void timer_interrupt(void)
{
    //电机控制线程
    motor_ControlLoop();
    //eb通讯掉线检测
    USB_Edgeboard_Timr();
//    //无线发送线程
//    Wireless_Timer();
    //蜂鸣器外设线程
    Buzzer_Timer();
    //智能车综合处理线程计数器
    ICAR_Timer();
    //电压采样线程
    adc_Timer();
//    //姿态角解算线程
//    icm20602_attitude_Angle_Timer();
}


////-------------------------------------------------------------------------------------------------------------------
//// 函数简介       定时器中断，用于陀螺仪解算
//// 参数说明       void
//// 返回参数       void
////-------------------------------------------------------------------------------------------------------------------
//void timer_interrupt0(void)
//{
//
//}


IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    //开启中断嵌套
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
    //中断服务函数
    timer_interrupt();
}

//IFX_INTERRUPT(cc60_pit_ch1_isr, 1, CCU6_0_CH1_ISR_PRIORITY)
//{
//    // 开启中断嵌套
//    interrupt_global_enable(0);
//    pit_clear_flag(CCU60_CH1);
//    //中断服务函数
//    timer_interrupt0();
//}


