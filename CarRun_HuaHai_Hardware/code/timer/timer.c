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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       定时中断初始化
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void timer_Init(void)
{
    //开启定时器计数中断,使用CCU61_CH0 ，定时1ms
    pit_ms_init(CCU61_CH0, 1);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       定时中断服务函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void timer_interrupt(void)
{
    //电机控制线程
#if USING_BLUETOOTH_OR_EGBOARD
    motor_ControlLoop(Bluetooth_data.data_speed);
#else
    motor_ControlLoop();
#endif


    //eb通讯掉线检测
    #if !USING_BLUETOOTH_OR_EGBOARD
        USB_Edgeboard_Timr();
    #endif

    //智能车综合处理线程计数器
    ICAR_Timer();
}


IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    //开启中断嵌套
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
    //中断服务函数
    timer_interrupt();
}
