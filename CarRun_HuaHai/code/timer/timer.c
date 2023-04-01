/*
 * timer.c
 *
 *  Created on: 2023��3��28��
 *      Author: wzl
 */
#include "timer/timer.h"
#include "motor/motor.h"
#include "isr_config.h"
#include "car_control/car_control.h"
#include "uart/uart.h"

//-------------------------------------------------------------------------------------------------------------------
// �������       ��ʱ�жϳ�ʼ��
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void timer_Init(void)
{
    //������ʱ�������ж�,ʹ��CCU61_CH0 ����ʱ1ms
    pit_ms_init(CCU61_CH0, 1);
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ��ʱ�жϷ�����
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void timer_interrupt(void)
{
    //��������߳�
#if USING_BLUETOOTH_OR_EGBOARD
    motor_ControlLoop(Bluetooth_data.data_speed);
#else
    motor_ControlLoop();
#endif


    //ebͨѶ���߼��
    #if !USING_BLUETOOTH_OR_EGBOARD
        USB_Edgeboard_Timr();
    #endif

    //���ܳ��ۺϴ����̼߳�����
    ICAR_Timer();
}


IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    //�����ж�Ƕ��
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
    //�жϷ�����
    timer_interrupt();
}
