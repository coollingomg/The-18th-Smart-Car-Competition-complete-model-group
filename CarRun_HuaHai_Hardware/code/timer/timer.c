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
#include "Buzzer/buzzer.h"
#include "servo/servo.h"
#include "voltage_sampling/voltage_sampling.h"
#include "icm20602_data_handle.h"

//-------------------------------------------------------------------------------------------------------------------
// �������       ��ʱ�жϳ�ʼ��
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void timer_Init(void)
{
    //������ʱ�������ж�,ʹ��CCU61_CH0 ����ʱ1ms
    pit_ms_init(CCU61_CH0, 1);
//    //������ʱ�������ж�,ʹ��CCU60_CH1 ����ʱ1ms
//    pit_ms_init(CCU60_CH1, 1);
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ��ʱ�жϷ�����
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void timer_interrupt(void)
{
    //��������߳�
    motor_ControlLoop();
    //ebͨѶ���߼��
    USB_Edgeboard_Timr();
//    //���߷����߳�
//    Wireless_Timer();
    //�����������߳�
    Buzzer_Timer();
    //���ܳ��ۺϴ����̼߳�����
    ICAR_Timer();
    //��ѹ�����߳�
    adc_Timer();
//    //��̬�ǽ����߳�
//    icm20602_attitude_Angle_Timer();
}


////-------------------------------------------------------------------------------------------------------------------
//// �������       ��ʱ���жϣ����������ǽ���
//// ����˵��       void
//// ���ز���       void
////-------------------------------------------------------------------------------------------------------------------
//void timer_interrupt0(void)
//{
//
//}


IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    //�����ж�Ƕ��
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
    //�жϷ�����
    timer_interrupt();
}

//IFX_INTERRUPT(cc60_pit_ch1_isr, 1, CCU6_0_CH1_ISR_PRIORITY)
//{
//    // �����ж�Ƕ��
//    interrupt_global_enable(0);
//    pit_clear_flag(CCU60_CH1);
//    //�жϷ�����
//    timer_interrupt0();
//}


