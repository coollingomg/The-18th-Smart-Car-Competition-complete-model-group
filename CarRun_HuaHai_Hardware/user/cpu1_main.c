#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

//����ͷ�ļ�
#include "icm20602_data_pose/icm20602_data_handle.h"
#include "voltage_sampling/voltage_sampling.h"
#include "car_control/car_control.h"
#include "icm20602_data_handle.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "key/key.h"
#include "uart.h"
#include "pid.h"


//-----------------------------------��������-----------------------------------
void core1_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�

//-----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    //�����������ʼ��
    icm20602_pose_init();
    //��ѹ������ʼ��
    adc_Init();
    //������ʼ��
    my_key_init();
    //���������ӿ�
    BLUETOOTH_uart_init(bluetooth_using_uart, bluetooth_using_uart_baud, uart_booluteeth_pin_tx, uart_booluteeth_pin_rx);

//-----------------------------------�˴���д�û����� ���������ʼ�������-----------------------------------

    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

        //�������ڷ������ݴ���
        Wireless_Handle();
//        //��̬�ǽ��㴦����
//        icm20602_attitude_Angle_handle();
        //��ѹ��������
        adc_Handle();
        //��̬�ǽ��㴦����
        icm20602_attitude_Angle_handle();

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

    }
}

#pragma section all restore
//----------------------------------��������----------------------------------
