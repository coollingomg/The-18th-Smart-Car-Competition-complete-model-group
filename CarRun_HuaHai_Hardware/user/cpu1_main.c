#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"



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
#include "Kalman/Kalman_Filter.h"


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
    //ƫ���Ǽ��ٶ��˲���������������ʼ��
    Kalman_Filter_Init(&kalman_struck1);

//-----------------------------------�˴���д�û����� ���������ʼ�������-----------------------------------

    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

        //��ѹ��������
        adc_Handle();
        //���ݴ���
        USB_Edgeboard_Handle();
        //��̬�ǽ��㴦����
        icm20602_attitude_Angle_handle();

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

    }
}

#pragma section all restore
//----------------------------------��������----------------------------------
