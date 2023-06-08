#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

//����ͷ�ļ�
#include "voltage_sampling/voltage_sampling.h"
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


//-----------------------------------�˴���д�û����� ���������ʼ�������-----------------------------------

    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

//ѡ��ͨ��ģʽ
#if USING_BLUETOOTH_OR_EGBOARD
//        //���ת�����
//        servo_contral(Bluetooth_data.data_angle);
#else
#endif

//        //���ת������̴߳�����
//        SERVO_Handle();
//        //��������̬�ǽ������
//        icm20602_attitude_Angle_pose(&icm20602_pose, 0.001);
        //��ѹ��������
        adc_Handle();

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

    }
}

#pragma section all restore
//----------------------------------��������----------------------------------
