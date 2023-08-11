#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"


//����ͷ�ļ�
#include "icm20602_data_handle.h"
#include "my_flash/my_flash.h"
#include "Buzzer/buzzer.h"
#include "Kalman_Filter.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "timer/timer.h"
#include "car_control.h"
#include "INA226.h"
#include "uart.h"


//----------------------------------��������----------------------------------
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���

//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    //����flash���Ĵ洢����
    my_flash_init();
    //�ȳ�ʼ�������������ж��п��ƶ��������bug
    servo_init();
    //�����ʼ��
    motor_init();
    //��ʼ����������
    INA226_Init();
    //��ʱ����ʼ��
    timer_Init();
    //��������ʼ��
    Buzzer_Init();
    //�����������ʼ��
    icm20602_pose_init();
    //ƫ�����˲���������������ʼ��
    Kalman_Filter_Init(&kalman_imu_yaw);
    //���ߴ��ڻ��������Խӿڳ�ʼ��
    Wireless_uart_init(wireless_using_uart, wireless_using_uart_baud, uart_wireless_pin_tx, uart_wireless_pin_rx);
//    Bluetooth_uart_init(bluetooth_using_uart, bluetooth_using_uart_baud, uart_booluteeth_pin_tx, uart_booluteeth_pin_rx);
    //ͨ��������ʾ�Ƴ�ʼ��
    gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL);
    //��ʼ����ɺ����ߵ�ƽ���ѵ��������
    gpio_init(P21_3, GPO, 1, GPO_PUSH_PULL);
    //��ʼ�����ţ�������ʾ���ݽ����Ƿ����
    gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL);
    //������������ʼ������������ٶ��˲�
    Kalman_Filter_Init(&kalman_motor_speedback);
    //���ܳ����Ʋ�����ʼ��
    ICAR_Init();
    //��ʼ����ɣ���������ʾ��
    Buzzer_Enable(BuzzerSysStart);

//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

        //flash������
        my_flash_Handle();
        //����������
        Buzzer_Handle();
        //���ܳ�����
        ICAR_Handle();
        //���ߴ��ڻ������������ݴ�����
        Wireless_Handle();
//        //�����������ݴ�����
//        use_bluetooth_Handle();
        //��̬�ǽ��㴦����
        icm20602_attitude_Angle_handle();

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

    }
}

#pragma section all restore
