#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��
// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��


//����ͷ�ļ�
#include "icm20602_data_handle.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "timer/timer.h"
#include "key/key.h"
#include "my_flash/my_flash.h"
#include "Buzzer/buzzer.h"
#include "car_control/car_control.h"
#include "pid.h"
#include "uart.h"


//----------------------------------��������----------------------------------
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���

//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------
    //�����������ʼ��
    //icm20602_pose_init();
    //����flash���Ĵ洢����
    my_flash_init();
    //�ȳ�ʼ�������������ж��п��ƶ��������bug
    servo_init();
    //�����ʼ��
    motor_init();
    //��������ʼ��
    Buzzer_Init();

//ѡ��ͨ�ų�ʼ��
#if USING_BLUETOOTH_OR_EGBOARD
    //ʹ������ͨ��
    my_uart_init(bluetooth_using_uart, bluetooth_using_uart_baud, uart_booluteeth_pin_tx, uart_booluteeth_pin_rx);
#else
    //��ȡpid����
    float pid_data[3] = {icarStr.data_Kp, icarStr.data_Ki, icarStr.data_Kd};
    //pid��ʼ��
    PID_Init(&car_speed_pid, PID_POSITION, pid_data, CAR_MAX_SPEED,CAR_IMAX_OUT);
    //��ebͨ�ų�ʼ��
    my_uart_init(eb_using_uart, eb_using_uart_baud, uart_eb_pin_tx, uart_eb_pin_rx);
    //�ȴ����ݽ������
    while(!usbStr.receiveFinished);
    //��ƣ�˵�����ݽ������
    gpio_init(P20_9, GPO, 0, GPO_PUSH_PULL);
#endif

    //�̳߳�ʼ��
    timer_Init();
    //���ܳ����Ʋ�����ʼ��
    ICAR_Init();
    //������ʼ��
    my_key_init();
    //��ʼ����ɣ���������ʾ��
    Buzzer_Enable(BuzzerSysStart);
//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------
#if USING_BLUETOOTH_OR_EGBOARD
        //�������ڷ�������
        Bluetooth_Send("%f,%f,%f",Bluetooth_data.data_angle, Bluetooth_data.data_speed, icarStr.SpeedFeedback);
        //�������ݴ���
        use_bluetooth_Handle();
#else
        //���ݴ���
        USB_Edgeboard_Handle();
        //flash������
        my_flash_Handle();
#endif
        //����������
        Buzzer_Handle();
//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

    }
}

#pragma section all restore
//----------------------------------��������----------------------------------

