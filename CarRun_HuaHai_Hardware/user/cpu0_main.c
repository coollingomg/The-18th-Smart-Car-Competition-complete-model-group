#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��
// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��


//����ͷ�ļ�
#include "Kalman/Kalman_Filter.h"
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

    //����flash���Ĵ洢����
    my_flash_init();
    //�ȳ�ʼ�������������ж��п��ƶ��������bug
    servo_init();
    //�����ʼ��
    motor_init();
     //�̳߳�ʼ��
    timer_Init();
    //��������ʼ��
    Buzzer_Init();
    //��ebͨ�ų�ʼ��
    USB_uart_init(eb_using_uart, eb_using_uart_baud, uart_eb_pin_tx, uart_eb_pin_rx);
    //ͨ��������ʾ�Ƴ�ʼ��
    gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL);
    //��ʼ����ɺ����ߵ�ƽ���ѵ��������
    gpio_init(P21_3, GPO, 1, GPO_PUSH_PULL);
    //��ʼ�����ţ�������ʾ���ݽ����Ƿ����
    gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL);
    //������������ʼ������������ٶ��˲�
    Kalman_Filter_Init(&kalman_struck);
    //���ܳ����Ʋ�����ʼ��
    ICAR_Init();
    //��ʼ����ɣ���������ʾ��
    Buzzer_Enable(BuzzerSysStart);

//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

        //���ݴ���
        USB_Edgeboard_Handle();
        //flash������
        my_flash_Handle();
        //����������
        Buzzer_Handle();
        //���ܳ�����
        ICAR_Handle();

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

    }
}

#pragma section all restore
//----------------------------------��������----------------------------------

