/*
 * my_flash.c
 *
 *  Created on: 2023��3��30��
 *      Author: wzl
 */


#include "car_control/car_control.h"
#include "my_flash/my_flash.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "uart/uart.h"
#include "pid/pid.h"


//����ȫ�ֱ������������Ƿ�������ݴ洢
bool flashSaveEnable = false;
bool flashPIDEnable = false;


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡFlash���ݣ���ʼ����ز���
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void my_flash_init(void)
{
#if USING_BLUETOOTH_OR_EGBOARD
    //��ȡ����0��һҳ���ݵ�������
    flash_read_page_to_buffer(0, 1);
    //����������ȡ������->�����������ֵ
    servoStr.thresholdMiddle = flash_union_buffer[0].uint16_type;
    //����������ȡ������->�����������ת��ֵ
    servoStr.thresholdLeft = flash_union_buffer[1].uint16_type;
    //����������ȡ������->�����������ת��ֵ
    servoStr.thresholdRight = flash_union_buffer[2].uint16_type;
#else
    //��ȡ����0��һҳ���ݵ�������
    flash_read_page_to_buffer(0, 1);
    //����������ȡ������->�����������ֵ
    servoStr.thresholdMiddle = flash_union_buffer[0].uint16_type;
    //����������ȡ������->�����������ת��ֵ
    servoStr.thresholdLeft = flash_union_buffer[1].uint16_type;
    //����������ȡ������->�����������ת��ֵ
    servoStr.thresholdRight = flash_union_buffer[2].uint16_type;

    //��ȡ����0��һҳ���ݵ�������
    flash_read_page_to_buffer(0, 0);
    //����������ȡ������->�ײ��˶��ٶȿ���pid����->speed_loop_Kp
    icarStr.speed_loop_Kp = flash_union_buffer[0].float_type;
    //����������ȡ������->�ײ��˶��ٶȿ���pid����->speed_loop_Ki
    icarStr.speed_loop_Ki = flash_union_buffer[1].float_type;
    //����������ȡ������->�ײ��˶��ٶȿ���pid����->speed_loop_Kd
    icarStr.speed_loop_Kd = flash_union_buffer[2].float_type;
    //����������ȡ������->�ײ��˶��ٶȿ���pid����->current_loop_Kp
    icarStr.current_loop_Kp = flash_union_buffer[3].float_type;
    //����������ȡ������->�ײ��˶��ٶȿ���pid����->current_loop_Ki
    icarStr.current_loop_Ki = flash_union_buffer[4].float_type;
    //����������ȡ������->�ײ��˶��ٶȿ���pid����->current_loop_Kd
    icarStr.current_loop_Kd = flash_union_buffer[5].float_type;
#endif
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ����Flash���ݣ���ʼ����ز���
// ����˵��     float* buf  Ҫд��pid�����������ַ
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void my_flash_write_pid(float* buf)
{
    //�������ݵ�д��->�ײ��˶��ٶȿ���pid����->speed_loop_Kp
    flash_union_buffer[0].float_type = buf[0];
    //�������ݵ�д��->�ײ��˶��ٶȿ���pid����->speed_loop_Ki
    flash_union_buffer[1].float_type = buf[1];
    //�������ݵ�д��->�ײ��˶��ٶȿ���pid����->speed_loop_Kd
    flash_union_buffer[2].float_type = buf[2];
    //�������ݵ�д��->�ײ��˶��ٶȿ���pid����->current_loop_Kp
    flash_union_buffer[3].float_type = buf[3];
    //�������ݵ�д��->�ײ��˶��ٶȿ���pid����->current_loop_Ki
    flash_union_buffer[4].float_type = buf[4];
    //�������ݵ�д��->�ײ��˶��ٶȿ���pid����->current_loop_Kd
    flash_union_buffer[5].float_type = buf[5];

    //д��ǰ�����������
    flash_erase_page(0, 0);
    //�ȴ�����д��
    while(flash_write_page_from_buffer(0, 0));
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ����Flash���ݣ���ʼ����ز���
// ����˵��     uint16* buf              pwmValue
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
static void my_flash_write_pwmValue(uint16* buf)
{
    //�������ݵ�д��->�����������ֵ
    flash_union_buffer[0].uint16_type = buf[0];
    //�������ݵ�д��->�����������ת��ֵ
    flash_union_buffer[1].uint16_type = buf[1];
    //�������ݵ�д��->�����������ת��ֵ
    flash_union_buffer[2].uint16_type = buf[2];

    //д��ǰ�����������
    flash_erase_page(0, 1);
    //�ȴ�����д��
    while(flash_write_page_from_buffer(0, 1));
}


//-------------------------------------------------------------------------------------------------------------------
// �������     Flash�����ݿ���
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void my_flash_Handle(void)
{
    // ������ݴ洢
    if(flashSaveEnable)
    {
        // ��ȡpwmValueֵ
        uint16 pwmValue_buff[3] = {servoStr.thresholdMiddle, servoStr.thresholdLeft, servoStr.thresholdRight};
        // д������pwmValue
        my_flash_write_pwmValue(pwmValue_buff);
        // д����ɣ��ر�д��״̬
        flashSaveEnable = false;
    }

    // PID���ݴ洢
    if(flashPIDEnable)
    {
        // ��ȡ������ƿ��Ʋ���
        float pidValue_buff[6] = {icarStr.speed_loop_Kp, icarStr.speed_loop_Ki, icarStr.speed_loop_Kd,
                                  icarStr.current_loop_Kp, icarStr.current_loop_Ki, icarStr.current_loop_Kd};

        // д������PID����
        my_flash_write_pid(pidValue_buff);

        // ��ʼ��speed_loop_pid����
        float speed_loop_pid_buff[3] = {icarStr.speed_loop_Kp, icarStr.speed_loop_Ki, icarStr.speed_loop_Kd};
        PID_Init(&car_speed_pid, PID_POSITION, speed_loop_pid_buff, SPEED_LOOP_MAX_OUT, SPEED_LOOP_P_MAX_OUT);
        // ��ʼ��current_loop_pid����
        float current_loop_pid_buff[3] = {icarStr.current_loop_Kp, icarStr.current_loop_Ki, icarStr.current_loop_Kd};
        PID_Init(&car_current_pid, PID_POSITION, current_loop_pid_buff, CURRENT_LOOP_MAX_OUT, CURRENT_LOOP_P_MAX_OUT);

        // д����ɣ��ر�д��״̬
        flashPIDEnable = false;
    }
}
