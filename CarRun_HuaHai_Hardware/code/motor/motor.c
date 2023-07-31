/*
 * motor.c
 *
 *  Created on: 2023��3��20��
 *      Author: wzl
 */
#include "car_control/car_control.h"
#include "motor/motor.h"
#include "uart/uart.h"
#include "pid.h"
#include "Buzzer/buzzer.h"
#include "Kalman_Filter.h"

//�������˶��ṹ�����
MotorStruct motorStr;

//-------------------------------------------------------------------------------------------------------------------
// �������     �����ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    //�ٶȿ��Ƴ�ʼ��
    pwm_init(pwm_rotation, 100000, 0);                       //ռ�ձ���0~10000.
    //����ת���Ƴ�ʼ��
    gpio_init(pwm_control, GPO, 0, GPO_PUSH_PULL);           //���Ƶ������

    //��ʼʱ������ٶ�Ϊ0
    motor_SetPwmValue(0);

    //��������ʼ��
    encoder_quad_init(USING_TIMER, encoder_pin_CH1, encoder_pin_CH2);
    //���������������
    encoder_clear_count(USING_TIMER);

    //���ģ�ͳ�ʼ��
    motorStr.EncoderLine = 500.0f;                          //����������
    motorStr.ReductionRatio = 1.0f;                         //������ٱ�
    motorStr.EncoderValue = 0;                              //��ʼ��������ʵʱ�ٶ�
    motorStr.DiameterWheel = 0.062f;                        //����ֱ��62cm���ò�����λΪm
    motorStr.CloseLoop = 1;                                 //1:�ջ�ģʽ | 0:����ģʽ
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ٶ�
// ����˵��     int16 pwm     �����ٶȺ�����ת ��Χ��-10000~10000
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_SetPwmValue(int16 pwm)
{
    if(pwm >= 0)
    {
        //�޷�����ֹ������Χ
        if(pwm >= PWM_DUTY_MAX)
        {
            pwm = PWM_DUTY_MAX;
        }
        //���õ����ת
        gpio_set_level(pwm_control, 1);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
    }
    else
    {
        //�����ű�Ϊ��
        pwm = -pwm;
        //�޷�����ֹ������Χ
        if(pwm >= PWM_DUTY_MAX)
        {
            pwm = PWM_DUTY_MAX;
        }
        //���õ����ת
        gpio_set_level(pwm_control, 0);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     pid�ջ��ٶ�
// ����˵��     float speed     �����趨���ٶ�ֵ
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_ControlLoop(void)
{
    //�������pwmֵ
    static int16 speed_to_pwm = 0;
    //�߳̿��ƣ�ÿ0.5ms����һ�θú���
    motorStr.Counter++;
    //ÿ5ms�Ե���ٶȽ��п���
    if(motorStr.Counter >= 10)
    {
        //��ȡ��ǰ��������ֵ
        motorStr.EncoderValue = encoder_get_count(USING_TIMER);
        //��ն�ʱ����ֵ
        encoder_clear_count(USING_TIMER);
        //��ȡʵ���ٶȣ����㹫ʽ����ʱ������ֵ/4��Ƶ/����������/����ļ��ٱ�/ѭ��ʱ��*���Ӱ뾶*PI
        float temp_speed = (float)(motorStr.EncoderValue * motorStr.DiameterWheel * PI_MOTOR)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;
        icarStr.SpeedFeedback = Kalman_Filter_Fun(&kalman_struck, temp_speed);

        //ͨ�����Ӳſ����ջ�
        if(usbStr.connected)
        {
            //�ջ��ٿ�
            if(motorStr.CloseLoop)
            {
                // pid���㣬����ֵ��pwm
                PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, icarStr.SpeedSet);
                speed_to_pwm = (int16)(car_speed_pid.out + usbStr.recevie_k * icarStr.SpeedFeedback);
                // ��ֵpwm
                motor_SetPwmValue(speed_to_pwm);
            }
            else
            {
                //�����ٷֱȿ���
                if(icarStr.SpeedSet > 100)
                    icarStr.SpeedSet = 100;
                else if(icarStr.SpeedSet < -100)
                    icarStr.SpeedSet = -100;
                //�������ٷֱ�%�������ٶ���ϢΪ0-10��10Ϊ��ת
                speed_to_pwm = (int16)(CAR_MAX_SPEED / 100.0f * icarStr.SpeedSet * 10.0);
                //��ֵpwm
                motor_SetPwmValue(speed_to_pwm);
            }
        }
        else
        {
            //ͨѶû�����ӣ������ת��
            PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, 0);
            speed_to_pwm = (int16)car_speed_pid.out;
            //��ֵpwm
            motor_SetPwmValue(speed_to_pwm);
        }

        //����߳�
        motorStr.Counter = 0;
    }
}

