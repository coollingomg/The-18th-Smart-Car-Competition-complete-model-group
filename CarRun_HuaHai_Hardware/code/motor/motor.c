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


//�������˶��ṹ�����
MotorStruct motorStr;

//-------------------------------------------------------------------------------------------------------------------
// �������     �����ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
//ѡ�������Ʒ�ʽ
#if motor_control_measure
    //��ת���Ƴ�ʼ����
    pwm_init(pwm_positive, 50000, 0);                        //ռ�ձ���0~10000.
    //��ת���Ƴ�ʼ��
    pwm_init(pwm_nagetive, 50000, 0);                        //���Ƶ������
#else
    //�ٶȿ��Ƴ�ʼ��
    pwm_init(pwm_rotation, 50000, 0);                        //ռ�ձ���0~10000.
    //����ת���Ƴ�ʼ��
    gpio_init(pwm_control, GPO, 0, GPO_PUSH_PULL);           //���Ƶ������
#endif

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
    motorStr.CloseLoop = 0;                                 //Ĭ�ϱջ�ģʽ|����ģʽ
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
#if motor_control_measure
        pwm_set_duty(pwm_positive, (uint32)pwm);
        pwm_set_duty(pwm_nagetive, 0);
#else
        gpio_set_level(pwm_control, 0);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
#endif

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
#if motor_control_measure
        pwm_set_duty(pwm_positive, 0);
        pwm_set_duty(pwm_nagetive, (uint32)pwm);
#else
        gpio_set_level(pwm_control, 1);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
#endif
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     pid�ջ��ٶ�
// ����˵��     float speed     �����趨���ٶ�ֵ
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
#if USING_BLUETOOTH_OR_EGBOARD
void motor_ControlLoop(float set_speed)
{
    //�߳̿��ƣ�ÿ1ms����һ�θú���
    motorStr.Counter++;
    //ÿ10ms�Ե���ٶȽ��п���
    if(motorStr.Counter >= 10)
    {
        //��ȡ��ǰ��������ֵ
        motorStr.EncoderValue = encoder_get_count(USING_TIMER);
        //��ն�ʱ����ֵ
        encoder_clear_count(USING_TIMER);
        //��ȡʵ���ٶȣ����㹫ʽ����ʱ������ֵ/4��Ƶ/����������/����ļ��ٱ�/ѭ��ʱ��*pi*����ֱ��
        icarStr.SpeedFeedback = (float)(motorStr.EncoderValue * PI_MOTOR * motorStr.DiameterWheel)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;

        //�������pwmֵ
        static int16 speed_to_pwm = 0;

        //�ջ�
        if(Bluetooth_data.data_choice)
        {
            //pid���㣬����ֵ��pwm
            PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, set_speed);
            speed_to_pwm = (int16)car_speed_pid.out;
            //��ֵpwm
            motor_SetPwmValue(speed_to_pwm);
        }
        else
        {
            //����
            speed_to_pwm = (int16)(10000.0f / 10.0f * set_speed);
            //��ֵpwm
            motor_SetPwmValue(speed_to_pwm);
        }

        //����߳�
        motorStr.Counter = 0;
    }
}
#else
void motor_ControlLoop(void)
{
    //�߳̿��ƣ�ÿ1ms����һ�θú���
    motorStr.Counter++;
    //ÿ10ms�Ե���ٶȽ��п���
    if(motorStr.Counter >= 10)
    {
        //��ȡ��ǰ��������ֵ
        motorStr.EncoderValue = -encoder_get_count(USING_TIMER);
        //��ն�ʱ����ֵ
        encoder_clear_count(USING_TIMER);
        //��ȡʵ���ٶȣ����㹫ʽ����ʱ������ֵ/4��Ƶ/����������/����ļ��ٱ�/ѭ��ʱ��*pi*����ֱ��
        icarStr.SpeedFeedback = (float)(motorStr.EncoderValue * PI_MOTOR * motorStr.DiameterWheel)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;

        //ͨ�����ӻ������Բſ����ջ�������+ʡ�磩
        if(icarStr.sprintEnable || usbStr.connected)
        {
            //�������pwmֵ
            static int16 speed_to_pwm = 0;

            //�ջ��ٿ�
            if(motorStr.CloseLoop)
            {
                //pid���㣬����ֵ��pwm
                PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, icarStr.SpeedSet);
                speed_to_pwm = (int16)car_speed_pid.out;
                //��ֵpwm
                motor_SetPwmValue(speed_to_pwm);
            }
            else
            {
                //�����ٷֱȿ���
                if(icarStr.SpeedSet > 100)
                    icarStr.SpeedSet = 100;
                else if(icarStr.SpeedSet < -100)
                    icarStr.SpeedSet = -100;
                //�������ٷֱ�%
                speed_to_pwm = (int16)(CAR_MAX_SPEED / 100.0f * icarStr.SpeedSet);
                //��ֵpwm
                motor_SetPwmValue(speed_to_pwm);
            }
        }
        else
        {
            //ͨѶû�����ӣ�Ҳû�е�����ԣ������ת��
            motor_SetPwmValue(0);
        }

        //����߳�
        motorStr.Counter = 0;
    }
}
#endif
