/*
 * motor.c
 *
 *  Created on: 2023年3月20日
 *      Author: wzl
 */
#include "car_control/car_control.h"
#include "motor/motor.h"
#include "uart/uart.h"
#include "pid.h"


//定义电机运动结构体参数
MotorStruct motorStr;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
//选择电机控制方式
#if motor_control_measure
    //正转控制初始化，
    pwm_init(pwm_positive, 50000, 0);                        //占空比在0~10000.
    //反转控制初始化
    pwm_init(pwm_nagetive, 50000, 0);                        //控制电机方向
#else
    //速度控制初始化
    pwm_init(pwm_rotation, 50000, 0);                        //占空比在0~10000.
    //正反转控制初始化
    gpio_init(pwm_control, GPO, 0, GPO_PUSH_PULL);           //控制电机方向
#endif

    //初始时，电机速度为0
    motor_SetPwmValue(0);

    //编码器初始化
    encoder_quad_init(USING_TIMER, encoder_pin_CH1, encoder_pin_CH2);
    //编码器计数区清空
    encoder_clear_count(USING_TIMER);

    //电机模型初始化
    motorStr.EncoderLine = 500.0f;                          //编码器线数
    motorStr.ReductionRatio = 1.0f;                         //电机减速比
    motorStr.EncoderValue = 0;                              //初始化编码器实时速度
    motorStr.DiameterWheel = 0.062f;                        //轮子直径62cm，该参数单位为m
    motorStr.CloseLoop = 0;                                 //默认闭环模式|开环模式
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机速度
// 参数说明     int16 pwm     设置速度和正反转 范围：-10000~10000
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void motor_SetPwmValue(int16 pwm)
{
    if(pwm >= 0)
    {
        //限幅，防止超出范围
        if(pwm >= PWM_DUTY_MAX)
        {
            pwm = PWM_DUTY_MAX;
        }
        //设置电机正转
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
        //将符号变为正
        pwm = -pwm;
        //限幅，防止超出范围
        if(pwm >= PWM_DUTY_MAX)
        {
            pwm = PWM_DUTY_MAX;
        }
        //设置电机反转
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
// 函数简介     pid闭环速度
// 参数说明     float speed     输入设定的速度值
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
#if USING_BLUETOOTH_OR_EGBOARD
void motor_ControlLoop(float set_speed)
{
    //线程控制，每1ms进入一次该函数
    motorStr.Counter++;
    //每10ms对电机速度进行控制
    if(motorStr.Counter >= 10)
    {
        //获取当前编码器的值
        motorStr.EncoderValue = encoder_get_count(USING_TIMER);
        //清空定时器的值
        encoder_clear_count(USING_TIMER);
        //获取实际速度；计算公式：定时器计数值/4倍频/编码器线数/电机的减速比/循环时间*pi*车轮直径
        icarStr.SpeedFeedback = (float)(motorStr.EncoderValue * PI_MOTOR * motorStr.DiameterWheel)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;

        //定义给定pwm值
        static int16 speed_to_pwm = 0;

        //闭环
        if(Bluetooth_data.data_choice)
        {
            //pid计算，并赋值给pwm
            PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, set_speed);
            speed_to_pwm = (int16)car_speed_pid.out;
            //赋值pwm
            motor_SetPwmValue(speed_to_pwm);
        }
        else
        {
            //开环
            speed_to_pwm = (int16)(10000.0f / 10.0f * set_speed);
            //赋值pwm
            motor_SetPwmValue(speed_to_pwm);
        }

        //清空线程
        motorStr.Counter = 0;
    }
}
#else
void motor_ControlLoop(void)
{
    //线程控制，每1ms进入一次该函数
    motorStr.Counter++;
    //每10ms对电机速度进行控制
    if(motorStr.Counter >= 10)
    {
        //获取当前编码器的值
        motorStr.EncoderValue = -encoder_get_count(USING_TIMER);
        //清空定时器的值
        encoder_clear_count(USING_TIMER);
        //获取实际速度；计算公式：定时器计数值/4倍频/编码器线数/电机的减速比/循环时间*pi*车轮直径
        icarStr.SpeedFeedback = (float)(motorStr.EncoderValue * PI_MOTOR * motorStr.DiameterWheel)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;

        //通信连接或电机测试才开启闭环（保护+省电）
        if(icarStr.sprintEnable || usbStr.connected)
        {
            //定义给定pwm值
            static int16 speed_to_pwm = 0;

            //闭环速控
            if(motorStr.CloseLoop)
            {
                //pid计算，并赋值给pwm
                PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, icarStr.SpeedSet);
                speed_to_pwm = (int16)car_speed_pid.out;
                //赋值pwm
                motor_SetPwmValue(speed_to_pwm);
            }
            else
            {
                //开环百分比控制
                if(icarStr.SpeedSet > 100)
                    icarStr.SpeedSet = 100;
                else if(icarStr.SpeedSet < -100)
                    icarStr.SpeedSet = -100;
                //开环：百分比%
                speed_to_pwm = (int16)(CAR_MAX_SPEED / 100.0f * icarStr.SpeedSet);
                //赋值pwm
                motor_SetPwmValue(speed_to_pwm);
            }
        }
        else
        {
            //通讯没有连接，也没有电机测试，电机不转动
            motor_SetPwmValue(0);
        }

        //清空线程
        motorStr.Counter = 0;
    }
}
#endif
