/*
 * motor.c
 *
 *  Created on: 2023年3月20日
 *      Author: wzl
 */

#include "SysSe/Math/Ifx_LowPassPt1F32.h"
#include "Kalman_Filter.h"
#include "car_control.h"
#include "motor.h"
#include "uart.h"
#include "pid.h"

//定义电机运动结构体参数
MotorStruct motorStr;
// 定义一阶低通滤波器结构体
Ifx_LowPassPt1F32 LowPass_filter_current;
// 定义一阶低通滤波器的参数配置结构体
Ifx_LowPassPt1F32_Config LowPass_filter_config_current;
// 定义一阶低通滤波器结构体
Ifx_LowPassPt1F32 LowPass_filter_speedset;
// 定义一阶低通滤波器的参数配置结构体
Ifx_LowPassPt1F32_Config LowPass_filter_config_speedset;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    //速度控制初始化
    pwm_init(pwm_rotation, 50000, 0);                        //占空比在0~10000.
    //正反转控制初始化
    gpio_init(pwm_control, GPO, 0, GPO_PUSH_PULL);           //控制电机方向

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
    motorStr.CloseLoop = 1;                                 //1:闭环模式 | 0:开环模式

    // 低通滤波器的初始化
    LowPass_filter_config_current.cutOffFrequency = 10;                                      // 低通滤波器的截止频率为10hz
    LowPass_filter_config_current.samplingTime = 0.005;                                      // 滤波的数据采样时间为0.005s
    LowPass_filter_config_current.gain = 0.85;                                               // 低通滤波器的增益为0.85
    Ifx_LowPassPt1F32_init(&LowPass_filter_current, &LowPass_filter_config_current);         // 低通滤波器的初始化

    // 低通滤波器的初始化
    LowPass_filter_config_speedset.cutOffFrequency = 10;                                      // 低通滤波器的截止频率为10hz
    LowPass_filter_config_speedset.samplingTime = 0.005;                                      // 滤波的数据采样时间为0.005s
    LowPass_filter_config_speedset.gain = 0.95;                                               // 低通滤波器的增益为0.85
    Ifx_LowPassPt1F32_init(&LowPass_filter_speedset, &LowPass_filter_config_speedset);        // 低通滤波器的初始化
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
        gpio_set_level(pwm_control, 1);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
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
        gpio_set_level(pwm_control, 0);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机速度环+电流环
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void motor_ControlLoop(void)
{
    // 定义给定pwm值
    static int16 speed_to_pwm = 0;

    // 线程控制，每0.5ms进入一次该函数
    motorStr.Counter_speed_loop++;
//    motorStr.Counter_current_loop++;

//    // 电机电流环控制，每1ms控制一次
//    if(motorStr.Counter_current_loop >= 2)
//    {
//        // 通信连接才闭环
//        if(usbStr.connected && motorStr.CloseLoop)
//        {
//            // 电机电流环pid控制器控制
////            motorStr.Set_current_motor = icarStr.SpeedSet;
//            PID_Calc(&car_current_pid, motorStr.Current_motor_After_filter, motorStr.Set_current_motor);
//
//            // 赋值pwm
//            speed_to_pwm = (int16)(car_current_pid.out);
//            motor_SetPwmValue(speed_to_pwm);
//        }
//
//        // 清空线程
//        motorStr.Counter_current_loop = 0;
//    }

    // 每5ms对电机速度环进行控制
    if(motorStr.Counter_speed_loop >= 10)
    {
        // 获取当前编码器的值
        motorStr.EncoderValue = encoder_get_count(USING_TIMER);
        // 清空定时器的值
        encoder_clear_count(USING_TIMER);
        // 获取实际速度；计算公式：定时器计数值/4倍频/编码器线数/电机的减速比/循环时间*轮子半径*PI
        float temp_speed = (float)(motorStr.EncoderValue * motorStr.DiameterWheel * PI_MOTOR)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;
        // 对于反馈速度进行卡尔曼滤波
        icarStr.SpeedFeedback = Kalman_Filter_Fun(&kalman_motor_speedback, temp_speed);
//        icarStr.SpeedFeedback = temp_speed;

        //通信连接才闭环
        if(usbStr.connected)
        {
            //闭环速控
            if(motorStr.CloseLoop)
            {
                // 速度环pid控制器控制
                icarStr.speed_set = Ifx_LowPassPt1F32_do(&LowPass_filter_speedset, icarStr.SpeedSet);
                PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, icarStr.speed_set);
                // 电流环pid控制器控制输入
//                motorStr.Set_current_motor = Ifx_LowPassPt1F32_do(&LowPass_filter_current, car_speed_pid.out);
//                if(icarStr.SpeedSet >= 0)
//                    motorStr.Set_current_motor = car_speed_pid.out + 0.15;
//                else
//                    motorStr.Set_current_motor = car_speed_pid.out - 0.15;
//                motorStr.Set_current_motor = car_speed_pid.out;

                // 赋值pwm
                speed_to_pwm = (int16)(car_speed_pid.out);
                motor_SetPwmValue(speed_to_pwm);
            }
            else
            {
                // 开环百分比控制
                if(icarStr.SpeedSet >= 10)
                    icarStr.SpeedSet = 10;
                else if(icarStr.SpeedSet <= -10)
                    icarStr.SpeedSet = -10;
                // 开环，百分比，输入速度信息为0-10，10为满转
                speed_to_pwm = (int16)(SPEED_LOOP_MAX_OUT * icarStr.SpeedSet / 10.0f);
                // 赋值pwm
                motor_SetPwmValue(speed_to_pwm);
            }
        }
        else
        {
            // 通信未连接，不输出pwm
            motor_SetPwmValue(0);
        }

        // 清空线程
        motorStr.Counter_speed_loop = 0;
    }
}
