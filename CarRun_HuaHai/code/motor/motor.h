/*
 * motor.h
 *
 *  Created on: 2023年3月20日
 *      Author: wzl
 */
#ifndef CODE_MOTOR_MOTOR_H_
#define CODE_MOTOR_MOTOR_H_

//包含头文件
#include "cpu0_main.h"
#include "uart/uart.h"

//宏定义控制方式,0为高低电平控制正反装；1为两路pwm控制正反转
#define motor_control_measure   0

//电机pwm输出引脚宏定义
#if motor_control_measure
    #define pwm_positive    51      //使用的ATOM1_CH0_P21_2
    #define pwm_nagetive    70      //使用的ATOM1_CH0_P21_4
#else
    #define pwm_rotation    51      //使用的ATOM1_CH0_P21_2
    #define pwm_control     676     //使用的IO引脚控制方向
#endif

//编码器读入引脚宏定义
#define encoder_pin_CH1   6         //20.3
#define encoder_pin_CH2   7         //20.0
//使用定时器宏定义
#define USING_TIMER       4         //tim6

//输出限制宏定义
#define CAR_MAX_SPEED   9500        //避免满占空比，损坏mos管
#define CAR_IMAX_OUT    1500

//宏定义pi
#define PI_MOTOR                    3.1415926535898f
//电机控制周期T：10ms
#define MOTOR_CONTROL_CYCLE         0.01f

//定义电机控制结构体
typedef struct
{
    float ReductionRatio;                       //电机减速比
    float EncoderLine;                          //编码器线数
    int16 EncoderValue;                         //编码器实时速度,定时器的值
    float DiameterWheel;                        //轮子直径：mm
    uint8_t CloseLoop;                          //开环模式
    uint16_t Counter;                           //线程计数器
}MotorStruct;

//声明结构体
extern MotorStruct motorStr;

//====================================================MOTOR 基础函数====================================================
void motor_init                 (void);
void motor_SetPwmValue          (int16_t pwm);

#if USING_BLUETOOTH_OR_EGBOARD
void motor_ControlLoop          (float speed);
#else
void motor_ControlLoop          (void);
#endif

//====================================================MOTOR 基础函数====================================================


#endif /* CODE_MOTOR_MOTOR_H_ */
