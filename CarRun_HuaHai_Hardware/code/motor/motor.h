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
    #define pwm_rotation    70      //使用的ATOM1_CH0_P21_4
    #define pwm_control     674     //使用的IO引脚控制方向21_2
#endif

//编码器读入引脚宏定义
#define encoder_pin_CH1   1         //33.7
#define encoder_pin_CH2   1         //33.6
//使用定时器宏定义
#define USING_TIMER       0         //tim2

//输出限制宏定义
#define CAR_MAX_SPEED   9500        //避免满占空比，损坏mos管
#define CAR_IMAX_OUT    4000

//宏定义pi
#define PI_MOTOR                    3.1415926535898f
//电机控制周期T：5ms
#define MOTOR_CONTROL_CYCLE         0.005f

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
void motor_ControlLoop          (void);
//====================================================MOTOR 基础函数====================================================


#endif /* CODE_MOTOR_MOTOR_H_ */
