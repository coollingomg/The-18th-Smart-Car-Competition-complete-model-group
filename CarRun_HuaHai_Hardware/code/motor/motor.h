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


//电机pwm输出引脚宏定义
#define pwm_rotation            70        //使用的ATOM1_CH0_P21_4
#define pwm_control             674       //使用的IO引脚控制方向21_2


//编码器读入引脚宏定义
#define encoder_pin_CH1         1         //33.7
#define encoder_pin_CH2         1         //33.6
//使用定时器宏定义
#define USING_TIMER             0         //TIM2


//输出电机速度环、电流环输出限幅宏定义
//#define SPEED_LOOP_MAX_OUT       5
//#define SPEED_LOOP_P_MAX_OUT     1
#define CURRENT_LOOP_MAX_OUT     10000
#define CURRENT_LOOP_P_MAX_OUT   8000
#define SPEED_LOOP_MAX_OUT       10000
#define SPEED_LOOP_P_MAX_OUT     5000


//宏定义pi
#define PI_MOTOR                 3.1415926535898f
//电机速度环控制周期为5ms
#define MOTOR_CONTROL_CYCLE      0.005f


//定义电机参数结构体
typedef struct
{
    float    ReductionRatio;                    //电机减速比
    float    EncoderLine;                       //编码器线数
    int16    EncoderValue;                      //编码器实时速度,定时器的值
    float    DiameterWheel;                     //轮子直径：mm
    uint8_t  CloseLoop;                         //开环模式

    uint16_t Counter_speed_loop;                //速度环线程计数器
    uint16_t Counter_current_loop;              //电流环线程计数器

    float Current_motor;                        //电机工作电流
    float Current_motor_After_filter;           //低通滤波后的电流值
    float Set_current_motor;                    //电机设定目标电流
}MotorStruct;


//声明电机参数结构体
extern MotorStruct motorStr;


//====================================================MOTOR 基础函数====================================================
void motor_init                 (void);
void motor_SetPwmValue          (int16_t pwm);
void motor_ControlLoop          (void);
//====================================================MOTOR 基础函数====================================================

#endif
