/*
 * servo.h
 *
 *  Created on: 2023年3月24日
 *      Author: wzl
 */
#ifndef CODE_SERVO_SERVO_H_
#define CODE_SERVO_SERVO_H_

//包含头文件
#include "zf_driver_pwm.h"
#include "cpu0_main.h"

//宏定义舵机控制引脚
#define servo_pwm_out_pin   13      //使用的ATOM0_CH1_P33_9

//比例换算
#define TO_PENCENT          (1.0f/1000.0f/20.0f*10000.0f)
//宏定义输出PWM最小值
#define SERVO_PWM_MIN       500
//宏定义输出PWM最大值
#define SERVO_PWM_MAX       2500
//宏定义pwm中值
#define medium_point        1500
//宏定义物理限制下最大转动角度
#define MAX_ANGLE           38.0f
//宏定义角度换算pwm关系式
#define translate_angle_to_pwm  (1000.0f / 90.0f)

//舵机由于物理安装的误差，需要补偿；上位机发送参数来进行补偿修正，定义如下结构体
typedef struct
{
    uint16_t thresholdMiddle;                   //舵机中值PWM
    uint16_t thresholdLeft;                     //舵机左向转角最大值PWM
    uint16_t thresholdRight;                    //舵机右向转角最大值PWM
    uint8_t  count;                             //线程计数器
    bool     count_handle;                      //线程处理标志位
}ServoStruct;

//声明舵机运动矫正结构体参数
extern ServoStruct servoStr;

//====================================================SERVO 基础函数====================================================
void servo_init                 (void);
void servo_contral              (float angle);
void SERVO_SetPwmValueCorrect   (uint16 pwm);
void SERVO_Timer                (void);
void SERVO_Handle               (void);
//====================================================SERVO 基础函数====================================================

#endif /* CODE_SERVO_SERVO_H_ */
