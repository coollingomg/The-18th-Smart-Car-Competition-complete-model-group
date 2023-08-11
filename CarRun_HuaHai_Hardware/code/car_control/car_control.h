/*
 * car_control.h
 *
 *  Created on: 2023年3月27日
 *      Author: wzl
 */

#ifndef CODE_CAR_CONTROL_CAR_CONTROL_H_
#define CODE_CAR_CONTROL_CAR_CONTROL_H_

//包含头文件
#include "cpu0_main.h"


//智能车自检步骤枚举，未使用过
typedef enum
{
    Selfcheck_None = 0,             //开始测试
    Selfcheck_MotorA,               //电机正转启动
    Selfcheck_MotorB,               //电机正转采样
    Selfcheck_MotorC,               //电机反转启动
    Selfcheck_MotorD,               //电机反转采样
    Selfcheck_MotorE,               //电机闭环正传启动
    Selfcheck_MotorF,               //电机闭环正传采样
    Selfcheck_MotorG,               //电机闭环反转启动
    Selfcheck_MotorH,               //电机闭环反转采样
    Selfcheck_ServoA,               //舵机测试A
    Selfcheck_Com,                  //通信测试
    Selfcheck_Buzzer,               //蜂鸣器测试
    Selfcheck_RgbLed,               //灯效测试
    Selfcheck_Key,                  //按键测试
    Selfcheck_Finish                //测试完成
}SelfcheckEnum;


//智能车相关参数控制结构体
typedef struct
{
    float  Voltage;                           //电池电压
    float  SpeedSet;                          //电机目标速度：m/s
    float  speed_set;                         //滤波后的目标速度：m/s
    float  SpeedFeedback;                     //电机模型实测速度：m/s
    uint16 ServoPwmSet;                       //舵机PWM设置

    float  speed_loop_Kp;                     //速度环kp
    float  speed_loop_Ki;                     //速度环ki
    float  speed_loop_Kd;                     //速度环kd
    float  current_loop_Kp;                   //电流环kp
    float  current_loop_Ki;                   //电流环ki
    float  current_loop_Kd;                   //电流环kd

    uint16 counterKeyA;                       //按键模式A计数器
    bool   keyPressed;                        //按键按下
    uint16 errorCode;                         //错误代码

    bool   selfcheckEnable;                   //智能车自检使能
    uint16 counterSelfcheck;                  //自检计数器
    uint8  timesSendStep;                     //发送超时数据次数
    uint16 counterModuleCheck;                //自检计数器
    SelfcheckEnum selfcheckStep;              //自检步骤
    uint8  speedSampleStep;                   //速度采样步骤
}IcarStruct;


//声明结构体
extern IcarStruct icarStr;


//==================================================CAR_CONTROL 基础函数==================================================
void ICAR_Init          (void);
void ICAR_Timer         (void);
void ICAR_Handle        (void);
//==================================================CAR_CONTROL 基础函数==================================================


#endif
