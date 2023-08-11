/*
 * pid.h
 *
 *  Created on: 2023年3月18日
 *      Author: wzl
 */


#ifndef PID_H
#define PID_H


#include "cpu0_main.h"


// pid控制器模式枚举
enum PID_MODE
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA        //增量式PID
};


// pid控制器结构体
typedef struct
{
    uint8_t mode_t;

    //PID 参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;      //设定值
    float fdb;      //实际值

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
} PidTypeDef;


//声明电机速度环结构体
extern PidTypeDef car_speed_pid;
//声明电机电流环结构体
extern PidTypeDef car_current_pid;


//====================================================PID 基础函数====================================================
extern void  PID_Init        (PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc        (PidTypeDef *pid, float ref, float set);
extern void  PID_clear       (PidTypeDef *pid);
//====================================================PID 基础函数====================================================


#endif
