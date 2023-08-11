/*
 * Kalman_Filter.h
 *
 *  Created on: 2023年5月15日
 *      Author: wzl
 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


//Kalman Filter parameter
typedef struct 
{
    float LastP;    //上次估算协方差 初始化值为0.02
    float Now_P;    //当前估算协方差 初始化值为0
    float out;      //卡尔曼滤波器输出 初始化值为0
    float Kg;       //卡尔曼增益 初始化值为0
    float Q;        //过程噪声协方差 初始化值为0.001
    float R;        //观测噪声协方差 初始化值为0.543
}KalmanInfo;


// 声明卡尔曼滤波器结构体参数
extern KalmanInfo kalman_motor_speedback;
extern KalmanInfo kalman_imu_yaw;


//==================================================Kalman 基础函数==================================================
void   Kalman_Filter_Init       (KalmanInfo *KalmanInfo_Structure);
float  Kalman_Filter_Fun        (KalmanInfo *info, float new_value);
//==================================================Kalman 基础函数==================================================

#endif
