/*
 * Kalman_Filter.c
 *
 *  Created on: 2023年5月15日
 *      Author: wzl
 */

#include "Kalman_Filter.h"

//定义卡尔曼结构体参数
KalmanInfo kalman_motor_speedback;       // 电机返回速度滤波
KalmanInfo kalman_imu_yaw;               // 偏航角加速度滤波


//初始化卡尔曼滤波结构体
void Kalman_Filter_Init(KalmanInfo *KalmanInfo_Structure)
{
    KalmanInfo_Structure->LastP = 0.02f;
    KalmanInfo_Structure->Now_P = 0.0f;
    KalmanInfo_Structure->out = 0.0f;
    KalmanInfo_Structure->Kg = 0.0f;
    KalmanInfo_Structure->Q = 0.001f;
    KalmanInfo_Structure->R = 0.58f;
    return;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     卡尔曼滤波函数
// 参数说明     info            卡尔曼滤波结构体指针
// 参数说明     new_value       新的测量值
// 返回参数     滤波后的估计值
//-------------------------------------------------------------------------------------------------------------------
float Kalman_Filter_Fun(KalmanInfo *info, float new_value)
{
    //先验协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    info->Now_P = info->LastP + info->Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    info->Kg = info->Now_P / (info->Now_P + info->R);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    info->out = info->out + info->Kg * (new_value -info->out);//因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差赋给 info->LastP 为下一次运算准备。
    info->LastP = (1-info->Kg) * info->Now_P;
    return info->out;
}


