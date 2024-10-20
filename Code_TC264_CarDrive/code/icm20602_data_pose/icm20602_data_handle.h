/*
 * icm20602_data_handle.h
 *
 *  Created on: 2023年3月16日
 *      Author: wzl
 */

#ifndef CODE_ICM20602_DATA_HANDLE_H_
#define CODE_ICM20602_DATA_HANDLE_H_

//包含头文件
#include "zf_device_icm20602.h"
#include "cpu0_main.h"


// 定义imu原始数据结构体
typedef struct
{
    float a_x;
    float a_y;
    float a_z;
    float g_x;
    float g_y;
    float g_z;
}Gyroscope_g_and_a_data;


// 定义姿态角参数结构体
typedef struct
{
    float yaw;
    float roll;
    float pitch;
    bool  Flag_handle;
}Gyroscope_attitude_Angle;


typedef struct
{
    int16_t Xdata;
    int16_t Ydata;
    int16_t Zdata;
}gyro_param_t;


//外部声明陀螺仪姿态角结构体
extern Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get;
//外部声明陀螺仪数据结构体
extern Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get;
//外部声明陀螺仪零飘矫正数据结构体
extern gyro_param_t GyroOffset;


//==================================================ICM20602_DATA_HANDLE 基础函数==================================================
void icm20602_pose_init                 (void);                                //姿态角解算初始化
void gyroOffsetInit                     (void);                                //陀螺仪零漂初始化
void icm20602_attitude_Angle_handle     (void);                                //姿态角解算
void icm20602_attitude_Angle_Timer      (void);                                //线程计时
void IMUupdate (Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get_t, Gyroscope_attitude_Angle *Gyroscope_attitude_Angle_data_get_t);
//==================================================ICM20602_DATA_HANDLE 基础函数==================================================

#endif
