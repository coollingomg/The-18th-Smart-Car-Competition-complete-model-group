/*
 * icm20602_data_handle.h
 *
 *  Created on: 2023年3月16日
 *      Author: wzl
 */

#ifndef CODE_ICM20602_DATA_HANDLE_H_
#define CODE_ICM20602_DATA_HANDLE_H_

//包含头文件
#include "zf_device_icm20602.h"             //陀螺仪bsp层头文件


typedef struct
{
    float a_x;
    float a_y;
    float a_z;
    float g_x;
    float g_y;
    float g_z;
}Gyroscope_g_and_a_data;

typedef struct
{
    float yaw;
    float roll;
    float pitch;
}Gyroscope_attitude_Angle;


//外部声明陀螺仪姿态角结构体
extern Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get;
//外部声明陀螺仪数据结构体
extern Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get;


//==================================================ICM20602_DATA_HANDLE 基础函数==================================================
void icm20602_pose_init(void);                                          //姿态角解算初始化
void icm20602_attitude_Angle_pose(void);                                //姿态角解算
//==================================================ICM20602_DATA_HANDLE 基础函数==================================================

#endif /* CODE_ICM20602_DATA_HANDLE_H_ */
