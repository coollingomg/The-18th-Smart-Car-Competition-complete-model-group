/*
 * icm20602_data_handle.c
 *
 *  Created on: 2023年3月15日
 *      Author: wzl
 */
//包含头文件
#include "icm20602_data_handle.h"
#include "zf_driver_gpio.h"


//定义陀螺仪姿态角结构体
Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get;
//外部声明陀螺仪数据结构体
Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪任务初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_pose_init(void)
{
    //初始化陀螺仪icm20602
    while(icm20602_init());
    //初始化姿态角解算各个参数
    initPose_Module(&icm20602_pose);
    //点灯，说明初始化成功
    gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL);


    //初始化获取数据结构体
    Gyroscope_g_and_a_data_get.a_x = 0;
    Gyroscope_g_and_a_data_get.a_y = 0;
    Gyroscope_g_and_a_data_get.a_z = 0;
    Gyroscope_g_and_a_data_get.g_x = 0;
    Gyroscope_g_and_a_data_get.g_y = 0;
    Gyroscope_g_and_a_data_get.g_z = 0;

    //地址关联
    icm20602_pose.interface.data.a_x = &Gyroscope_g_and_a_data_get.a_x;
    icm20602_pose.interface.data.a_y = &Gyroscope_g_and_a_data_get.a_y;
    icm20602_pose.interface.data.a_z = &Gyroscope_g_and_a_data_get.a_z;
    icm20602_pose.interface.data.g_x = &Gyroscope_g_and_a_data_get.g_x;
    icm20602_pose.interface.data.g_y = &Gyroscope_g_and_a_data_get.g_y;
    icm20602_pose.interface.data.g_z = &Gyroscope_g_and_a_data_get.g_z;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪姿态角解算更新
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_attitude_Angle_pose(Pose_Module *pose, float cycle)
{
    //关联变量
    icm20602_get_acc();
    Gyroscope_g_and_a_data_get.a_x = icm20602_acc_transition(icm20602_acc_x);
    Gyroscope_g_and_a_data_get.a_x = icm20602_acc_transition(icm20602_acc_y);
    Gyroscope_g_and_a_data_get.a_x = icm20602_acc_transition(icm20602_acc_z);
    icm20602_get_gyro();
    Gyroscope_g_and_a_data_get.g_x = icm20602_gyro_transition(icm20602_gyro_x);
    Gyroscope_g_and_a_data_get.g_y = icm20602_gyro_transition(icm20602_gyro_y);
    Gyroscope_g_and_a_data_get.g_z = icm20602_gyro_transition(icm20602_gyro_z);

    //姿态角解算
    calculatePose_Module(pose, cycle);

    //获取姿态角
    Gyroscope_attitude_Angle_data_get.pitch = pose->data.pit;
    Gyroscope_attitude_Angle_data_get.roll = pose->data.rol;
    Gyroscope_attitude_Angle_data_get.yaw = pose->data.yaw;
}




