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
//声明函数
void IMUupdate(Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get, Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get);


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪任务初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_pose_init(void)
{
    //初始化陀螺仪icm20602
    while(icm20602_init());
//    //初始化姿态角解算各个参数
//    initPose_Module(&icm20602_pose);
    //点灯，说明初始化成功
    gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL);

    //初始化获取数据结构体
    Gyroscope_g_and_a_data_get.a_x = 0;
    Gyroscope_g_and_a_data_get.a_y = 0;
    Gyroscope_g_and_a_data_get.a_z = 0;
    Gyroscope_g_and_a_data_get.g_x = 0;
    Gyroscope_g_and_a_data_get.g_y = 0;
    Gyroscope_g_and_a_data_get.g_z = 0;

//    //地址关联
//    icm20602_pose.interface.data.a_x = &Gyroscope_g_and_a_data_get.a_x;
//    icm20602_pose.interface.data.a_y = &Gyroscope_g_and_a_data_get.a_y;
//    icm20602_pose.interface.data.a_z = &Gyroscope_g_and_a_data_get.a_z;
//    icm20602_pose.interface.data.g_x = &Gyroscope_g_and_a_data_get.g_x;
//    icm20602_pose.interface.data.g_y = &Gyroscope_g_and_a_data_get.g_y;
//    icm20602_pose.interface.data.g_z = &Gyroscope_g_and_a_data_get.g_z;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪姿态角解算更新
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_attitude_Angle_pose(void)
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

//    //姿态角解算
//    calculatePose_Module(pose, cycle);
    //姿态角解算
    IMUupdate(Gyroscope_g_and_a_data_get, Gyroscope_attitude_Angle_data_get);

//    //获取姿态角
//    Gyroscope_attitude_Angle_data_get.pitch = pose->data.pit;
//    Gyroscope_attitude_Angle_data_get.roll = pose->data.rol;
//    Gyroscope_attitude_Angle_data_get.yaw = pose->data.yaw;
}


/**********************四元数解算***********************/
#define Kp 10.0f                            // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                           // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0005f                       // half the sample period采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      // scaled integral error

void IMUupdate(Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get, Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get)
{
    float norm;
    //float hx, hy, hz, bx, bz;
    float vx, vy, vz;// wx, wy, wz;
    float ex, ey, ez;

    // 先把这些用得到的值算好
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    //  float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    //  float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(Gyroscope_g_and_a_data_get.a_x*Gyroscope_g_and_a_data_get.a_y*Gyroscope_g_and_a_data_get.a_z==0)
        return;

    //acc数据归一化
    norm = sqrt(Gyroscope_g_and_a_data_get.a_x*Gyroscope_g_and_a_data_get.a_x +
                Gyroscope_g_and_a_data_get.a_y*Gyroscope_g_and_a_data_get.a_y +
                Gyroscope_g_and_a_data_get.a_z*Gyroscope_g_and_a_data_get.a_z);
    Gyroscope_g_and_a_data_get.a_x = Gyroscope_g_and_a_data_get.a_x /norm;
    Gyroscope_g_and_a_data_get.a_y = Gyroscope_g_and_a_data_get.a_y / norm;
    Gyroscope_g_and_a_data_get.a_z = Gyroscope_g_and_a_data_get.a_z / norm;

    // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
    vx = 2*(q1q3 - q0q2);                                             //四元素中xyz的表示
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (Gyroscope_g_and_a_data_get.a_y*vz - Gyroscope_g_and_a_data_get.a_z*vy) ;                                             //向量外积在相减得到差分就是误差
    ey = (Gyroscope_g_and_a_data_get.a_z*vx - Gyroscope_g_and_a_data_get.a_x*vz) ;
    ez = (Gyroscope_g_and_a_data_get.a_x*vy - Gyroscope_g_and_a_data_get.a_y*vx) ;

    exInt = exInt + ex * Ki;                                //对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // adjusted gyroscope measurements
    Gyroscope_g_and_a_data_get.g_x = Gyroscope_g_and_a_data_get.g_x + Kp*ex + exInt; //将误差PI后补偿到陀螺仪，即补偿零点漂移
    Gyroscope_g_and_a_data_get.g_y = Gyroscope_g_and_a_data_get.g_y + Kp*ey + eyInt;
    Gyroscope_g_and_a_data_get.g_z = Gyroscope_g_and_a_data_get.g_z + Kp*ez + ezInt; //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // integrate quaternion rate and normalise                           //四元素的微分方程
    q0 = q0 + (-q1*Gyroscope_g_and_a_data_get.g_x - q2*Gyroscope_g_and_a_data_get.g_y - q3*Gyroscope_g_and_a_data_get.g_z)*halfT;
    q1 = q1 + (q0*Gyroscope_g_and_a_data_get.g_x + q2*Gyroscope_g_and_a_data_get.g_z - q3*Gyroscope_g_and_a_data_get.g_y)*halfT;
    q2 = q2 + (q0*Gyroscope_g_and_a_data_get.g_y - q1*Gyroscope_g_and_a_data_get.g_z + q3*Gyroscope_g_and_a_data_get.g_x)*halfT;
    q3 = q3 + (q0*Gyroscope_g_and_a_data_get.g_z + q1*Gyroscope_g_and_a_data_get.g_y - q2*Gyroscope_g_and_a_data_get.g_x)*halfT;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    Gyroscope_attitude_Angle_data_get.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3;        // yaw
    Gyroscope_attitude_Angle_data_get.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;                                   // pitch
    Gyroscope_attitude_Angle_data_get.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;     // roll
}

