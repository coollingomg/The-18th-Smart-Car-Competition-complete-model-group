/*
 * icm20602_data_handle.h
 *
 *  Created on: 2023��3��16��
 *      Author: wzl
 */

#ifndef CODE_ICM20602_DATA_HANDLE_H_
#define CODE_ICM20602_DATA_HANDLE_H_

//����ͷ�ļ�
#include "zf_device_icm20602.h"             //������bsp��ͷ�ļ�


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


//�ⲿ������������̬�ǽṹ��
extern Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get;
//�ⲿ�������������ݽṹ��
extern Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get;


//==================================================ICM20602_DATA_HANDLE ��������==================================================
void icm20602_pose_init(void);                                          //��̬�ǽ����ʼ��
void icm20602_attitude_Angle_pose(void);                                //��̬�ǽ���
//==================================================ICM20602_DATA_HANDLE ��������==================================================

#endif /* CODE_ICM20602_DATA_HANDLE_H_ */
