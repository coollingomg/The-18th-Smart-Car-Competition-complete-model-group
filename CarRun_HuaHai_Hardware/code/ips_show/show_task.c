/*
 * show_task.c
 *
 *  Created on: 2023年3月18日
 *      Author: wzl
 */

//包含头文件
#include "icm20602_data_handle.h"
#include "show_task.h"


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示屏初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void ips_show_init(void)
{
    //设置显示方向
    ips200_set_dir(IPS200_PORTAIT);
    //设置字体
    ips200_set_font(IPS200_8X16_FONT);
    //显示屏硬件初始化
    ips200_init(IPS200_TYPE_PARALLEL8);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示任务
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void ips_show_task(void)
{
    //显示yaw值
    ips200_show_string(0, 0, "yaw:");
    ips200_show_float(32, 0, Gyroscope_attitude_Angle_data_get.yaw, 3, 2);
    //显示pitch值
    ips200_show_string(0, 20, "pitch:");
    ips200_show_float(48, 0, Gyroscope_attitude_Angle_data_get.pitch, 3, 2);
    //显示roll值
    ips200_show_string(0, 40, "roll:");
    ips200_show_float(40, 0, Gyroscope_attitude_Angle_data_get.roll, 3, 2);
}
