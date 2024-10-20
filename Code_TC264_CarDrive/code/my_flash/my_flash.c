/*
 * my_flash.c
 *
 *  Created on: 2023年3月30日
 *      Author: wzl
 */


#include "car_control/car_control.h"
#include "my_flash/my_flash.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "uart/uart.h"
#include "pid/pid.h"


//定义全局变量，来处理是否进行数据存储
bool flashSaveEnable = false;
bool flashPIDEnable = false;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取Flash数据，初始化相关参数
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void my_flash_init(void)
{
#if USING_BLUETOOTH_OR_EGBOARD
    //读取扇区0的一页数据到缓存区
    flash_read_page_to_buffer(0, 1);
    //处理扇区读取的数据->舵机矫正的中值
    servoStr.thresholdMiddle = flash_union_buffer[0].uint16_type;
    //处理扇区读取的数据->舵机矫正的左转阈值
    servoStr.thresholdLeft = flash_union_buffer[1].uint16_type;
    //处理扇区读取的数据->舵机矫正的右转阈值
    servoStr.thresholdRight = flash_union_buffer[2].uint16_type;
#else
    //读取扇区0的一页数据到缓存区
    flash_read_page_to_buffer(0, 1);
    //处理扇区读取的数据->舵机矫正的中值
    servoStr.thresholdMiddle = flash_union_buffer[0].uint16_type;
    //处理扇区读取的数据->舵机矫正的左转阈值
    servoStr.thresholdLeft = flash_union_buffer[1].uint16_type;
    //处理扇区读取的数据->舵机矫正的右转阈值
    servoStr.thresholdRight = flash_union_buffer[2].uint16_type;

    //读取扇区0的一页数据到缓存区
    flash_read_page_to_buffer(0, 0);
    //处理扇区读取的数据->底层运动速度控制pid参数->speed_loop_Kp
    icarStr.speed_loop_Kp = flash_union_buffer[0].float_type;
    //处理扇区读取的数据->底层运动速度控制pid参数->speed_loop_Ki
    icarStr.speed_loop_Ki = flash_union_buffer[1].float_type;
    //处理扇区读取的数据->底层运动速度控制pid参数->speed_loop_Kd
    icarStr.speed_loop_Kd = flash_union_buffer[2].float_type;
    //处理扇区读取的数据->底层运动速度控制pid参数->current_loop_Kp
    icarStr.current_loop_Kp = flash_union_buffer[3].float_type;
    //处理扇区读取的数据->底层运动速度控制pid参数->current_loop_Ki
    icarStr.current_loop_Ki = flash_union_buffer[4].float_type;
    //处理扇区读取的数据->底层运动速度控制pid参数->current_loop_Kd
    icarStr.current_loop_Kd = flash_union_buffer[5].float_type;
#endif
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存入Flash数据，初始化相关参数
// 参数说明     float* buf  要写入pid参数的数组地址
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void my_flash_write_pid(float* buf)
{
    //扇区数据的写入->底层运动速度控制pid参数->speed_loop_Kp
    flash_union_buffer[0].float_type = buf[0];
    //扇区数据的写入->底层运动速度控制pid参数->speed_loop_Ki
    flash_union_buffer[1].float_type = buf[1];
    //扇区数据的写入->底层运动速度控制pid参数->speed_loop_Kd
    flash_union_buffer[2].float_type = buf[2];
    //扇区数据的写入->底层运动速度控制pid参数->current_loop_Kp
    flash_union_buffer[3].float_type = buf[3];
    //扇区数据的写入->底层运动速度控制pid参数->current_loop_Ki
    flash_union_buffer[4].float_type = buf[4];
    //扇区数据的写入->底层运动速度控制pid参数->current_loop_Kd
    flash_union_buffer[5].float_type = buf[5];

    //写入前先清除该扇区
    flash_erase_page(0, 0);
    //等待扇区写入
    while(flash_write_page_from_buffer(0, 0));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存入Flash数据，初始化相关参数
// 参数说明     uint16* buf              pwmValue
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void my_flash_write_pwmValue(uint16* buf)
{
    //扇区数据的写入->舵机矫正的中值
    flash_union_buffer[0].uint16_type = buf[0];
    //扇区数据的写入->舵机矫正的左转阈值
    flash_union_buffer[1].uint16_type = buf[1];
    //扇区数据的写入->舵机矫正的右转阈值
    flash_union_buffer[2].uint16_type = buf[2];

    //写入前先清除该扇区
    flash_erase_page(0, 1);
    //等待扇区写入
    while(flash_write_page_from_buffer(0, 1));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存数据控制
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void my_flash_Handle(void)
{
    // 舵机数据存储
    if(flashSaveEnable)
    {
        // 获取pwmValue值
        uint16 pwmValue_buff[3] = {servoStr.thresholdMiddle, servoStr.thresholdLeft, servoStr.thresholdRight};
        // 写入数据pwmValue
        my_flash_write_pwmValue(pwmValue_buff);
        // 写入完成，关闭写入状态
        flashSaveEnable = false;
    }

    // PID数据存储
    if(flashPIDEnable)
    {
        // 获取电机控制控制参数
        float pidValue_buff[6] = {icarStr.speed_loop_Kp, icarStr.speed_loop_Ki, icarStr.speed_loop_Kd,
                                  icarStr.current_loop_Kp, icarStr.current_loop_Ki, icarStr.current_loop_Kd};

        // 写入数据PID数据
        my_flash_write_pid(pidValue_buff);

        // 初始化speed_loop_pid参数
        float speed_loop_pid_buff[3] = {icarStr.speed_loop_Kp, icarStr.speed_loop_Ki, icarStr.speed_loop_Kd};
        PID_Init(&car_speed_pid, PID_POSITION, speed_loop_pid_buff, SPEED_LOOP_MAX_OUT, SPEED_LOOP_P_MAX_OUT);
        // 初始化current_loop_pid参数
        float current_loop_pid_buff[3] = {icarStr.current_loop_Kp, icarStr.current_loop_Ki, icarStr.current_loop_Kd};
        PID_Init(&car_current_pid, PID_POSITION, current_loop_pid_buff, CURRENT_LOOP_MAX_OUT, CURRENT_LOOP_P_MAX_OUT);

        // 写入完成，关闭写入状态
        flashPIDEnable = false;
    }
}
