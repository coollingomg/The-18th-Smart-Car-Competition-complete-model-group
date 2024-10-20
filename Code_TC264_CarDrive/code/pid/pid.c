/*
 * pid.c
 *
 *  Created on: 2023年3月18日
 *      Author: wzl
 */


#include "pid.h"
#include "stddef.h"


// 限幅定义
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


//创建电机速度环结构体
PidTypeDef car_speed_pid;
//创建电机电流环结构体
PidTypeDef car_current_pid;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       pid控制参数初始化
// 参数说明       PidTypeDef* pid -> 控制器结构体
// 参数说明       uint8_t mode -> 控制模式
// 参数说明       const float PID[3] -> pid参数
// 参数说明       float max_out -> 输出最大值限幅
// 参数说明       float max_iout -> 输出积分项最大值限幅
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if(pid == NULL || PID == NULL)
        return;

    pid->mode_t = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       pid控制器
// 参数说明       PidTypeDef* pid -> 控制器结构体
// 参数说明       float ref -> 被控体返回状态值
// 参数说明       float set -> 被控体设定目标值
// 返回参数       float -> 控制输出量
//-------------------------------------------------------------------------------------------------------------------
float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    if(pid == NULL)
        return 0.0f;

    // 更新参数
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    // 控制模式选择
    if(pid->mode_t == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode_t == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

    // 返回控制输出量
    return pid->out;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       pid控制器参数清除
// 参数说明       PidTypeDef* pid -> 控制器结构体
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
        return;

    // 参数清除
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
