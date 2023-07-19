#pragma once

/**
 * @file motion_controller.cpp
 * @brief 运动控制器：姿态控制||速度控制
 * @version 0.3
 * @date 2023-05-23
 * @copyright Copyright (c) 2023
 */

#include <cmath>
#include <fstream>
#include <iostream>

#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter_cal.cpp"
#include "./recognize/track_recognition.cpp"
#include "math.h"

using namespace std;

class MotionController 
{
public:
    MotionController()
    {
        loadParams();
        Mid_line = (double)params.Control_Mid;
    }
private:
  int counterShift = 0; // 变速计数器

public:
    /**
     * @brief 控制器核心参数
     */
    struct Params 
    {
        float speedLow = 1.0;       // 智能车最低速
        float speedHigh = 1.0;      // 智能车最高速
        float speedAI = 1.0;        // ai识别速度
        float speedCorners = 1.0;   // 贴弯速度
        float speedcoiled = 1.0;    // 连续弯道速度

        float speed_and_angle_k1 = 0.0;     // 速度方向耦合方程一次项
        float speed_and_angle_k2 = 0.0;     // 速度方向耦合方程二次项

        float runP1 = 0.9;          // 一阶比例系数：直线控制量
        float runP2 = 0.018;        // 二阶比例系数：弯道控制量
        float runP3 = 0.0;          // 三阶比例系数：弯道控制量
        float turnP = 3.5;          // 一阶比例系数：转弯控制量
        float turnD = 3.5;          // 一阶微分系数：转弯控制量

        float Kp = 1000.0;
        float Ki = 0.0;
        float Kd = 0.0;
        float Kv = 0.0;

        float Angle_Kp = 0.0;
        float Angle_Ki = 0.0;
        int   dynamic_Mid_low = 140;
        int   dynamic_Mid_high = 180;
        float Angle_target = 0.1;

        float K_K_limit = 0.0;              // 前馈控制斜率死区
        float K_foreword = 0.0;             // 前馈控制系数

        int Control_Mid = 160;              // 控制中线
        int Control_Down_set = 20;          // 图像近处点界限
        int Control_Up_set = 190;           // 图像远处点界限
        int Control_foreword_down = 160;    // 前瞻下限
        int Control_foreword_up = 190;      // 前瞻上限
        float ki_down_out_max;              // 近处点积分项限幅
        float Kp_dowm = 1.0;                // 近处点pi控制kp
        float Ki_down = 0.1;                // 近处点pi控制ki
        float Line_compensation_coefficient;// 线偏前馈补偿系数

        uint16_t rowCutUp = 30;     // 图像顶部切行
        uint16_t rowCutBottom = 10; // 图像顶部切行

        bool Debug = false;
        bool Button = false;
        bool SaveImage = false;
        bool CloseLoop = true;
        bool GarageEnable = false;   // 出入库使能
        bool RingEnable = false;     // 环岛使能
        bool CrossEnable = true;     // 十字使能
        bool StopEnable = false;     // 冲出赛道停止使能
        bool BridgeEnable = false;
        bool SlowzoneEnable = false;
        bool DepotEnable = false;
        bool FarmlandEnable = false;
        string pathModel = "res/model/yolov3_mobilenet_v1";
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, speedLow, speedHigh, speedAI, speedCorners, speedcoiled, speed_and_angle_k1, speed_and_angle_k2, 
            runP1, runP2, runP3, turnP, turnD,Control_Mid, Control_Down_set, Control_Up_set, ki_down_out_max, Kp_dowm, 
            Ki_down, Line_compensation_coefficient, Kp, Ki, Kd, Kv, Angle_Kp, Angle_Ki, dynamic_Mid_low, dynamic_Mid_high, 
            Angle_target, rowCutUp, rowCutBottom,  Debug, Button, SaveImage, CloseLoop, GarageEnable, RingEnable, CrossEnable, 
            StopEnable, BridgeEnable, SlowzoneEnable, DepotEnable, FarmlandEnable, pathModel, K_foreword, Control_foreword_up,
            Control_foreword_down, K_K_limit); // 添加构造函数
    };


    /**********模糊控制的相关参数定义**************/
    struct dis{
        int num[2][2];
    };
    dis arr1;
    dis arr2;

    Params      params;                         // 读取控制参数
    uint16_t    servoPwm = PWMSERVOMID;         // 舵机打角     
    float       compensation_error = 0;         // 线偏前馈
    float       error = 0;                      // 线偏
    float       motorSpeed  = 1.0;              // 发送给电机的速度
    float       Line_offset_mid;                // 打角区域线偏 
    float       Line_offset_up;                 // 远处前瞻点线偏
    float       servoPWM_compensate = 0;        // 近处点前馈
    int         line_Mid_Num = 0;               // 控制打角点个数
    int         line_Up_Num = 0;                // 远处点个数

    // 模糊控制，用于速度控制
    float       Arr_Fuzzy_Error[7] = {-60,-40,-20,0,20,40,60};
    float       Arr_Fuzzy_dError[7] = {-60,-40,-20,0,20,40,60};
    float       Arr_Fuzzy_Speed[7][7] = {0.75,0.75,0.75,0.76,0.77,0.78,0.80,
                                         0.75,0.76,0.76,0.76,0.80,0.80,0.80,
                                         0.75,0.77,0.80,0.95,0.85,0.80,0.76,
                                         0.76,0.80,0.90,1.00,0.90,0.80,0.77,
                                         0.76,0.80,0.85,0.95,0.80,0.77,0.75,
                                         0.80,0.80,0.80,0.76,0.76,0.76,0.76,
                                         0.80,0.78,0.77,0.76,0.75,0.75,0.75};


    /**********角偏线偏串级相关参数定义**************/
    double      Angle_rad = 0.0;
    double      CenterLine_k = 0.0;
    double      Slope_previewPoint = 0.0f;
    double      Mid_line = 160.0;               //角偏环输出


    // /**
    //  * @brief 变加速控制
    //  * @param up_speed_enable 加速使能
    //  * @param control 小车控制类
    //  */
    // void speedController(bool up_speed_enable, ControlCenterCal control) 
    // {
    //     // 控制率，在符合加速条件，并且一段时间后，开始加速，不然低速跑
    //     uint8_t controlLow = 0;             // 速度控制计数器，限幅最低阈值
    //     uint8_t controlspeedCorners = 3;    // 贴弯控制率，3帧图片，就加速
    //     uint8_t controlMid = 3;             // 速度控制计数器，3帧图片就提速
    //     uint8_t controlHigh = 10;           // 速度控制计数器，限幅最高阈值

    //     if(up_speed_enable) // 加速使能
    //     {
    //         // 连续转弯，慢速行驶
    //         if (control.style == "RIGHTCC" || control.style == "LEFTCC")
    //         {
    //             //发送的电机速度，最低速
    //             motorSpeed = params.speedcoiled;
    //             //变速计数器清零
    //             counterShift = controlLow;
    //             return;
    //         }
    //         // 急转弯中，不单边
    //         else if (control.style == "RIGHT" || control.style == "LEFT")     
    //         {
    //             //发送的电机速度，最低速
    //             motorSpeed = params.speedCorners;
    //             return;
    //         }
    //         // 急转弯中，且单边，说明贴线行驶，可加速
    //         else if (control.style == "RIGHT_D" || control.style == "LEFT_D")     
    //         {
    //             //发送的电机速度，最低速
    //             motorSpeed = params.speedLow;
    //             return;
    //         }

    //         // 直道
    //         else if(control.style == "STRIGHT")
    //         {
    //             if (abs(control.sigmaCenter) < 100.0)
    //             {
    //                 //符合加速条件，开始计数
    //                 counterShift++;
    //                 //在符合条件下，达到计数长度上限
    //                 if (counterShift > controlHigh)
    //                     counterShift = controlHigh;

    //                 //判断是加速还是减速
    //                 if (counterShift > controlMid)
    //                     motorSpeed = params.speedHigh;
    //                 else
    //                     motorSpeed = params.speedLow;
    //             }
    //             else    //控制线比较弯曲，并且持续一段时间，减速
    //             {
    //                 counterShift--;
    //                 //在符合条件下，达到计数长度下限
    //                 if (counterShift < controlLow)
    //                     counterShift = controlLow;

    //                 //判断是加速还是减速
    //                 if (counterShift > controlMid)
    //                     motorSpeed = params.speedHigh;
    //                 else
    //                     motorSpeed = params.speedLow;
    //             }
    //         }
    //     }
    //     else    //没有任何操作，最低速跑车
    //     {
    //         counterShift = controlLow;
    //         motorSpeed = params.speedLow;
    //     }
    // }


    /**
     * @brief 基于模糊控制的变加速控制
     * @param up_speed_enable 加速使能
     * @param control 小车控制类
     */
    void speedController(ControlCenterCal control)
    {
        // 记录前一次的偏差
        static float errorLast = 0;                                     //定义上一次的偏差
        float derror = error - errorLast;                               //derror
        errorLast = error;                                              //上一次的偏差

        float **arr_fuzzy_error = (float **)malloc(2 * sizeof(float *));
        for (int i = 0; i < 2; i++)
        {
            arr_fuzzy_error[i] = (float *)malloc(2 * sizeof(float));
        }
        arr_fuzzy_error = Get_FuzzyRule(error);
        
        float **arr_fuzzy_derror = (float **)malloc(2 * sizeof(float *));
        for (int i = 0; i < 2; i++)
        {
            arr_fuzzy_derror[i] = (float *)malloc(2 * sizeof(float));
        }
        arr_fuzzy_derror = Get_FuzzyRule(derror);

        // 速度比例系数，急弯系数小，最大值为1
        float speed_k = Arr_Fuzzy_Speed[(int)arr_fuzzy_derror[0][0]][(int)arr_fuzzy_error[0][0]] * arr_fuzzy_error[1][0] * arr_fuzzy_derror[1][0] +      //error和derror都小值
                        Arr_Fuzzy_Speed[(int)arr_fuzzy_derror[0][0]][(int)arr_fuzzy_error[0][1]] * arr_fuzzy_error[1][1] * arr_fuzzy_derror[1][0] +      //error大值、derror小值
                        Arr_Fuzzy_Speed[(int)arr_fuzzy_derror[0][1]][(int)arr_fuzzy_error[0][0]] * arr_fuzzy_error[1][0] * arr_fuzzy_derror[1][1] +      //error小值、derror大值
                        Arr_Fuzzy_Speed[(int)arr_fuzzy_derror[0][1]][(int)arr_fuzzy_error[0][1]] * arr_fuzzy_error[1][1] * arr_fuzzy_derror[1][1];       //error和derror都大值

        // 实际给定速度
        motorSpeed = params.speedHigh * speed_k;

        for(int i=0;i<2;i++)
        {
            delete []arr_fuzzy_error[i];
        }
        delete []arr_fuzzy_error;

        for(int i=0;i<2;i++)
        {
            delete []arr_fuzzy_derror[i];
        }
        delete []arr_fuzzy_derror;
    }


    /**
     * @brief 基于角偏的变加速控制
     * @param control 小车控制类
     */
    void speedControl(ControlCenterCal control) 
    {
        if(control.centerEdge_yh.size() < 1){
            return;
        }

        Slope_previewPoint = perspectiveTangentSlope(control.centerEdge_yh, control.centerEdge_yh.size()*0.8, 3);

        double speed = 0.8f;
        double y_offset = ROWSIMAGE - control.centerEdge_yh[0].x - params.rowCutBottom;
        double x_offset = abs(control.centerEdge_yh[0].y - Mid_line);
        if(y_offset > 60)
            y_offset = 60;

        // speed = params.speedHigh - (params.speedHigh - params.speedLow) * atan(abs(CenterLine_k)) - y_offset / 100 - x_offset / 200;
        speed = params.speedHigh - (params.speedHigh - params.speedLow) * atan(abs(CenterLine_k)) - params.speedHigh * 0.1 * atan(min(abs(Slope_previewPoint), 4.0)) - x_offset / 200;
        // 最低速限制
        if(speed < 0.8)
            speed = 0.8;

        //加速限幅
        if(speed - motorSpeed > 0.05f)
            motorSpeed += 0.05f;
        else 
            motorSpeed = speed;
    }


    // /**
    //  * @brief 角度控制器
    //  * @param controlCenter 智能车控制中心
    //  */
    // void Angle_Controller(ControlCenterCal controlCenter)
    // {
    //     // 线偏计算
    //     for(int i=0;i<controlCenter.centerEdge.size();i++)
    //     {
    //         if(controlCenter.centerEdge[i].x > (240 - params.Control_Down_set))
    //         {
    //             Line_offset_down += (controlCenter.centerEdge[i].y - params.Control_Mid);
    //             line_down_Num++;
    //         }
    //         else if(controlCenter.centerEdge[i].x <= (240 - params.Control_Down_set) && controlCenter.centerEdge[i].x > (240 - params.Control_Up_set))
    //         {
    //             Line_offset_mid += (controlCenter.centerEdge[i].y - params.Control_Mid);
    //             line_Mid_Num++;
    //         }
    //         else if(controlCenter.centerEdge[i].x <= (240 - params.Control_Up_set))
    //         {
    //             Line_offset_up += (controlCenter.centerEdge[i].y - params.Control_Mid);
    //             line_Up_Num++;
    //         }
    //     }

    //     // 线偏值具体计算
    //     if(line_down_Num != 0)
    //     {
    //         Line_offset_down = Line_offset_down / line_down_Num;
    //         line_down_Num = 0;
    //     }
    //     else
    //         Line_offset_down = 0;
        
    //     if(line_Mid_Num != 0)
    //     {
    //         Line_offset_mid = Line_offset_mid / line_Mid_Num;
    //         line_Mid_Num = 0;
    //     }
    //     else
    //         Line_offset_mid = 0;

    //     if(line_Up_Num != 0)
    //     {
    //         Line_offset_up = Line_offset_up / line_Up_Num;
    //         line_Up_Num = 0;
    //     }
    //     else
    //         Line_offset_up = 0;

    //     // 路劲规划偏置
    //     if(Line_offset_up > 0)
    //         Line_offset_up = Line_offset_up - params.Control_Skew;
    //     else if(Line_offset_up < 0)
    //         Line_offset_up = Line_offset_up + params.Control_Skew;

    //     // 图像前瞻丢线比例
    //     float rate_lost_line = 0.0;
    //     if(controlCenter.centerEdge.size() != 0)
    //         rate_lost_line = 1.0 - controlCenter.centerEdge[controlCenter.centerEdge.size() - 1].x / 240;
    //     else
    //         rate_lost_line = 0;

    //     // 记录前一次的偏差
    //     static float errorLast = 0;                                     //定义上一次的偏差
    //     float derror = Line_offset_mid - errorLast;                     //derror
    //     errorLast = Line_offset_mid;                                    //上一次的偏差

    //     float **arr_fuzzy_error = (float **)malloc(2 * sizeof(float *));
    //     for (int i = 0; i < 2; i++)
    //     {
    //         arr_fuzzy_error[i] = (float *)malloc(2 * sizeof(float));
    //     }
    //     arr_fuzzy_error = Get_FuzzyRule(Line_offset_mid);
        
    //     float **arr_fuzzy_derror = (float **)malloc(2 * sizeof(float *));
    //     for (int i = 0; i < 2; i++)
    //     {
    //         arr_fuzzy_derror[i] = (float *)malloc(2 * sizeof(float));
    //     }
    //     arr_fuzzy_derror = Get_FuzzyRule(Line_offset_mid);

    //     servoPwm = Arr_Fuzzy_PWM[(int)arr_fuzzy_derror[0][0]][(int)arr_fuzzy_error[0][0]] * arr_fuzzy_error[1][0] * arr_fuzzy_derror[1][0] +      //error和derror都小值
    //                Arr_Fuzzy_PWM[(int)arr_fuzzy_derror[0][0]][(int)arr_fuzzy_error[0][1]] * arr_fuzzy_error[1][1] * arr_fuzzy_derror[1][0] +      //error大值、derror小值
    //                Arr_Fuzzy_PWM[(int)arr_fuzzy_derror[0][1]][(int)arr_fuzzy_error[0][0]] * arr_fuzzy_error[1][0] * arr_fuzzy_derror[1][1] +      //error小值、derror大值
    //                Arr_Fuzzy_PWM[(int)arr_fuzzy_derror[0][1]][(int)arr_fuzzy_error[0][1]] * arr_fuzzy_error[1][1] * arr_fuzzy_derror[1][1];       //error和derror都大值

    //     // 近处点姿态补偿
    //     static float servoPWM_compensate_Iout = 0;
    //     servoPWM_compensate_Iout += params.Ki_down * Line_offset_down;
    //     // 积分项限幅
    //     if(servoPWM_compensate_Iout >= params.ki_down_out_max)
    //         servoPWM_compensate_Iout = params.ki_down_out_max;
    //     else if(servoPWM_compensate_Iout <= -params.ki_down_out_max)
    //         servoPWM_compensate_Iout = -params.ki_down_out_max;
    //     servoPWM_compensate = params.Kp_dowm * Line_offset_down + servoPWM_compensate_Iout;

    //     servoPwm = servoPwm + servoPWM_compensate;

    //     for(int i=0;i<2;i++)
    //     {
    //         delete []arr_fuzzy_error[i];
    //     }
    //     delete []arr_fuzzy_error;

    //     for(int i=0;i<2;i++)
    //     {
    //         delete []arr_fuzzy_derror[i];
    //     }
    //     delete []arr_fuzzy_derror;

    // }


    /**
     * @brief 得到Error隶属度或者dError
     * @param Error 智能车打角偏差
     */
    float** Get_FuzzyRule(float Error)
    {
        float** arr = (float**)malloc(2*sizeof(float*));
        for(int i=0;i<2;i++)
        {
            arr[i] = (float*)malloc(2*sizeof(float));
        }

        for(int i = 0;i < 7;i++)
        {
            if(Error < Arr_Fuzzy_dError[i])
            {
                if(i == 0)
                {
                    arr[0][0] = 0;      //小值
                    arr[0][1] = 0;      //大值
                    arr[1][0] = 0;      //小值的隶属度
                    arr[1][1] = 1;      //大值的隶属度
                    break;
                }
                arr[0][0] = i -1;
                arr[0][1] = i;
                arr[1][0] = (Arr_Fuzzy_dError[i] - Error) / (Arr_Fuzzy_dError[i] - Arr_Fuzzy_dError[i-1]);
                arr[1][1] = 1 - arr[1][0];
                break;
            }

            if(i == 6 && Error > Arr_Fuzzy_dError[i])
            {
                arr[0][0] = 6;      //小值
                arr[0][1] = 6;      //大值
                arr[1][0] = 1;      //小值的隶属度
                arr[1][1] = 0;      //大值的隶属度
                break;
            }
        }

        return arr;

        //释放空间
        for(int i=0;i<2;i++)
        {
            delete []arr[i];
        }
        delete []arr;
    }



    /**
     * @brief 方向控制器
     * @param controlCenter 智能车控制中心
     * @param track 路径
     */
    void Angle_Controller(ControlCenterCal controlCenter, TrackRecognition track, int enum_RoadType = 0)
    {
        /**********线偏均值计算**************/
        Line_offset_mid = 0;
        Line_offset_up = 0;
        vector<POINT> line_perspective = track.line_perspective(controlCenter.centerEdge);  //得到俯视域的中线，算斜率
        // 计算中线给定段的平均值
        for(int i=0;i<controlCenter.centerEdge.size();i++)
        {
            if(controlCenter.centerEdge[i].x <= (ROWSIMAGE - params.Control_foreword_down) && controlCenter.centerEdge[i].x > (ROWSIMAGE - params.Control_foreword_up))
            {
                Line_offset_up += controlCenter.centerEdge[i].y;
                line_Up_Num++;
            }

            if(controlCenter.centerEdge[i].x <= (ROWSIMAGE - params.Control_Down_set) && controlCenter.centerEdge[i].x > (ROWSIMAGE - params.Control_Up_set))
            {
                Line_offset_mid += controlCenter.centerEdge[i].y;
                line_Mid_Num++;
            }
        }


        /**********线偏值具体计算**************/
        if(line_Up_Num != 0)
        {
            Line_offset_up = Line_offset_up / line_Up_Num;
            line_Up_Num = 0;
        }
        else
            Line_offset_up = COLSIMAGE / 2;
        
        if(line_Mid_Num != 0)
        {
            Line_offset_mid = Line_offset_mid / line_Mid_Num;
            line_Mid_Num = 0;
        }
        else
            Line_offset_mid = COLSIMAGE / 2;


        /**********转弯撞线前馈计算**************/
        compensation_error = 0;
        // 左边与中线撞线的点
        if((controlCenter.intersectionLeft.x < 195 && controlCenter.intersectionLeft.x > 100) && controlCenter.intersectionRight.x ==0)
            compensation_error = (controlCenter.intersectionLeft.x - 100) * params.Line_compensation_coefficient;
        // 右边与中线撞线的点
        if((controlCenter.intersectionRight.x < 195 && controlCenter.intersectionRight.x > 100) && controlCenter.intersectionLeft.x ==0)
            compensation_error = -(controlCenter.intersectionRight.x - 100) * params.Line_compensation_coefficient;
        
        if(enum_RoadType == 9)
            compensation_error *= 2;


        /**********角偏控制**************/
        CenterLine_k = track.LeastSquare(line_perspective);
        Slope_previewPoint = perspectiveTangentSlope(controlCenter.centerEdge_yh, controlCenter.centerEdge_yh.size()*0.8, 3);

        float centerline_in_this_function_k = CenterLine_k;
        if((CenterLine_k >= 0 && Slope_previewPoint >= 0) || (CenterLine_k < 0 && Slope_previewPoint < 0))
            centerline_in_this_function_k = abs(Slope_previewPoint) > abs(CenterLine_k) ? Slope_previewPoint : CenterLine_k;
        
        if(abs(CenterLine_k) < params.Angle_target / 2)
            centerline_in_this_function_k = 0;

        // Angle_rad = atan(CenterLine_k);
        // 角偏积分项
        static float Angle_Iout = 0;
        // Angle_Iout += params.Ki_down * CenterLine_k;
        if(enum_RoadType == 1)
        {
            Angle_Iout += params.Ki_down * centerline_in_this_function_k;
        }
        else 
        {
            Angle_Iout = 0;
        }
        // 积分项限幅
        if(Angle_Iout >= params.ki_down_out_max)
            Angle_Iout = params.ki_down_out_max;
        else if(Angle_Iout <= -params.ki_down_out_max) 
            Angle_Iout = -params.ki_down_out_max;
        // 达到目标值动态中线积分项清除
        if(abs(CenterLine_k) <= params.Angle_target)
        {
            Angle_Iout *= 0.5;
        }
        // 动态中线输出值
        if(controlCenter.centerEdge.size() < params.Control_Up_set - params.Control_Down_set)
        {
            Mid_line = params.Control_Mid + (params.Angle_Kp * centerline_in_this_function_k + Angle_Iout);
            // 动态中线限幅
            if(Mid_line <= params.dynamic_Mid_low)
                Mid_line = params.dynamic_Mid_low;
            else if(Mid_line >= params.dynamic_Mid_high)
                Mid_line = params.dynamic_Mid_high;
        }

        /**********线偏控制**************/
        error = Line_offset_mid - Mid_line;                                                                 // 主要控制打角的线偏error值
        int error_foreword = Line_offset_up - Mid_line;                                                     // 前瞻的线偏error值
        if(abs(error) < 5)
            error = 0;
        static float errorLast = 0;

        // 限幅计算
        // if(abs(error - errorLast) > COLSIMAGE / 10) 
        //     error = error > errorLast ? errorLast + COLSIMAGE / 10 : errorLast - COLSIMAGE / 10;

        // 计算动态线偏P值
        params.turnP = abs(error) * abs(error) * params.runP3 + abs(error) * params.runP2 + params.runP1;

        int pwmDiff = 0;
        if(enum_RoadType == 4)
        {
            pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
            errorLast = error;
        }
        else if(enum_RoadType == 1)
        {
            pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;

            /**********加入路径规划，大转角**********/
            if(abs(centerline_in_this_function_k) > params.K_K_limit)
                pwmDiff = pwmDiff - error_foreword * params.K_foreword; 
            errorLast = error;
        }
        else
        {
            pwmDiff = ((error + compensation_error) * params.turnP) + (error - errorLast) * params.turnD;

            /**********加入路径规划，大转角**********/
            if(abs(centerline_in_this_function_k) > params.K_K_limit)
                pwmDiff = pwmDiff - error_foreword * params.K_foreword; 
            errorLast = error;
        }

        // PWM转换
        servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff);
    }


private:
	/**
	 * @brief 在俯视域计算预瞄点的切线斜率
	 *
     * @param line 图像域下的中线
	 * @param index 预瞄点的下标号
	 * @param size 开窗大小
	 * @return 斜率
	 */
    double perspectiveTangentSlope(std::vector<POINT> line, uint16_t index, int size)
    {
        if(line.size() < 5)
            return 0;
            
		// End
		Point2d endIpm = ipm.homography(
			Point2d(line[inRange(line, index-size)].y, line[inRange(line, index-size)].x)); // 透视变换
		POINT p1 = POINT(endIpm.y, endIpm.x);

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(line[inRange(line, index+size)].y, line[inRange(line, index+size)].x)); // 透视变换
		POINT p0 = POINT(startIpm.y, startIpm.x);

        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        if(dx == 0)
            return 10.0;
        else
            return dy / dx;
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/motion_new.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good()) 
        {
            std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try 
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e) 
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        motorSpeed = params.speedLow;
        // cout << "--- runP1:" << params.runP1 << " | runP2:" << params.runP2
        //     << " | runP3:" << params.runP3 << endl;
        // cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
        cout << "--- speedLow:" << params.speedLow
            << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
    }
};
