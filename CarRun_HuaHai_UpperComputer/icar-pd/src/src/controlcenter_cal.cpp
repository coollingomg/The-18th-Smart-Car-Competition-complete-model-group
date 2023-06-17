#pragma once
/**
 * @file controlcenter_cal.cpp
 * @author your name (you@domain.com)
 * @brief 智能车控制中心计算
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <random>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "recognize/track_recognition.cpp"

using namespace cv;
using namespace std;

class ControlCenterCal
{
public:
    int controlCenter;           // 智能车控制中心（0~320）
    vector<POINT> centerEdge;    // 赛道中心点集
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差
    string style = "";           // 赛道类型

    /**
     * @brief 控制中心计算
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */
    void controlCenterCal(TrackRecognition &track)
    {
        sigmaCenter = 0;                    // 中心点集的方差
        controlCenter = COLSIMAGE / 2;      // 智能车控制中心（0~320）
        centerEdge.clear();
        vector<POINT> v_center(4);          // 三阶贝塞尔曲线
        style = "STRIGHT";                  // 赛道类型，定义为直道

        double miu = ROWSIMAGE / 2;                                     //正太分布函数参数值
        double singema = miu / 3;                                       //正太分布函数参数值

        // 边缘斜率标准差 重计算（边缘修正之后）
        track.stdevLeft = track.stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE);
        track.stdevRight = track.stdevEdgeCal(track.pointsEdgeRight, ROWSIMAGE);

        // 连续弯道判断
        if (track.stdevLeft > 280 && track.stdevRight < 210)            // 右转连续
        {
            // 有效边缘行获取
            //coiled_validRowsCal_right(track.pointsEdgeLeft, track.pointsEdgeRight);
            // 赛道类型，定义为右急转弯
            style = "RIGHTCC";
        }
        else if(track.stdevRight > 280 && track.stdevLeft < 210)        // 左转连续
        {
            // 有效边缘行获取
            //coiled_validRowsCal_left(track.pointsEdgeLeft, track.pointsEdgeRight);
            // 赛道类型，定义为左急转弯
            style = "LEFTCC";
        }

        // 边缘有效行优化，左边方差大右边方差小；或者左边小右边大，就是转弯。将边缘没用的边线优化
        if ((track.stdevLeft < 65 && track.stdevRight > 65) || (track.stdevLeft > 65 && track.stdevRight < 65))
        {
            validRowsCal(track.pointsEdgeLeft, track.pointsEdgeRight); // 边缘有效行计算
            track.pointsEdgeLeft.resize(validRowsLeft);
            track.pointsEdgeRight.resize(validRowsRight);
        }

        /****补丁****/
        if((track.stdevLeft == 0 && track.stdevRight > 50) && track.pointsEdgeLeft.size() < 90
            && track.pointsEdgeLeft[0].y == track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y)
        {
            track.pointsEdgeLeft.resize(0);
        }
        else if((track.stdevRight == 0 && track.stdevLeft > 50) && track.pointsEdgeRight.size() < 90
            && track.pointsEdgeRight[0].y == track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y)
        {
            track.pointsEdgeRight.resize(0);
        }

        // 通过双边缘有效点的差来判断赛道类型，使用双段三阶贝塞尔拟合中线
        if (track.pointsEdgeLeft.size() > 45 && track.pointsEdgeRight.size() > 45) 
        {
            if(style == "RIGHTCC" || style == "LEFTCC")
            {
                //中线拟合去掉上面1/4行，处理左边线
                if(track.pointsEdgeLeft.size() > 0)
                {
                    if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x <= ROWSIMAGE / 3)
                    {
                        int cnt = track.pointsEdgeLeft.size() - 1;
                        // 去除图片上面固定行的点集
                        for(int i = track.pointsEdgeLeft.size() - 1; track.pointsEdgeLeft[i].x <= ROWSIMAGE / 3; i--)
                        {
                            cnt--;
                        }
                        if(cnt <= 0)
                            cnt = 0;
                        track.pointsEdgeLeft.resize(cnt);
                    }
                }

                //中线拟合去掉上面1/4行，处理右边线
                if(track.pointsEdgeRight.size() > 0)
                {
                    if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x <= ROWSIMAGE / 3)
                    {
                        int cnt = track.pointsEdgeRight.size() - 1;
                        // 去除图片上面固定行的点集
                        for(int i = track.pointsEdgeRight.size() - 1; track.pointsEdgeRight[i].x <= ROWSIMAGE / 3; i--)
                        {
                            cnt--;
                        }
                        if(cnt <= 0)
                            cnt = 0;
                        track.pointsEdgeRight.resize(cnt);
                    }
                }

                //中线曲线拟合
                centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);
            }
            else
            {
                //中线曲线拟合
                centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);

                //类型判断为直线
                style = "STRIGHT";
            }
        }
        else if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() == 0) // 左单边
        {
            v_center[0] = {(track.pointsEdgeLeft[0].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeLeft[0].y + COLSIMAGE - 1) / 2};

            v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + COLSIMAGE - 1) / 2};

            v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + COLSIMAGE - 1) / 2};

            v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y + COLSIMAGE - 1) / 2};

            centerEdge = Bezier(0.02, v_center);

            style = "RIGHT_D";
        }
        else if (track.pointsEdgeLeft.size() == 0 && track.pointsEdgeRight.size() > 4) // 右单边
        {
            v_center[0] = {(track.pointsEdgeRight[0].x + ROWSIMAGE - 1) / 2,
                            (track.pointsEdgeRight[0].y) / 2};

            v_center[1] = {(track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

            v_center[2] = {(track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

            v_center[3] = {(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x + ROWSIMAGE - 1) / 2,
                           (track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y) / 2};

            centerEdge = Bezier(0.02, v_center);

            style = "LEFT_D";
        }
        // 左转弯，但不单边
        else if ((track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() <= 45) ||
                 (track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft[0].x - track.pointsEdgeRight[0].x > ROWSIMAGE / 2))
        {
            //中线曲线拟合
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);

            //类型判断
            style = "RIGHT";
        }
        // 右转弯，但不单边
        else if ((track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() <= 45) ||
                 (track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x > ROWSIMAGE / 2))
        {
            //中线曲线拟合
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);

            //类型判断
            style = "LEFT";
        }

        // 加权控制中心计算
        double controlNum = 0;
        double controlCenter_Calculate = 0.0;
        for (auto p : centerEdge)
        {
            // //传统分割区域，经验参数权限比重控制
            // if (p.x < ROWSIMAGE / 5)
            // {
            //     controlNum += (p.x * 0.34);
            //     controlCenter_Calculate += p.y * (p.x * 0.34);
            // }
            // else if(p.x < ROWSIMAGE * 2 / 5 && p.x >= ROWSIMAGE / 5)
            // {
            //     controlNum += (0.5 * p.x - 9.6);
            //     controlCenter_Calculate += p.y * (0.5 * p.x - 9.6);
            // }
            // else if(p.x < ROWSIMAGE * 3 / 5 && p.x >= ROWSIMAGE * 2 / 5)
            // {
            //     controlNum += (0.68 * p.x - 10);
            //     controlCenter_Calculate += p.y * (0.68 * p.x - 10);
            // }
            // else if(p.x < ROWSIMAGE * 4 / 5 && p.x >= ROWSIMAGE * 3 / 5)
            // {
            //     controlNum += (300 - p.x);
            //     controlCenter_Calculate += p.y * (300 - p.x);
            // }
            // else if(p.x < ROWSIMAGE && p.x >= ROWSIMAGE * 4 / 5)
            // {
            //     controlNum += (150 - p.x * 0.8);
            //     controlCenter_Calculate += p.y * (150 - p.x * 0.8);
            // }

            //经验参数曲线拟合权限控制法,加上经验比例参数->计算量比较大，而且效果一般
            if (p.x < ROWSIMAGE / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 0.35;
                controlCenter_Calculate += p.y * temp * 0.35;
            }
            else if(p.x < ROWSIMAGE * 2 / 5 && p.x >= ROWSIMAGE / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 0.96;
                controlCenter_Calculate += p.y * temp * 0.96;
            }
            else if(p.x < ROWSIMAGE * 3 / 5 && p.x >= ROWSIMAGE * 2 / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 1.21;
                controlCenter_Calculate += p.y * temp * 1.21;
            }
            else if(p.x < ROWSIMAGE * 4 / 5 && p.x >= ROWSIMAGE * 3 / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 2.08;
                controlCenter_Calculate += p.y * temp * 2.08;
            }
            else if(p.x < ROWSIMAGE && p.x >= ROWSIMAGE * 4 / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 0.45;
                controlCenter_Calculate += p.y * temp * 0.45;
            }
        }
        if (controlNum > 0)
        {
            controlCenter = (int)(controlCenter_Calculate / controlNum);
        }

        //限制幅值
        if (controlCenter > COLSIMAGE)
            controlCenter = COLSIMAGE;
        else if (controlCenter < 0)
            controlCenter = 0;

        // 控制率计算
        if (centerEdge.size() > 20)
        {
            vector<POINT> centerV;
            int filt = centerEdge.size() / 5;
            for (int i = filt; i < centerEdge.size() - filt; i++) // 过滤中心点集前后1/5的诱导性
            {
                centerV.push_back(centerEdge[i]);
            }
            sigmaCenter = sigma(centerV);
        }
        else
            sigmaCenter = 1000;
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param centerImage 需要叠加显示的图像
     */
    void drawImage(TrackRecognition track, Mat &centerImage)
    {
        // 赛道边缘绘制
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制中心点集
        for (int i = 0; i < centerEdge.size(); i++)
        {
            circle(centerImage, Point(centerEdge[i].y, centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
        }

        // 绘制加权控制中心：方向
        Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
        rectangle(centerImage, rect, Scalar(0, 0, 255), CV_FILLED);

        // 详细控制参数显示
        int dis = 20;
        string str;
        putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型

        str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右

        str = "Center: " + formatDoble2String(sigmaCenter, 2);
        putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差

        putText(centerImage, to_string(controlCenter), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1); // 中心
    }

private:

    /**
     * @brief  控制函数曲线拟合
     * @param  _highly_control_point 最高权限控制点
     * @return double curve_fitting_putout 拟合参数值
     */
    double curve_fitting(int _highly_control_point)
    {
        //定义曲线拟合参数输出值
        double curve_fitting_putout = 1;

        int x = _highly_control_point;

        //计算拟合参数值
        curve_fitting_putout = (log((double)(ROWSIMAGE / 2)) -log((double)ROWSIMAGE)) / (log((double)x) -log((double)ROWSIMAGE));

        return curve_fitting_putout;
    }

    /**
     * @brief  得到拟合参数，采用次方函数拟合
     * @param  _highly_control_point 最高权限控制点
     * @return double curve_fitting_output_value 拟合曲线输出值
     */
    double curve_fitting_output(int point_x)
    {
        //定义曲线拟合输出值
        double curve_fitting_output_value = 0;

        //获取拟合曲线参数
        double m = curve_fitting(highly_control_point);
        //计算拟合曲线输出值
        curve_fitting_output_value = pow((double)ROWSIMAGE, (1 - m)) * pow((double)point_x, m);

        return curve_fitting_output_value;
    }

    /**
     * @brief  正太分布函数
     * @param  x 函数x的值
     * @param  func_miu 系数μ
     * @param  func_singema 系数singema
     * @return 高斯分布函数输出值
     */
    double normal_pdf(double x, double func_miu, double func_singema)
    {
        //根号下2*pi分之一的值
        static const float inv_sqrt_2pi = 0.3989422804014327;
        //计算次方系数
        double a = (x / func_miu) / func_singema;
    
        return inv_sqrt_2pi / func_singema * std::exp(-0.5 * a * a) * AMPLIFICATION_FACTOR;
    }

    /**
     * @brief 搜索边线起点（左下）
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        // 计数器
        uint16_t counter = 0;

        // 寻找左边起始点
        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
        {
            if (pointsEdgeLeft[i].y >= 3)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }
        return 0;
    }

    /**
     * @brief 搜索边线起点（右下）
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        // 计数器
        uint16_t counter = 0;

        // 寻找右边起始点
        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) 
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }
        return 0;
    }

    /**
     * @brief 搜索边线突变点（右下）
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t search_Mutation_point_right(vector<POINT> pointsEdgeRight)
    {
        bool start = false;
        uint16_t rowBreakRight = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size(); i++) // 寻找右边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 3 || i >= 30)
                start = true;
            if(start)
            {
                if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRight].y)
                {
                    rowBreakRight = i;
                    counter = 0;
                }
                else if (pointsEdgeRight[i].y >= pointsEdgeRight[rowBreakRight].y) // 突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakRight;
                }
            }
        }
        return rowBreakRight;
    }

    /**
     * @brief 搜索边线突变点（左下）
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t search_Mutation_point_left(vector<POINT> pointsEdgeLeft)
    {
        bool start = false;
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size(); i++) // 寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > 3 || i >= 30)
                start = true;
            if(start)
            {
                if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y)
                {
                    rowBreakLeft = i;
                    counter = 0;
                }
                else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakLeft;
                }
            }
        }
        return rowBreakLeft;
    }

    /**
     * @brief 双段三阶贝塞尔拟合中线
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     * @return uint16_t
     */
    vector<POINT> bezier_curve_fitting(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        vector<POINT> su_centerEdge;        // 双阶贝塞尔曲中点计算点集
        vector<POINT> centerEdge_func;      // 赛道中心点集

        vector<POINT> v_center(4);          // 三阶贝塞尔曲线
        vector<POINT> center_point(2);      // 中点寻找容器
        POINT v_midpoint;                   // 分段贝塞尔的中点

        //清空点集
        su_centerEdge.clear();
        centerEdge_func.clear();

        //寻找拟合曲线的点集
        v_center[0] = {(pointsEdgeLeft[0].x + pointsEdgeRight[0].x) / 2, (pointsEdgeLeft[0].y + pointsEdgeRight[0].y) / 2};

        v_center[1] = {(pointsEdgeLeft[pointsEdgeLeft.size() / 7].x + pointsEdgeRight[pointsEdgeRight.size() / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() / 7].y + pointsEdgeRight[pointsEdgeRight.size() / 7].y) / 2};

        v_center[2] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 2 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 2 / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() * 2 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 2 / 7].y) / 2};

        center_point[0] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 3 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 3 / 7].x) / 2,
                            (pointsEdgeLeft[pointsEdgeLeft.size() * 3 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 3 / 7].y) / 2};
        center_point[1] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 4 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 4 / 7].x) / 2,
                            (pointsEdgeLeft[pointsEdgeLeft.size() * 4 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 4 / 7].y) / 2};

        v_center[3] = {(center_point[0].x + center_point[1].x) / 2,
                        (center_point[0].y + center_point[1].y) / 2};

        centerEdge_func = Bezier(0.03, v_center);

        v_center[0] = {(center_point[0].x + center_point[1].x) / 2,
                        (center_point[0].y + center_point[1].y) / 2};

        v_center[1] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 5 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 5 / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() * 5 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 5 / 7].y) / 2};

        v_center[2] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 6 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 6 / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() * 6 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 6 / 7].y) / 2};

        v_center[3] = {(pointsEdgeLeft[pointsEdgeLeft.size() - 1].x + pointsEdgeRight[pointsEdgeRight.size() - 1].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y + pointsEdgeRight[pointsEdgeRight.size() - 1].y) / 2};

        su_centerEdge = Bezier(0.03, v_center);

        for(int i = 0; i < su_centerEdge.size(); i++)
        {
            centerEdge_func.push_back(su_centerEdge[i]);
        }

        //返回中心点集
        return centerEdge_func;
    }

    /**
     * @brief 边缘有效行计算：左/右
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void validRowsCal(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        int counter = 0;
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakLeft = searchBreakLeftDown(pointsEdgeLeft);                                           // 左边缘上升拐点->起始点
            uint16_t rowBreakRight = searchBreakRightDown(pointsEdgeRight);                                        // 右边缘上升拐点->起始点

            //截断多余的左边缘贴图片的边缘线
            if (pointsEdgeRight[pointsEdgeRight.size() - 1].y < COLSIMAGE / 2 && rowBreakRight - rowBreakLeft > 5) // 左弯道
            {
                if (pointsEdgeLeft.size() > rowBreakRight) // 左边缘有效行重新搜索
                {
                    for (int i = rowBreakRight; i < pointsEdgeLeft.size(); i++)
                    {
                        if (pointsEdgeLeft[i].y < 5)
                        {
                            counter++;
                            if (counter >= 3)
                            {
                                pointsEdgeLeft.resize(i - 3);
                            }
                        }
                        else
                            counter = 0;
                    }
                }
            }

            //截断多余的右边缘贴图片的边缘线
            else if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y > COLSIMAGE / 2 && rowBreakLeft - rowBreakRight > 5) // 右弯道
            {
                if (pointsEdgeRight.size() > rowBreakLeft) // 右边缘有效行重新搜索
                {
                    for (int i = rowBreakLeft; i < pointsEdgeRight.size(); i++)
                    {
                        if (pointsEdgeRight[i].y > COLSIMAGE - 3)
                        {
                            counter++;
                            if (counter >= 3)
                            {
                                pointsEdgeRight.resize(i - 3);
                            }
                        }
                        else
                            counter = 0;
                    }
                }
            }
        }

        // 左边有效行
        validRowsLeft = 0;
        if (pointsEdgeLeft.size() > 1)
        {
            for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeLeft[i].y > 5 && pointsEdgeLeft[i - 1].y >= 5)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsEdgeLeft[i].y < 5 && pointsEdgeLeft[i - 1].y >= 5)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

        // 右边有效行
        validRowsRight = 0;
        if (pointsEdgeRight.size() > 1)
        {
            for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeRight[i].y <= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
                if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
            }
        }
    }

    /**
     * @brief 连续转弯边缘有效行计算：右转
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void coiled_validRowsCal_right(vector<POINT> &pointsEdgeLeft, vector<POINT> &pointsEdgeRight)
    {
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakLeft = search_Mutation_point_left(pointsEdgeLeft);                                           // 左边缘上升突变点

            //截断多余的左边缘贴图片的边缘线
            pointsEdgeLeft.resize(rowBreakLeft);

            //两边平齐
            pointsEdgeRight.resize(pointsEdgeLeft.size());

            //改变类型
            style = "RIGHT_CC";
        }
    }

    /**
     * @brief 连续转弯边缘有效行计算：左转
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void coiled_validRowsCal_left(vector<POINT> &pointsEdgeLeft, vector<POINT> &pointsEdgeRight)
    {
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakRight = search_Mutation_point_right(pointsEdgeRight);                                        // 右边缘上升突变点

            //截断多余的右边缘贴图片的边缘线
            pointsEdgeRight.resize(rowBreakRight);

            //两边平齐
            pointsEdgeLeft.resize(pointsEdgeRight.size());

            //改变类型
            style = "LEFT_CC";
        }
    }
};