#pragma once
/**
 * @file slowzone_detection.cpp
 * @author PXX
 * @brief 慢行区
 * @version 0.1
 * @date 2022-05-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognize/track_recognition.cpp"

using namespace cv;
using namespace std;

class SlowZoneDetection
{
public:
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        counterDisable = 0;     // 标志失效计数
        counterSession = 0;     // 图像场次计数器
        counterRec = 0;         // 加油站标志检测计数器
        counterFild = 0;
        slowZoneEnable = false; // 慢行区使能标志
    }

    bool slowZoneDetection(TrackRecognition &track, vector<PredictResult> predict)
    {
        // if(counterFild < 30)
        // {
        //     counterFild++;//屏蔽计数器
        //     return false;
        // }

        // 检测标志
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_BUMP || predict[i].label == LABEL_PIG)
            {
                counterRec++;
                break;
            }
        }

        if (counterRec)
        {
            counterSession++;
            if (counterRec >= 4 && counterSession < 8)
            {
                counterRec = 0;
                counterSession = 0;
                counterDisable = 0;
                slowZoneEnable = true; // 检测到慢行区
                return true;
            }
            else if (counterSession >= 8)
            {
                counterRec = 0;
                counterSession = 0;
            }
        }

        // 进入慢行区
        if (slowZoneEnable)
        {
            counterDisable++;
            if (counterDisable > 25)
            {
                counterRec = 0;
                counterDisable = 0;
                counterFild = 0;
                slowZoneEnable = false;
                return false;
            }

            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2) // 切行，防止错误前瞻引发转向
            {
                track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() * 0.8);
                track.pointsEdgeRight.resize(track.pointsEdgeRight.size() * 0.8);
            }

            return true;
        }

        return false;
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, Mat &image)
    {
        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        if (slowZoneEnable)
            putText(image, "slowZoneEnable", Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    uint16_t counterDisable = 0; // 标志失效计数
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 检测计数器
    uint16_t counterFild = 0;    // 屏蔽计数器
    bool slowZoneEnable = false; // 慢行区使能标志
};