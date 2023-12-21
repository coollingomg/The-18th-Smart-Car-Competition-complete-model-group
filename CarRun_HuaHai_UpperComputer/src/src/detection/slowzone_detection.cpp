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
    SlowZoneDetection()
    {
        loadParams();
    }
    ~SlowZoneDetection() {}
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
        _slowZoneEnable = false; // 慢行区使能标志
    }

    void slowZoneCheck(vector<PredictResult> predict)
    {
        if(counterFild < 30)
        {
            counterFild++;//屏蔽计数器
            return;
        }

        // 检测标志
        if(!_slowZoneEnable)
        {
            for (int i = 0; i < predict.size(); i++)
            {
                if ((predict[i].label == LABEL_BUMP && predict[i].y + predict[i].height / 2 > 70) || (predict[i].label == LABEL_PIG && predict[i].y + predict[i].height / 2 > 70))
                {
                    counterRec++;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= params.SlowzoneCheck && counterSession < params.SlowzoneCheck * 2 + 1)
                {
                    counterRec = 0;
                    counterSession = 0;
                    counterDisable = 0;
                    _slowZoneEnable = true; // 检测到慢行区
                    return;
                }
                else if (counterSession >= params.SlowzoneCheck * 2 + 1)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
        }
    }

    bool slowZoneDetection(TrackRecognition &track)
    {
        // 进入慢行区
        if (_slowZoneEnable)
        {
            counterDisable++;
            if (counterDisable > params.ExitFrameCnt)
            {
                counterRec = 0;
                counterDisable = 0;
                counterFild = 0;
                _slowZoneEnable = false;
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

        if (_slowZoneEnable)
            putText(image, "_slowZoneEnable", Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/slowzone.json";
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
    }

private:
    uint16_t counterDisable = 0; // 标志失效计数
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 检测计数器
    uint16_t counterFild = 0;    // 屏蔽计数器
    bool _slowZoneEnable = false; // 慢行区使能标志
    /**
     * @brief 动物出没区域核心参数
     */
    struct Params 
    {
        uint16_t SlowzoneCheck = 4;
        uint16_t CheckRow = 30;
        uint16_t ExitFrameCnt = 20;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, SlowzoneCheck, CheckRow, ExitFrameCnt); // 添加构造函数
    };
    Params params;

};