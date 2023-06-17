#pragma once
/**
* @file slope_detection.cpp
* @author Pxx
* @brief 坡道（桥）路径处理
* @version 0.1
* @date 2023-06-03
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

class BridgeDetection
{
public:
    BridgeDetection()
    {
        loadParams();
    }
    /**
     * @brief 桥梁区域核心参数
     */
    struct Params 
    {
        uint16_t BridgeCheck = 3;
        uint16_t rowCheck = 90;
        float SpeedUp = 1.0;
        uint16_t ExitFrameCnt = 30;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, BridgeCheck, rowCheck, SpeedUp, ExitFrameCnt); // 添加构造函数
    };

    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        counterSession = 0;   // 图像场次计数器
        counterRec = 0;       // 加油站标志检测计数器
        _bridgeEnable = false; // 桥区域使能标志
        counterFild = 0;
    }

    void bridgeCheck(vector<PredictResult> predict)
    {
        if(counterFild < 30)
        {
            counterFild++;//屏蔽计数器
            return;
        }
        if(!_bridgeEnable)
        {
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].label == LABEL_BRIDGE && predict[i].y + predict[i].height / 2 > params.rowCheck)
                {
                    counterRec++;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec > params.BridgeCheck && counterSession < params.BridgeCheck + 3)
                {
                    counterRec = 0;
                    counterSession = 0;
                    _bridgeEnable = true; // 检测到桥标志
                    return;
                }
                else if (counterSession >= params.BridgeCheck + 3)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
        }
    }

    bool bridgeDetection(TrackRecognition &track)
    {
        if (_bridgeEnable) // 进入桥梁
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2) // 切行，防止错误前瞻引发转向
            {
                track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() / 2);
                track.pointsEdgeRight.resize(track.pointsEdgeRight.size() / 2);
            }
            counterSession++;
            if (counterSession > params.ExitFrameCnt) // 上桥40场图像后失效
            {
                counterRec = 0;
                counterSession = 0;
                counterFild = 0;
                _bridgeEnable = false;
            }

            return true;
        }
        else // 检测桥
            return false;
    }

    float get_speed()
    {
        if(_bridgeEnable)
        {
            return params.SpeedUp;
        }
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

        if (_bridgeEnable)
            putText(image, "_bridgeEnable", Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/bridge.json";
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
    Params params;
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 加油站标志检测计数器
    bool _bridgeEnable = false;   // 桥区域使能标志
    uint16_t counterFild = 0;
};