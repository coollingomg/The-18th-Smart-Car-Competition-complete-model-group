#pragma once

/**
 * @file farmland_avoidance.cpp
 * @author pxx
 * @brief 农田断路区避障算法
 * @version 0.1
 * @date 2023-05-22
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

class FarmlandAvoidance
{
public:
    FarmlandAvoidance()
    {
        loadParams();
        kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.CloseWidth, params.CloseHeigth));//创建结构元
		kernel_enrode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(params.EnrodeWidth, params.EnrodeHeigth));
		kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(params.OpenSize, params.OpenSize));
    }
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        counterRec = 0;
        counterSession = 0;
        farmlandStep = FarmlandStep::None;
    }
    
    /**
     * @brief 农田区域检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool farmlandAvoid(TrackRecognition &track, vector<PredictResult> predict, cv::Mat frame)
    {
        switch(farmlandStep)
        {
        case FarmlandStep::None:
        {
            searchCorn(predict); // 玉米检测
            if (pointCorn.x > 0 || pointCorn.y > 0)
                counterRec++;

            if (counterRec)
            {
                counterSession++;
                if (counterRec > params.FarmlandCheck && counterSession < params.FarmlandCheck * 2)
                {
                    farmlandStep = FarmlandStep::Enable; // 农田区域使能
                    counterRec = 0;
                    counterSession = 0;
                }
                else if (counterSession >= params.FarmlandCheck * 2)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            break;
        }
        case FarmlandStep::Enable:
        {
            if (track.pointsEdgeLeft.size() < params.EnterLine && track.pointsEdgeRight.size() < params.EnterLine)
            {
                counterRec++;
                if (counterRec > 2)
                {
                    counterSession = 0;
                    counterRec = 0;
                    farmlandStep = FarmlandStep::Cruise;
                }
            }
            else
            {
                if (track.pointsEdgeLeft.size() > 50)
                {
                    track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() * 0.7);
                }
                if (track.pointsEdgeRight.size() > 50)
                {
                    track.pointsEdgeRight.resize(track.pointsEdgeRight.size() * 0.7);
                }
            }
            
            break;
        }
        case FarmlandStep::Enter:
        {
            //空出用于补充
            break;
        }
        case FarmlandStep::Cruise:
        {
            searchCorn(predict);
            if ((track.pointsEdgeLeft.size() > 80 || track.pointsEdgeRight.size() > 80) && pointCorn.x == 0 &&
                counterSession && (track.pointsEdgeLeft[10].x > COLSIMAGE / 2 || track.pointsEdgeRight[10].x > COLSIMAGE / 2))
            {
                counterRec++;
                if(counterRec > 5)
                {
                    counterSession = 0;
                    counterRec = 0;
                    farmlandStep = FarmlandStep::None; // 出农田
                }
            }

            searchCones(predict);
            pointsSortForX(pointEdgeDet);
            if(pointEdgeDet[0].x > COLSIMAGE /2)
            {
                counterSession++;
            }

            _imageGray = ConeEnrode(frame);
            threshold(_imageGray, _imageBinary, 0, 255, THRESH_OTSU);

            track.reset();
            track.trackRecognition(_imageBinary);
            movingAverageFilter(track.pointsEdgeLeft, 10);
            movingAverageFilter(track.pointsEdgeRight, 10);
            line_extend(track.pointsEdgeLeft);
            line_extend(track.pointsEdgeRight);
            uint16_t size = MIN(track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());
            if(size > 180)
            {
                size = 180;
                track.pointsEdgeRight.resize(size);
                track.pointsEdgeLeft.resize(size);
            }
            for(int i = 0; i < size; i++)
            {
                uint16_t width = track.pointsEdgeRight[i].y - track.pointsEdgeLeft[i].y;
                if(width > COLSIMAGE / 10)
                    track.widthBlock.push_back(POINT(i, width));
                else
                {
                    track.pointsEdgeRight.resize(i);
                    track.pointsEdgeLeft.resize(i);
                    break;
                }
            }
            break;
        }
        }

        if (farmlandStep == FarmlandStep::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/farmland.json";
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

    /**
     * @brief 对锥桶做腐蚀处理，以便于重新得到边线
     *
     * @param image 彩色通道图
     */
    cv::Mat ConeEnrode(cv::Mat image)
    {
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat blueChannel = channels[0];

		cv::Mat imageGray;

        // cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//创建结构元
		// cv::Mat kernel_enrode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(120, 60));
		cv::morphologyEx(blueChannel, blueChannel, cv::MORPH_CLOSE, kernel_close, cv::Point(-1, -1));//闭运算
		cv::morphologyEx(blueChannel, imageGray, cv::MORPH_ERODE, kernel_enrode, cv::Point(-1, -1));//腐蚀运算
		cv::morphologyEx(imageGray, imageGray, cv::MORPH_OPEN, kernel_open, cv::Point(-1, -1));//开运算
        return imageGray;
    }

    /**
     * @brief 滑动均值滤波器
     *
     * @param data 边缘点集
     * @param windowSize 滤波器窗口大小
     */
    void movingAverageFilter(std::vector<POINT>& data, int windowSize) 
    {
        int dataSize = data.size();
        
        for (int i = 0; i < dataSize; ++i) 
        {
            int start = i - windowSize + 1;
            int end = i + 1;
            start = (start < 0) ? 0 : start;
            
            double sum = 0.0;
            int count = 0;
            for (int j = start; j < end; ++j) 
            {
                if (j >= dataSize) break;
                sum += data[j].y;
                count++;
            }
            
            double average = sum / count;
            data[i].y = (uint16_t)average;
        }
    }

    float get_speed()
    {
        float speed = 0.0f;
        switch(farmlandStep)
        {
        case FarmlandStep::Enable:
        {
            speed = params.Speed; 
            break;
        }
        case FarmlandStep::Cruise:
        {
            speed = params.Speed * params.SpeedScale; 
            break;
        }
        }
        return speed;
    }


    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, cv::Mat &image)
    {
        if(farmlandStep == FarmlandStep::Cruise)
        {
            cv::Mat colorImage;
            cv::cvtColor(_imageGray, colorImage, cv::COLOR_GRAY2BGR);
            image = colorImage;
        }

        // 绘制4象限分割线
        line(image, Point(0, image.rows / 2), Point(image.cols, image.rows / 2), Scalar(255, 255, 255), 1);
        line(image, Point(image.cols / 2, 0), Point(image.cols / 2, image.rows - 1), Scalar(255, 255, 255), 1);

        // 绘制锥桶坐标
        for (int i = 0; i < pointEdgeDet.size(); i++)
        {
            // circle(image, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 4, Scalar(92, 92, 205), -1); // 锥桶坐标：红色
            putText(image, to_string(i+1), Point(pointEdgeDet[i].y, pointEdgeDet[i].x), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
        }

        circle(image, Point(pointCorn.y, pointCorn.x), 3, Scalar(8, 112, 247), -1); // 玉米坐标绘制：橙色
        circle(image, Point(pointAverage.y, pointAverage.x), 5, Scalar(226, 43, 138), -1); // 紫色

        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        // 显示施工区状态
        string state = "None";
        switch (farmlandStep)
        {
        case FarmlandStep::Enable:
            state = "FarmlandEnable";
            break;
        case FarmlandStep::Enter:
            state = "FarmlandEnter";
            break;
        case FarmlandStep::Cruise:
            state = "FarmlandCruise";
            break;
        }
        putText(image, state, Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        putText(image, to_string(indexDebug), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
    }

private:
    /**
     * @brief 玉米坐标检测
     *
     * @param predict
     */
    void searchCorn(vector<PredictResult> predict)
    {
        POINT corn = POINT(0, 0);
        int distance = COLSIMAGE / 2;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CORN && abs(predict[i].x + predict[i].width / 2 - COLSIMAGE / 2) < distance) // 玉米检测
            {
                corn = POINT(predict[i].y + predict[i].height, predict[i].x + predict[i].width / 2);
                distance = abs(predict[i].x + predict[i].width / 2 - COLSIMAGE / 2);
            }
        }

        pointCorn = corn;
    }
    /**
     * @brief 从AI检测结果中检索锥桶坐标集合
     *
     * @param predict AI检测结果
     */
    void searchCones(vector<PredictResult> predict)
    {
        // AI结果检索
        pointEdgeDet.clear();
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CONE) // 锥桶检测
            {
                pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }
        for(int i = 0; i < pointEdgeDet.size(); i++)
        {
            if(i == 0)
            {
                pointAverage = pointEdgeDet[0];
                continue;
            }
            pointAverage.x = (pointAverage.x + pointEdgeDet[i].x) / 2;
            pointAverage.y = (pointAverage.y + pointEdgeDet[i].y) / 2;
        }
    }

    /**
     * @brief 按照坐标点的x 降序排列
     *
     * @param points
     * @return vector<int>
     */
    void pointsSortForX(vector<POINT> &points)
     {
        int n = points.size();
        bool flag = true;

        for (int i = 0; i < n - 1 && flag; i++)
        {
            flag = false;
            for (int j = 0; j < n - i - 1; j++) 
            {
                if (points[j].x < points[j + 1].x) 
                {
                    POINT temp = points[j];
                    points[j] = points[j + 1];
                    points[j + 1] = temp;
                    flag = true;
                }
            }
        }
    }


    //画延长线
    void line_extend(std::vector<POINT> &points)
    {
        int edge_size = points.size();

        int x0 = points[edge_size - 1].x;
        int y0 = points[edge_size - 1].y;
        int dx = abs(points[edge_size - 1].x - points[edge_size - 6].x), sx = points[edge_size - 1].x < points[edge_size - 8].x ? 1 : -1;
        int dy = abs(points[edge_size - 1].y - points[edge_size - 6].y), sy = points[edge_size - 1].y < points[edge_size - 8].y ? 1 : -1;
        int erro = (dx > dy ? dx : -dy) / 2;

        while (x0 < ROWSIMAGE && x0 > 0 && y0 < COLSIMAGE && y0 > 0)
        {
            int e2 = erro;
            if(x0 != points[points.size() - 1].x)
            {
                points.push_back(POINT(x0, y0));
            }
            if (e2 > -dx) { erro -= dy; x0 -= sx; }
            if (e2 < dy) { erro += dx; y0 -= sy; }
        }
    }


    cv::Mat kernel_close;//闭运算滤波结构元
	cv::Mat kernel_enrode;//腐蚀运算滤波结构元
    cv::Mat kernel_open;//开运算滤波结构元
    cv::Mat _imageBinary = cv::Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC1);
    cv::Mat _imageGray = cv::Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC1);
    vector<POINT> pointEdgeDet;        // AI元素检测边缘点集
    POINT pointCorn = POINT(0, 0);     // 玉米检测坐标
    POINT pointAverage = POINT(0, 0);  // 玉米检测坐标
    uint16_t counterSession = 0;       // 图像场次计数器
    uint16_t counterRec = 0;           // 农田区标志检测计数器
    int indexDebug = 0;
    enum FarmlandStep
    {
        None = 0, // 未触发
        Enable,   // 农田区域检测使能（检测到玉米）
        Enter,    // 完全驶入农田
        Cruise
    };
    FarmlandStep farmlandStep = FarmlandStep::None;

    /**
     * @brief 农田区域核心参数
     */
    struct Params 
    {
        uint16_t FarmlandCheck = 4;
        float Speed = 0.6;
        float SpeedScale = 0.6;
        uint16_t CloseWidth = 3;
        uint16_t CloseHeigth = 3;
        uint16_t EnrodeWidth = 80;
        uint16_t EnrodeHeigth = 40;
        uint16_t OpenSize = 30;
        uint16_t EnterLine = 50;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, FarmlandCheck, Speed, SpeedScale, CloseWidth, CloseHeigth, EnrodeWidth, EnrodeHeigth, OpenSize, EnterLine); // 添加构造函数
    };
    Params params;
};
