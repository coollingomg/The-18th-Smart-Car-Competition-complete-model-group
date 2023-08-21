/**
 * @file granary_detect.cpp
 * @author pxx
 * @brief 粮仓区域检测与路径规划
 * @version 0.1
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
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

class GranaryDetection
{
public:
    enum GranaryType
    {
        ExitNone = 0, // 未触发
        ExitOne,  // 一号出口
        ExitTwo   // 二号出口
    };
    GranaryType granaryType = GranaryType::ExitNone;

public:
    GranaryDetection()
    {
        loadParams();
    }
    ~GranaryDetection() {}

    /**
     * @brief 粮仓初始化
     *
     */
    void reset()
    {
        granaryStep = GranaryStep::None;
        granaryType = GranaryType::ExitNone;
        lastPointsEdgeLeft.clear();
        lastPointsEdgeRight.clear();
        counterSession = 0;
        counterRec = 0;
    	counterExit = 0;
        exitTwoEnable = false;
        numGranary = 0;
    }

    /**
     * @brief 粮仓检测确认 运行于AI线程
     *
     * @param predict AI 检测结果
     */
    void granaryCheck(std::vector<PredictResult> predict)
    {
        if(counterShield < 15)
        {
            counterShield++;
            return;
        }

		std::vector<POINT> granarys = searchGranary(predict);
		if(granarys.size() > 1)
			numGranary++;

        if(granaryStep == GranaryStep::None)
        {
            if(granarys.size() > 0)
                counterRec++;

            if(counterRec)
            {
                counterSession++;
                if(counterRec > params.GranaryCheck && counterSession < params.GranaryCheck * 2 + 1)
                {
                    counterRec = 0;
                    counterSession = 0;
                    granaryStep = GranaryStep::Enable;
                    // if(numGranary > params.GranaryCheck / 2)
                    //     granaryType = GranaryType::ExitTwo;
                    // else
                    //     granaryType = GranaryType::ExitOne;
                }
                else if(counterSession >= params.GranaryCheck * 2 + 1)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
        }
		else if(granaryStep == GranaryStep::Enable) // 维持一段时间，知道粮仓标志消失
		{
			if(granarys.size() == 0)
			{
				counterExit++;
				if(counterRec)
					counterSession++;
				// if(numGranary > params.GranaryCheck / 2)
				// 	granaryType = GranaryType::ExitTwo;
				// else
				// 	granaryType = GranaryType::ExitOne;
			}
			else if(granarys[0].x > params.ServoEnter + ROWSIMAGE / 6)
			{
				counterRec++;
				// if(numGranary > params.GranaryCheck / 2)
				// 	granaryType = GranaryType::ExitTwo;
				// else
				// 	granaryType = GranaryType::ExitOne;
			}
			else {
				counterSession = 0;
				counterExit = 0;
			}

			if(counterSession > 1 || counterExit > 4)
			{
				counterRec = 0;
				counterSession = 0;
				counterExit = 0;
				if(numGranary > params.GranaryCheck / 2)
					granaryType = GranaryType::ExitTwo;
				else
					granaryType = GranaryType::ExitOne;
			}
		}
    }

    /**
     * @brief 粮仓区路径规划 运行于主线程
     *
     * @param track
     * @param frame
     */
    bool granaryDetection(TrackRecognition& track, cv::Mat img_rgb)
    {
        _pointNearCone = POINT(0, 0);
        pointEdgeDet.clear();
        _coneRects.clear();
        _bezier_input.clear();

        switch(granaryStep)
        {
        case GranaryStep::Enable:
        {

			if(granaryType == GranaryType::ExitNone)
				return true;

			counterExit++;
			if (counterExit > 100) {
				reset();
				return false;
			}

            _coneRects = detectCones(img_rgb);
			searchCones(_coneRects, track.rowCutUp);
            _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet); // 搜索右下锥桶

			if (_pointNearCone.x > params.ServoEnter && _pointNearCone.x < params.ServoEnter + ROWSIMAGE / 6
				&& _pointNearCone.y != 0) // 当车辆开始靠近右边锥桶：准备入库
            {
                counterRec++;
				if (counterRec > 1)
				{
					granaryStep = GranaryStep::Enter; // 进站使能
					counterRec = 0;
					counterSession = 0;
				}
            }
            else if (_pointNearCone.x > params.ServoEnter && _pointNearCone.x < params.ServoEnter + ROWSIMAGE / 3)
            {
                slowDown = true; // 进站减速
            }

            break;
        }
        case GranaryStep::Enter:
        {
            _coneRects = detectCones(img_rgb);
            searchCones(_coneRects, track.rowCutUp);
			if (track.pointsEdgeLeft.size() > COLSIMAGE / 2) // 第一阶段：当赛道边缘存在时
            {
                _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet); // 搜索右下锥桶(图像右上方)

                if(_pointNearCone.x > 0)
                {
					POINT startPoint = POINT((_pointNearCone.x + ROWSIMAGE) / 2, (_pointNearCone.y + COLSIMAGE) / 2); // 线起点：右
					POINT midPoint = _pointNearCone; // 补线中点
					double k = 0, b = 0;
					k = (float)(midPoint.y - startPoint.y) / (float)(midPoint.x - startPoint.x);
					b = midPoint.y - k * midPoint.x;
					if (b < 0)
						b = 0;
					else if (b >= COLSIMAGE)
						b = COLSIMAGE - 1;
					POINT endPoint = POINT(0, b);	 // 补线终点：左

					vector<POINT> input = {startPoint, midPoint, endPoint};
					vector<POINT> repair = Bezier(0.02, input);
                    _bezier_input = input;
					track.pointsEdgeRight = repair;
					track.pointsEdgeLeft.clear();

					for (int i = 0; i < repair.size(); i++)
					{
						track.pointsEdgeLeft.push_back(POINT(repair[i].x, 0));
					}
                }
            }
			else // 第二阶段：检查右下锥桶坐标满足巡航条件
            {
				POINT coneRightDown = searchRightDownCone(pointEdgeDet); // 右下方锥桶
				_pointNearCone = coneRightDown;
				counterSession++;
				if ((coneRightDown.x > params.ServoCruise && coneRightDown.y > COLSIMAGE - 80)/* || counterSession > params.DelayCnt*/)
				{
					counterRec++;
					if (counterRec > 1)
					{
						granaryStep = GranaryStep::Cruise; // 巡航使能
						counterRec = 0;
						counterSession = 0;
					}
				}
				if (coneRightDown.x > 0) // 进站补线
				{
					POINT startPoint = POINT((coneRightDown.x + ROWSIMAGE) / 2, (coneRightDown.y + COLSIMAGE) / 2); // 线起点：右
					POINT midPoint = coneRightDown; // 补线中点
					double k = 0, b = 0;
					k = (float)(midPoint.y - startPoint.y) / (float)(midPoint.x - startPoint.x);
					b = midPoint.y - k * midPoint.x;
					if (b < 0)
						b = 0;
					else if (b >= COLSIMAGE)
						b = COLSIMAGE - 1;
					POINT endPoint = POINT(0, b);	// 补线终点：左

					vector<POINT> input = {startPoint, midPoint, endPoint};
					vector<POINT> repair = Bezier(0.02, input);
                    _bezier_input = input;
					track.pointsEdgeRight = repair;
					track.pointsEdgeLeft.clear();

					for (int i = 0; i < repair.size(); i++)
					{
						track.pointsEdgeLeft.push_back(POINT(repair[i].x, 0));
					}
				}
            }
            break;
        }
        case GranaryStep::Cruise:
        {
            _coneRects = detectCones(img_rgb);
            searchCones(_coneRects, track.rowCutUp);
			vector<POINT> conesLeft = searchLeftCone(pointEdgeDet); // 搜索左方锥桶

			if (track.pointsEdgeLeft.size() > ROWSIMAGE / 5 &&
				track.pointsEdgeRight.size() > ROWSIMAGE / 5)
			{
				slowDown = true; // 出站减速
				counterRec++;
				if (counterRec >= 2)
				{
					granaryStep = GranaryStep::Exit; // 出站使能
					counterRec = 0;
					counterSession = 0;
				}
			}
			else
				counterRec = 0;

			if (conesLeft.size() >= 2)
			{
				int indexMin = 0;
				int indexMax = 0;
				for (int i = 0; i < conesLeft.size(); i++)
				{
					if (conesLeft[i].x > conesLeft[indexMax].x)
						indexMax = i;
					if (conesLeft[i].x < conesLeft[indexMin].x)
						indexMin = i;
				}

				if (indexMin != indexMax) // 开始补线
				{
					_pointNearCone = conesLeft[indexMin];
					double k = 0, b = 0;
					k = (float)(conesLeft[indexMax].y - conesLeft[indexMin].y) /
						(float)(conesLeft[indexMax].x - conesLeft[indexMin].x);
					b = conesLeft[indexMax].y - k * conesLeft[indexMax].x;

					if (k != 0 && b != 0)
					{
						POINT startPoint = POINT(-b / k, 0); // 补线起点：左
						// POINT startPoint = POINT((-b / k + ROWSIMAGE) / 2, 0); // 补线起点：左
						// POINT startPoint = POINT(ROWSIMAGE - 40, 0); // 补线起点：左
						
						POINT endPoint = POINT(0, b);		 // 补线终点：右
						POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 补线中点
						startPoint = POINT((-b / k) * (1 - params.ReverseScale) + ROWSIMAGE * params.ReverseScale, 0); //补线起点修正

						vector<POINT> input = {startPoint, midPoint, endPoint};
                        _bezier_input = input;
						vector<POINT> repair = Bezier(0.02, input);

						track.pointsEdgeRight = track.predictEdgeRight(repair, true, params.ConeWidth); // 俯视域预测右边缘

						// if (repair.size() > 10) // 左边缘切行，提升右拐能力
						// {
						// 	int index = repair.size() * 0.2;
						// 	track.pointsEdgeLeft.clear();
						// 	for (int i = index; i < repair.size(); i++)
						// 	{
						// 		track.pointsEdgeLeft.push_back(repair[i]);
						// 	}
						// }
						// else
							track.pointsEdgeLeft = repair;

						lastPointsEdgeLeft = track.pointsEdgeLeft;
					}
				}
			}
			else if (pointEdgeDet.size() > 3) // 左边锥桶太少，总锥桶数量足够，判断还在粮仓区域内，使用上一帧的数据
			{
				track.pointsEdgeLeft = lastPointsEdgeLeft;
				track.pointsEdgeRight = track.predictEdgeRight(track.pointsEdgeLeft, true, params.ConeWidth); // 俯视域预测右边缘
			}
			else
			{
				// 添加异常处理函数
				
			}

			// 出口检测 1号出口
			if (granaryType == GranaryType::ExitOne) 
			{
				POINT coneRightDown = searchRightDownCone(pointEdgeDet); // 右下方锥桶
				_pointNearCone = coneRightDown;
				counterSession++;
				
				if ((coneRightDown.x > params.ServoExitOne && coneRightDown.x < params.ServoExitOne + ROWSIMAGE / 6
					&& counterSession > params.DelayCnt)) // 右下方锥桶检测完毕
				{
					granaryStep = GranaryStep::Exit; // 出站使能
					counterRec = 0;
					counterSession = 0;
				}
			}
			else if(granaryType == GranaryType::ExitTwo)
			{
				POINT coneRightUp = searchTopUpCone(pointEdgeDet); // 右上方锥桶
				_pointNearCone = coneRightUp;
				counterSession++;
				
				if ((_pointNearCone.x > params.ServoExitTwo && _pointNearCone.x < params.ServoExitTwo + ROWSIMAGE / 6
					&& counterSession > params.DelayCnt)) // 右下方锥桶检测完毕
				{
					granaryStep = GranaryStep::Exit; // 出站使能
					counterRec = 0;
					counterSession = 0;
				}

			}
            
            break;
        }
        case GranaryStep::Exit:
        {
			_coneRects = detectCones(img_rgb);
            searchCones(_coneRects, track.rowCutUp);
			POINT coneLeftUp = searchRightUpCone(pointEdgeDet); // 搜索右上方的锥桶用于补线
			_pointNearCone = coneLeftUp;

			if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 &&
				track.pointsEdgeRight.size() > ROWSIMAGE / 2 &&
				pointEdgeDet.size() < 3)
			{
				reset();
			}
			else
			{
				if (coneLeftUp.x > 0)
				{
					POINT p1 = POINT(ROWSIMAGE - 10, coneLeftUp.y / 2);
					POINT p2 = POINT((coneLeftUp.x + ROWSIMAGE) / 2, coneLeftUp.y / 2);
					POINT p3 = coneLeftUp;
					POINT p4 = POINT(coneLeftUp.x / 2, (coneLeftUp.y + COLSIMAGE) / 2);
					vector<POINT> input = {p1, p2, p3, p4};
					vector<POINT> repair = Bezier(0.02, input);

					_bezier_input = input;
					track.pointsEdgeLeft = repair;
					lastPointsEdgeLeft = repair;
					track.pointsEdgeRight.clear();
					for (int i = 0; i < repair.size(); i++)
					{
						track.pointsEdgeRight.push_back(POINT(repair[i].x, COLSIMAGE - 1));
					}
					lastPointsEdgeRight = track.pointsEdgeRight;
				}
				else
				{
					track.pointsEdgeLeft = lastPointsEdgeLeft;
					track.pointsEdgeRight = lastPointsEdgeRight;
				}
			}

            break;
        }
        }

        if(granaryStep == GranaryStep::None)
            return false;
        else
            return true;
    }

	/**
	 * @brief 获取粮仓区速度规划
	 *
	 */
	float get_speed(float motionSpeed)
	{
		float speed_temp;
		if(granaryType == GranaryType::ExitOne)
			speed_temp = params.SpeedExitOne;
		else if(granaryType == GranaryType::ExitTwo)
			speed_temp = params.SpeedExitTwo;

		switch(granaryStep)
		{
		case GranaryStep::Enable:
		{
			motionSpeed -= 0.05f;
			if(motionSpeed < speed_temp)
				motionSpeed = speed_temp;
			
			break;
		}
		case GranaryStep::Enter:
		{
			motionSpeed = speed_temp;
			break;
		}
		case GranaryStep::Cruise:
		{
			motionSpeed = speed_temp;
			break;
		}
		case GranaryStep::Exit:
		{
			motionSpeed = speed_temp;
			break;
		}
		}
		return motionSpeed;
	}

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, Mat &image)
    {
        // 绘制锥桶坐标
        for (int i = 0; i < pointEdgeDet.size(); i++)
        {
            cv::circle(image, cv::Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 5, Scalar(200, 200, 200), -1); 
        }
		// for (int i = 0; i < pointEdgeDet.size(); i++)
		// {
        //     putText(image, to_string(i+1), Point(pointEdgeDet[i].y, pointEdgeDet[i].x), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
		// }

        // for(const auto& rect : _coneRects)
        // {
        //     // cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
        //     circle(image, Point(rect.x + rect.width / 2, rect.y + rect.height / 2), 5, Scalar(200, 200, 200), -1);
        // }


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

        // 显示粮仓状态
        string state = "None";
        switch (granaryStep)
        {
        case GranaryStep::Enable:
            state = "GranaryEnable";
            break;
        case GranaryStep::Enter:
            state = "GranaryEnter";
            break;
        case GranaryStep::Cruise:
            state = "GranaryCruise";
            break;
        case GranaryStep::Exit:
            state = "GranaryExit";
            break;
        }
        if(granaryType == GranaryType::ExitOne)
            state += "-one";
        else if(granaryType == GranaryType::ExitTwo)
            state += "-two";

        putText(image, state, Point(COLSIMAGE / 2 - 40, 20), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 255, 0), 1, CV_AA);
        putText(image, to_string(granaryStep), Point(COLSIMAGE / 2, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
        for(int i = 0; i < _bezier_input.size(); i++)
        {
            circle(image, Point(_bezier_input[i].y, _bezier_input[i].x), 5, Scalar(0, 0, 255), 1);
        }
        if (_pointNearCone.x > 0)
        {
			putText(image, "Nearest: " + to_string(_pointNearCone.x), Point(COLSIMAGE / 2 - 20, 40),
					cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
					CV_AA); // 显示锥桶距离
            circle(image, Point(_pointNearCone.y, _pointNearCone.x), 4, Scalar(226, 43, 138), -1);//紫色
        }
    }

private:
    /**
     * @brief 从AI检测结果中检索谷仓标志
     *
     * @param predict
     * @return vector<POINT>
     */
    std::vector<POINT> searchGranary(std::vector<PredictResult> predict)
    {
        vector<POINT> granarys;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_GRANARY && predict[i].y + predict[i].height / 2 > 40)
            {
                granarys.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }

        return granarys;
    }

	/**
	 * @brief 从AI检测结果中检索锥桶坐标集合
	 *
	 * @param predict AI检测结果
	 * @return vector<POINT>
	 */
	void searchCones(vector<PredictResult> predict)
	{
		pointEdgeDet.clear();
		for (int i = 0; i < predict.size(); i++)
		{
			if (predict[i].label == LABEL_CONE) // 锥桶检测
			{
				pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2,
											 predict[i].x + predict[i].width / 2));
			}
		}
	}

	/**
	 * @brief 从视觉结果中检索锥桶坐标集合
	 *
	 * @param predict 检测结果
	 * @param rowCutUp 滤除掉图片最上方部分的色块
	 * @return vector<POINT>
	 */
	void searchCones(vector<Rect> predict, uint16_t rowCutUp = 0)
	{
		pointEdgeDet.clear();
		for (int i = 0; i < predict.size(); i++)
		{
			if(predict[i].y + predict[i].height / 2 > rowCutUp)
				pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2,
												predict[i].x + predict[i].width / 2));
		}
	}

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/granary.json";
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
	 * @brief 搜索距离赛道边缘最近的锥桶坐标（左下 / 右下）
	 *
	 * @param pointsEdgeLine 赛道边缘点集
	 * @param pointsCone     锥桶检测结果
	 * @return POINT
	 */
	POINT searchNearestCone(vector<POINT> pointsEdgeLine, vector<POINT> pointsCone)
	{
		POINT point(0, 0);
		double disMin = 50; // 右边缘锥桶离赛道左边缘最小距离

		if (pointsCone.size() <= 0 || pointsEdgeLine.size() < 10)
			return point;

		POINT a = pointsEdgeLine[pointsEdgeLine.size() / 4];
		POINT b = pointsEdgeLine[pointsEdgeLine.size() / 2];

		for (int i = 0; i < pointsCone.size(); i++)
		{
			double dis = distanceForPoint2Line(a, b, pointsCone[i]);
			if (dis < disMin && pointsCone[i].x > point.x)
			{
				point = pointsCone[i];
			}
		}

		return point;
	}

	/**
	 * @brief 搜索右下方的锥桶坐标
	 *
	 * @param pointsCone
	 * @return POINT
	 */
	POINT searchRightDownCone(vector<POINT> pointsCone)
	{
		POINT point(0, 0);

		if (pointsCone.size() <= 0)
			return point;

		for (int i = 0; i < pointsCone.size(); i++)
		{
			if (pointsCone[i].y > COLSIMAGE / 2 && pointsCone[i].x < ROWSIMAGE - 40 && pointsCone[i].x > point.x)
			{
				point = pointsCone[i];
			}
		}

		return point;
	}

	/**
	 * @brief 搜索左方的锥桶坐标
	 *
	 * @param pointsCone
	 * @return vector<POINT>
	 */
	vector<POINT> searchLeftCone(vector<POINT> pointsCone)
	{
		vector<POINT> points;

		if (pointsCone.size() <= 0)
			return points;

		for (int i = 0; i < pointsCone.size(); i++)
		{
			if (pointsCone[i].y < COLSIMAGE / 2 - 45)
			{
				points.push_back(pointsCone[i]);
			}
		}

		return points;
	}

	/**
	 * @brief 搜索左右方的锥桶坐标
	 *
	 * @param pointsCone
	 * @return vector<POINT>
	 */
	POINT searchRightUpCone(vector<POINT> pointsCone)
	{
		POINT point(0, 0);

		if (pointsCone.size() <= 0)
			return point;

		for (int i = 0; i < pointsCone.size(); i++)
		{
			if (pointsCone[i].y > point.y && pointsCone[i].x < ROWSIMAGE * 0.8)
			{
				point = pointsCone[i];
			}
		}

		return point;
	}
	/**
	 * @brief 搜索左右方的锥桶坐标
	 *
	 * @param pointsCone
	 * @return vector<POINT>
	 */
	POINT searchTopUpCone(vector<POINT> pointsCone)
	{
		POINT point(0, 0);

		if (pointsCone.size() <= 0)
			return point;
		else
			point = pointsCone[0];

		for (int i = 0; i < pointsCone.size(); i++)
		{
			if (pointsCone[i].x < point.x && pointsCone[i].x < ROWSIMAGE * 0.8)
			{
				point = pointsCone[i];
			}
		}

		return point;
	}

	//传统视觉识别锥桶
	std::vector<cv::Rect> detectCones(cv::Mat img_rgb)
	{
		std::vector<cv::Rect> coneRects;
		// 设置锥桶颜色的RGB范围（黄色）
		cv::Scalar lowerYellow(0, 100, 100);
		cv::Scalar upperYellow(100, 255, 255);

		// 在RGB图像中根据颜色范围提取锥桶区域
		cv::Mat mask;
		cv::inRange(img_rgb, lowerYellow, upperYellow, mask);

		// 进行形态学操作，去除噪声并提取锥桶区域的轮廓
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// 查找最大轮廓
		size_t maxContourIndex = 0;
		double maxContourArea = 0.0;
		for (size_t i = 0; i < contours.size(); ++i)
		{
			double contourArea = cv::contourArea(contours[i]);
			if (contourArea > maxContourArea)
			{
				maxContourArea = contourArea;
				maxContourIndex = i;
			}
		}

		// 绘制正方形框选中锥桶区域
		for (const auto& contour : contours)
		{
			cv::Rect boundingRect = cv::boundingRect(contour);
			coneRects.push_back(boundingRect);
		}	
		return coneRects;
	}

private:
    POINT _pointNearCone;
    std::vector<POINT> pointEdgeDet;        // 锥桶检测边缘点集
	std::vector<cv::Rect> _coneRects;       // 传统视觉识别锥桶的方框点集
    std::vector<POINT> lastPointsEdgeLeft;  // 记录上一场边缘点集（丢失边）
    std::vector<POINT> lastPointsEdgeRight; // 记录上一场边缘点集（丢失边）
    std::vector<POINT> _bezier_input;
    bool exitTwoEnable = false;             // 二号出口使能标志
    bool slowDown = false;                  // 减速使能

    enum GranaryStep
    {
        None = 0, // 未触发
        Enable,   // 粮仓操作使能（标志识别成功）
        Enter,    // 粮仓进站
        Cruise,   // 粮仓巡航
        Exit      // 粮仓出站
    };

    GranaryStep granaryStep = GranaryStep::None;
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 粮仓标志检测计数器
    uint16_t counterShield = 0;  // 粮仓屏蔽计数器
	uint16_t counterExit = 0;	 // 标志结束计数器
    uint16_t numGranary = 0;     // 粮仓标志的数量

    /**
     * @brief 粮仓区核心参数
     *
     */
    struct Params
    {
        uint16_t GranaryCheck = 3;
		float SpeedExitOne = 0.8;
		float SpeedExitTwo = 0.8;
        uint16_t ServoEnter = 50;
        uint16_t ServoCruise = 150;
		uint16_t ServoExitOne = 70;
		uint16_t ServoExitTwo = 70;
		float ReverseScale = 0.5;
        uint16_t DelayCnt = 20;
		uint16_t ConeWidth = 180;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, GranaryCheck, SpeedExitOne, SpeedExitTwo, ServoEnter, ServoCruise, ServoExitOne, ServoExitTwo,
										ReverseScale, DelayCnt, ConeWidth); // 添加构造函数
    };
    Params params;
};
