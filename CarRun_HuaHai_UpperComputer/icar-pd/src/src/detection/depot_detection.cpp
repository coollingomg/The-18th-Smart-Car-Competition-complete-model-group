#pragma once

/**
 * @file depot_detection.cpp
 * @author pxx
 * @brief 维修厂检测
 * @version 0.1
 * @date 2023-05-17
 *
 */

#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognize/track_recognition.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class DepotDetection
{
public:
    /**
     * @brief 控制器核心参数
     */
    struct Params 
    {
		uint16_t DepotCheck = 3;
		double DangerClose = 100.0;       // 智能车危险距离
		uint16_t ServoRow = 120;
		uint16_t ServoValue = 15;
		float DepotSpeed = 0.5;
		uint16_t BrakeCnt = 5;
		uint16_t ExitFrameCnt = 10;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, DepotCheck, DangerClose, ServoRow, ServoValue, DepotSpeed, BrakeCnt, ExitFrameCnt); // 添加构造函数
    };

	enum DepotStep
	{
		DepotNone = 0, // 未触发
		DepotEnable,   // 维修厂操作使能（标志识别成功）
		DepotEnter,	   // 维修厂进站
		DepotCruise,   // 维修站巡航
		DepotStop,	   // 停车
		DepotExit	   // 维修厂出站
	};

	/**
	 * @brief 维修厂检测初始化
	 *
	 */
	void reset(void)
	{
		depotStep = DepotStep::DepotNone;
		counterSession = 0;			// 图像场次计数器
		counterRec = 0;				// 维修厂标志检测计数器
		counterExit = 0;
		counterImmunity = 0;
	}

	/**
	 * @brief 维修厂检测与路径规划
	 *
	 * @param track 赛道识别结果
	 * @param detection AI检测结果
	 */
	bool depotDetection(TrackRecognition &track, vector<PredictResult> predict)
	{
		_pointNearCone = POINT(0, 0);
		_distance = 0;
		pointEdgeDet.clear();
		indexDebug = 0;

		switch (depotStep)
		{
		case DepotStep::DepotNone: //[01] 维修厂标志检测
			if (counterImmunity > 20)
			{
				for (int i = 0; i < predict.size(); i++)
				{
					if (predict[i].label == LABEL_TRACTOR) // 拖拉机标志检测
					{
						counterRec++;
						break;
					}
				}
				if (counterRec)
				{
					counterSession++;
					if (counterRec > params.DepotCheck && counterSession < params.DepotCheck + 3)
					{
						depotStep = DepotStep::DepotEnable; // 维修厂使能
						counterRec = 0;
						counterSession = 0;
					}
					else if (counterSession >= params.DepotCheck + 3)
					{
						counterRec = 0;
						counterSession = 0;
					}
				}
			}
			else
				counterImmunity++;
			break;

		case DepotStep::DepotEnable: //[02] 维修厂使能
		{
			// counterExit++;
			// if (counterExit > 60) {
			//   reset();
			//   return false;
			// }

			searchCones(predict);
			_pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet);		 // 搜索右下锥桶
			if (_pointNearCone.x > params.ServoRow && _pointNearCone.y != 0) // 当车辆开始靠近右边锥桶：准备入库
			{
				counterRec++;
				if (counterRec > 2)
				{
					depotStep = DepotStep::DepotEnter; // 进站使能
					counterRec = 0;
					counterSession = 0;
				}
			}

			indexDebug = counterRec;
			break;
		}
		case DepotStep::DepotEnter: //[03] 进站使能
		{
			searchCones(predict);
			_pointNearCone = searchClosestCone(pointEdgeDet);
			if(_distance < params.DangerClose)
			{
				counterRec++;
				if(counterRec > 1)
				{
					depotStep = DepotStep::DepotCruise;
					counterRec = 0;
				}
			}

			POINT start = POINT(ROWSIMAGE - 40, COLSIMAGE - 1);
			POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, 0);
			POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
			vector<POINT> input = {start, middle, end};
			track.pointsEdgeRight = Bezier(0.05, input); // 补线
			track.pointsEdgeLeft =
				predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘

			pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			pathsEdgeRight.push_back(track.pointsEdgeRight);

			break;
		}

		case DepotStep::DepotCruise: //[04] 巡航使能(Brake)
		{
			{
				// 预留给以后完善
			}
			counterRec++;
			if(counterRec > params.BrakeCnt)
			{
				counterRec = 0;
				depotStep = DepotStep::DepotStop;
			}
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			// pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			// pathsEdgeRight.push_back(track.pointsEdgeRight);
			break;
		}

		case DepotStep::DepotStop: //[05] 停车使能
		{
			counterRec++;
			if (counterRec > 30) // 停车：40场 = 2s
			{
				depotStep = DepotStep::DepotExit; // 出站使能
				counterRec = 0;
			}
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			break;
		}

		case DepotStep::DepotExit: //[06] 出站使能
		{
			counterRec++;
			if (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1)
			{
				depotStep = DepotStep::DepotNone; // 出厂完成
				reset();
			}
			else
			{
				track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
				track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
			}
			
			if(counterRec > params.ExitFrameCnt)
			{
				pathsEdgeLeft.pop_back();
				pathsEdgeRight.pop_back();
			}
			break;
		}
		}

		if (depotStep == DepotStep::DepotNone) // 返回维修厂控制模式标志
			return false;
		else
			return true;
	}

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/depot.json";
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
	 * @brief 获取维修区速度规划
	 *
	 */
	float get_speed()
	{
		float speed = 0.0f;
		switch(depotStep)
		{
		case DepotStep::DepotEnable:
		{
			speed = params.DepotSpeed;
			break;
		}
		case DepotStep::DepotEnter:
		{
			speed = params.DepotSpeed;
			break;
		}
		case DepotStep::DepotCruise:
		{
			speed = -params.DepotSpeed;
			break;
		}
		case DepotStep::DepotStop:
		{
			speed = 0.0f;
			break;
		}
		case DepotStep::DepotExit:
		{
			speed = -params.DepotSpeed * 1.5;
			break;
		}
		}
		return speed;
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
			circle(image, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 2,
				   Scalar(92, 92, 205), -1); // 锥桶坐标：红色
		}

		// 赛道边缘
		for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
		{
			circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x),
				   1, Scalar(0, 255, 0), -1); // 绿色点
		}
		for (int i = 0; i < track.pointsEdgeRight.size(); i++)
		{
			circle(image,
				   Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
				   Scalar(0, 255, 255), -1); // 黄色点
		}

		// 显示维修厂状态
		string state = "None";
		switch (depotStep)
		{
		case DepotStep::DepotEnable:
			state = "DepotEnable";
			break;
		case DepotStep::DepotEnter:
			state = "DepotEnter";
			break;
		case DepotStep::DepotCruise:
			state = "DepotCruise";
			break;
		case DepotStep::DepotStop:
			state = "DepotStop";
			break;
		case DepotStep::DepotExit:
			state = "DepotExit";
			break;
		}
		putText(image, state, Point(COLSIMAGE / 2 - 10, 20),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

		putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40),
				cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
				CV_AA); // 显示锥桶距离
		if (_pointNearCone.y > 0)
			circle(image, Point(_pointNearCone.y, _pointNearCone.x), 3,
				   Scalar(200, 200, 200), -1);

		putText(image, to_string(indexDebug),
				Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
				0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
	}

private:
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
	 * @brief 搜索距离赛道左边缘锥桶坐标(右下，非常规意义上的)
	 *
	 * @param pointsEdgeLeft 赛道边缘点集
	 * @param predict AI检测结果
	 * @return POINT
	 */
	POINT searchNearestCone(vector<POINT> pointsEdgeLeft,
							vector<POINT> pointsCone)
	{
		POINT point(ROWSIMAGE - 10, 0);
		double disMin = 50; // 右边缘锥桶离赛道左边缘最小距离

		if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
			return point;

		POINT a = pointsEdgeLeft[pointsEdgeLeft.size() * 0.5];
		POINT b = pointsEdgeLeft[pointsEdgeLeft.size() * 0.8];

		for (int i = 0; i < pointsCone.size(); i++)
		{
			double dis = distanceForPoint2Line(a, b, pointsCone[i]);
			if (dis < disMin && pointsCone[i].x < point.x)
			{
				point = pointsCone[i];
				_distance = dis;
			}
		}

		return point;
	}

	/**
	 * @brief 搜索距离车最近的锥桶
	 *
	 * @param pointsEdgeLeft 赛道边缘点集
	 * @param predict AI检测结果
	 * @return POINT
	 */
	POINT searchClosestCone(vector<POINT> pointsCone)
	{
		POINT closestCone(0, 0);
		if (pointsCone.size() <= 0)
			return closestCone;

		POINT carCenter(ROWSIMAGE - 1, COLSIMAGE / 2);
		double closestLen = 0;
		closestCone = pointsCone[0];
		closestLen = distance(closestCone, carCenter);

		for (int i = 1; i < pointsCone.size(); i++)
		{
			double len = distance(pointsCone[i], carCenter);
			if(len < closestLen)
			{
				closestLen = len;
				closestCone = pointsCone[i];
			}
		}
		_distance = closestLen;

		return closestCone;
	}

	/**
	 * @brief 在俯视域由左边缘预测右边缘
	 *
	 * @param pointsEdgeLeft
	 * @return vector<POINT>
	 */
	vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft)
	{
		int offset = 120; // 右边缘平移尺度
		vector<POINT> pointsEdgeRight;
		if (pointsEdgeLeft.size() < 3)
			return pointsEdgeRight;

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
		Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
		Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT startPoint = POINT(startIipm.y, startIipm.x);

		// Middle
		Point2d middleIpm = ipm.homography(
			Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y,
					pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
		prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
		Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT midPoint = POINT(middleIipm.y, middleIipm.x);	  // 补线中点

		// End
		Point2d endIpm = ipm.homography(
			Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y,
					pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
		prefictRight = Point2d(endIpm.x + offset, endIpm.y);
		Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT endPoint = POINT(endtIipm.y, endtIipm.x);

		// 补线
		vector<POINT> input = {startPoint, midPoint, endPoint};
		vector<POINT> repair = Bezier(0.05, input);

		for (int i = 0; i < repair.size(); i++)
		{
			if (repair[i].x >= ROWSIMAGE)
				repair[i].x = ROWSIMAGE - 1;

			else if (repair[i].x < 0)
				repair[i].x = 0;

			else if (repair[i].y >= COLSIMAGE)
				repair[i].y = COLSIMAGE - 1;
			else if (repair[i].y < 0)
				repair[i].y = 0;

			pointsEdgeRight.push_back(repair[i]);
		}

		return pointsEdgeRight;
	}

	/**
	 * @brief 在俯视域由右边缘预测左边缘
	 *
	 * @param pointsEdgeRight
	 * @return vector<POINT>
	 */
	vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight)
	{
		int offset = 120; // 右边缘平移尺度
		vector<POINT> pointsEdgeLeft;
		if (pointsEdgeRight.size() < 3)
			return pointsEdgeLeft;

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
		Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
		Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT startPoint = POINT(startIipm.y, startIipm.x);

		// Middle
		Point2d middleIpm = ipm.homography(
			Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y,
					pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
		prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
		Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT midPoint = POINT(middleIipm.y, middleIipm.x);	 // 补线中点

		// End
		Point2d endIpm = ipm.homography(
			Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y,
					pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
		prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
		Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT endPoint = POINT(endtIipm.y, endtIipm.x);

		// 补线

		vector<POINT> input = {startPoint, midPoint, endPoint};
		vector<POINT> repair = Bezier(0.05, input);

		for (int i = 0; i < repair.size(); i++)
		{
			if (repair[i].x >= ROWSIMAGE)
				repair[i].x = ROWSIMAGE - 1;

			else if (repair[i].x < 0)
				repair[i].x = 0;

			else if (repair[i].y >= COLSIMAGE)
				repair[i].y = COLSIMAGE - 1;
			else if (repair[i].y < 0)
				repair[i].y = 0;

			pointsEdgeLeft.push_back(repair[i]);
		}

		return pointsEdgeLeft;
	}

	//两点之间的像素距离
	double distance(POINT x1, POINT x2)
	{
		double dx = x1.x - x2.x;
		double dy = x1.y - x2.y;
		double len = std::sqrt(dx * dx + dy * dy);
		return len;
	}
	
	DepotStep depotStep = DepotStep::DepotNone;
	Params params;                   // 读取控制参数
	
	double _distance = 0;
	POINT _pointNearCone;
	vector<POINT> pointEdgeDet; // AI元素检测边缘点集

	vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
	vector<vector<POINT>> pathsEdgeRight;
	int indexDebug = 0;

	uint16_t counterSession = 0;  // 图像场次计数器
	uint16_t counterRec = 0;	  // 维修厂标志检测计数器
	uint16_t counterExit = 0;	  // 标志结束计数器
	uint16_t counterImmunity = 0; // 屏蔽计数器
};
