#include <iostream>
#include <memory>
#include <string>
#include <ctime>
#include <signal.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstdlib>  // 包含system函数的头文件

#include "../include/stop_watch.hpp"
#include "../include/uart.hpp"               //串口通信
#include "./recognize/track_recognition.cpp" //基础巡线
#include "./recognize/cross_recognition.cpp" //十字赛道
#include "./recognize/ring_recognize.cpp"    //环岛赛道
#include "./recognize/garage_recognize.cpp"  //车库及斑马线识别类
#include "./detection/farmland_avoidance.cpp"//农田区

#include "image_preprocess.cpp" //图像预处理
#include "controlcenter_cal.cpp"
#include "motion_controller.cpp"

#include "./detection/bridge_detection.cpp"
#include "./detection/slowzone_detection.cpp"
#include "./detection/depot_detection.cpp"

#include "../include/capture.hpp"
#include "../include/serial.hpp"
#include "../include/detection.hpp"

using namespace cv;
using namespace std;

enum RoadType
{
    BaseHandle = 0, // 基础赛道处理
    RingHandle,     // 环岛赛道处理
    CrossHandle,    // 十字道路处理
    FreezoneHandle, // 泛行区处理
    GarageHandle,   // 车库处理
    GranaryHandle,  // 粮仓处理
    DepotHandle,    // 修车厂处理
    BridgeHandle,   // 坡道(桥)处理
    SlowzoneHandle, // 慢行区（动物出没）处理
    FarmlandHandle, // 农田区域处理
};

void callbackSignal(int signum); // 系统退出回调函数
void displayWindowInit(void);    // 显示窗口初始化
void savePicture(Mat &image, RoadType roadType);
void ClearFolder(const std::string& folderPath);
void slowDownEnable(void);

bool slowDown = false;        // 特殊区域减速标志
uint16_t counterSlowDown = 0; // 减速计数器

CaptureInterface captureInterface("/dev/video0");
SerialInterface serialInterface("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_460800);
Detection detection;

int main(int argc, char *argv[])
{
    RoadType roadType = RoadType::BaseHandle;  // 初始化赛道类型
    ControlCenterCal controlCenterCal;         // 控制中心计算
    MotionController motionController;         // 运动控制
    ImagePreprocess imagePreprocess;           // 图像预处理类
    TrackRecognition trackRecognition;         // 赛道识别
    RingRecognition ringRecognition;           // 环岛识别
    CrossroadRecognition crossroadRecognition; // 十字道路处理
    BridgeDetection bridgeDetection;           // 桥梁检测
    GarageRecognition garageRecognition;       // 车库检测
    SlowZoneDetection slowZoneDetection;       // 慢行区检测
    DepotDetection depotDetection;             // 车辆维修区检测
    FarmlandAvoidance farmlandAvoidance;       // 农田断路区检测
    uint16_t counterRunBegin = 1;              // 智能车启动计数器：等待摄像头图像帧稳定
    uint16_t counterOutTrackA = 0;             // 车辆冲出赛道计数器A
    uint16_t counterOutTrackB = 0;             // 车辆冲出赛道计数器B

    if(motionController.params.SaveImage)
    {
        std::string folderPath = "../res/train/";  // 替换为你要清空的文件夹路径
        ClearFolder(folderPath);
    }

    // PPNC初始化
    if (!detection.init("../res/model/yolov3_mobilenet_v1")) // AI推理初始化
        return 1;

    ipm.init(Size(COLSIMAGE, ROWSIMAGE),
             Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM逆透视变换初始化

    // 下位机初始化通信
    int ret = serialInterface.open();
    if (ret != 0)
        return 0;
    if (motionController.params.CloseLoop)
    {
        cout << "-------- 速度闭环控制 -------" << endl;
        serialInterface.set_PID(motionController.params.Kp, motionController.params.Ki, motionController.params.Kd);
        cout << "Kp = " << motionController.params.Kp << endl;
        cout << "Ki = " << motionController.params.Ki << endl;
        cout << "Kd = " << motionController.params.Kd << endl;
    }
    else
    {
        serialInterface.set_PID(0, 0, 0);
        cout << "-------- 速度开环控制 -------" << endl;
    }
    serialInterface.Start();
    captureInterface.Start();
    detection.Start();

    signal(SIGINT, callbackSignal); // 程序退出信号

    imagePreprocess.imageCorrecteInit(); // 图像矫正参数初始化
    trackRecognition.rowCutUp = motionController.params.rowCutUp;
    trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
    // garageRecognition.disGarageEntry = motionController.params.disGarageEntry;

    /*****出入库使能******/
    if (motionController.params.GarageEnable)
    {
        roadType = RoadType::GarageHandle;          // 初始赛道元素为出库
        garageRecognition.garage_reset();       	// 出入库状态机，初始为出库准备
    }

    /*****调试模式初始化图像窗口*****/
    if (motionController.params.Debug)
    {
        displayWindowInit();
    }

    cout << "等待发车!!!" << endl;

    while (motionController.params.Debug)
    {
        Mat frame;
        frame = captureInterface.get_frame();
        imshow("imageTrack", frame);
        char key = waitKey(5);
        if (key == 13) // 回车
        {
            cout << "即将发车！！！" << endl;
            break;
        }
    }
    for (int i = 3; i > 0; i--) // 3秒后发车
    {
        cout << "------------- " << i << " -----------" << endl;
        serialInterface.set_control(0, PWMSERVOMID); // 智能车停止运动|建立下位机通信
        waitKey(1000);
    }
    cout << "--------- System start!!! -------" << endl;

    while (1)
    {
        // 特殊赛道图像显示标志
        bool imshowRec = false;

        // 处理帧时长监测 速度监测
        // {
            static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
            auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
            float detFPS = (float)1000.f / (startTime - preTime);
            float speed = serialInterface.get_speed();
            cout << "run frame time : " << startTime - preTime << "ms  "
                 << "FPS: " << (int)detFPS << "\t";
            cout << speed << "m/s  " << roadType << endl;
            preTime = startTime;
        // }

        Mat frame;
        frame = captureInterface.get_frame();
        detection.setFrame(frame);

        /*1.AI推理*/
        bool AI_enable = detection.AI_Enable();
        std::shared_ptr<DetectionResult> ai_results = nullptr;
        if (roadType > 2) AI_enable = true;
        if (AI_enable)
        {
            ai_results = detection.getLastFrame();
            frame = ai_results->rgb_frame;
        }

        /*2.图像预处理*/
        // Mat imgaeCorrect = imagePreprocess.imageCorrection(frame);         // RGB
        Mat imgaeCorrect = frame;                                          // RGB
        Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

        /*3.基础赛道识别*/
        trackRecognition.trackRecognition(imageBinary); // 赛道线识别
        Mat imageTrack = imgaeCorrect.clone();          // RGB

		/*******4.出库和入库识别与路径规划*********/
		if (motionController.params.GarageEnable && AI_enable) // 出库元素是否使能开启，根据配置文件得到
		{
			// 道路类型在车库或者基础赛道，进行相关处理
			if (roadType == RoadType::GarageHandle || roadType == RoadType::BaseHandle)
			{
				// 进行斑马线检测，检测到斑马线，清空其他状态机
				if (garageRecognition.garage_contral(ai_results->predictor_results, trackRecognition))
				{
					bridgeDetection.reset(); // 桥梁
					depotDetection.reset();	 // 维修
					// farmlandDetection.reset();				// 农田区域
					// granaryDetection.reset();				// 粮仓
					slowZoneDetection.reset();	  // 慢行区
					crossroadRecognition.reset(); // 十字道路
					// freezoneRecognition.reset(); 			// 泛行区
					ringRecognition.reset(); // 环岛

					if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
						serialInterface.buzzerSound(1);	  // OK

					roadType = RoadType::GarageHandle;
				}
				else
					roadType = RoadType::BaseHandle;

				if (garageRecognition.get_now_value())
				{
					// 蜂鸣器提示，已经入库
					serialInterface.buzzerSound(1);
					cout << ">>>>>>>   入库结束 !!!!!" << endl;
					callbackSignal(0);
				}

				// 在调试模式下，如果有检测到斑马线，显示图像
				if (roadType == RoadType::GarageHandle && motionController.params.Debug)
				{
					Mat imageGarage = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
					garageRecognition.drawImage(trackRecognition, imageGarage);
					imshow("imageRecognition", imageGarage); // 添加需要显示的图像
					imshowRec = true;
				}
			}
		}

		/*农田区域检测*/
		if (motionController.params.FarmlandEnable && AI_enable) // 赛道元素是否使能
		{
			if (roadType == RoadType::FarmlandHandle || roadType == RoadType::BaseHandle)
			{
				if (farmlandAvoidance.farmlandAvoid(trackRecognition, ai_results->predictor_results, imgaeCorrect))
				{
					if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
						serialInterface.buzzerSound(1);	  // OK

					roadType = RoadType::FarmlandHandle;
					if (motionController.params.Debug)
					{
						Mat imageFarmland = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
						farmlandAvoidance.drawImage(trackRecognition, imageFarmland);
						imshow("imageRecognition", imageFarmland); // 添加需要显示的图像
						imshowRec = true;
					}
				}
				else
					roadType = RoadType::BaseHandle;
			}
		}


        /*维修厂检测*/
        if (motionController.params.DepotEnable && AI_enable) // 赛道元素是否使能
        {
            if (roadType == RoadType::DepotHandle || roadType == RoadType::BaseHandle)
            {
                if (depotDetection.depotDetection(trackRecognition, ai_results->predictor_results))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                        serialInterface.buzzerSound(1);   // OK

                    roadType = RoadType::DepotHandle;
                    if (motionController.params.Debug)
                    {
                        Mat imageDepot = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
                        depotDetection.drawImage(trackRecognition, imageDepot);
                        imshow("imageRecognition", imageDepot); // 添加需要显示的图像
                        imshowRec = true;
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        /*坡道（桥）检测与路径规划*/
        if (motionController.params.BridgeEnable && AI_enable) // 赛道元素是否使能
        {
            if (roadType == RoadType::BridgeHandle || roadType == RoadType::BaseHandle)
            {
                if (bridgeDetection.bridgeDetection(trackRecognition, ai_results->predictor_results))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                        serialInterface.buzzerSound(1);   // OK

                    roadType = RoadType::BridgeHandle;
                    if (motionController.params.Debug)
                    {
                        Mat imageBridge = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
                        bridgeDetection.drawImage(trackRecognition, imageBridge);
                        imshow("imageRecognition", imageBridge);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        /*动物出没，慢行区检测*/
        if (motionController.params.SlowzoneEnable && AI_enable) // 赛道元素是否使能
        {
            if (roadType == RoadType::SlowzoneHandle || roadType == RoadType::BaseHandle)
            {
                if (slowZoneDetection.slowZoneDetection(trackRecognition, ai_results->predictor_results))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                        serialInterface.buzzerSound(1);   // OK

                    roadType = RoadType::SlowzoneHandle;
                    if (motionController.params.Debug)
                    {
                        Mat imageSlow = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
                        slowZoneDetection.drawImage(trackRecognition, imageSlow);
                        imshow("imageRecognition", imageSlow);
                        imshowRec = true;
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        /*环岛识别与处理*/
        if (motionController.params.RingEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::RingHandle || roadType == RoadType::BaseHandle)
            {
                if (ringRecognition.ringRecognition(trackRecognition, imageBinary))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                        serialInterface.buzzerSound(1);   // OK
                    roadType = RoadType::RingHandle;

                    if (motionController.params.Debug)
                    {
                        Mat imageRing = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
                        ringRecognition.drawImage(trackRecognition, imageRing);
                        imshow("imageRecognition", imageRing);
                    }
                    ringRecognition.print();
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        // /*十字赛道*/
        if (motionController.params.CrossEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::CrossHandle || roadType == RoadType::BaseHandle)
            {
                if (crossroadRecognition.crossroadRecognition(trackRecognition))
                {
                    roadType = RoadType::CrossHandle;
                    if (motionController.params.Debug)
                    {
                        Mat imageCross = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
                        crossroadRecognition.drawImage(trackRecognition, imageCross);
                        imshow("imageRecognition", imageCross);
                    }
                }
                else
                {
                    roadType = RoadType::BaseHandle;
                }
            }
        }

        /*控制中心计算*/
        if (trackRecognition.pointsEdgeLeft.size() < 60 &&
            trackRecognition.pointsEdgeRight.size() < 60 &&
            motionController.params.StopEnable &&
            roadType != RoadType::BridgeHandle &&
            roadType != RoadType::GranaryHandle &&
            roadType != RoadType::DepotHandle &&
            roadType != RoadType::FarmlandHandle) // 防止车辆冲出赛道
        {
            counterOutTrackA++;
            counterOutTrackB = 0;
            if (counterOutTrackA > 20)
                callbackSignal(0);
        }
        else
        {
            counterOutTrackB++;
            if (counterOutTrackB > 50)
            {
                counterOutTrackA = 0;
                counterOutTrackB = 50;
            }
        }
        controlCenterCal.controlCenterCal(trackRecognition); // 根据赛道边缘信息拟合运动控制中心

        /*智能车运动控制 通讯*/
        if (counterRunBegin > 30)
        {
            // 智能车方向控制
            motionController.pdController(controlCenterCal.controlCenter); // PD控制器姿态控制

			// 智能车速度控制
			switch (roadType)
			{
				case RoadType::DepotHandle:
				{
					motionController.motorSpeed = depotDetection.get_speed();
					break;
				}
				case RoadType::BridgeHandle:
				{
					motionController.motorSpeed = bridgeDetection.get_speed();
					break;
				}
				case RoadType::GarageHandle:
				{
					motionController.motorSpeed = garageRecognition.get_speed();
					break;
				}
				case RoadType::FarmlandHandle:
				{
					motionController.motorSpeed = farmlandAvoidance.get_speed();
					break;
				}
				case RoadType::RingHandle:
				{
					motionController.motorSpeed = 1.2;
					break;
				}
				case RoadType::SlowzoneHandle:
				{
					motionController.motorSpeed = 0.8f;
					break;
				}
				default:
				{
					// 智能车变速度控制
                    static bool AI_Last = false;
                    static float speed = 0.0f;
                    if(!AI_Last && AI_enable)
                        speed = 0.0f;
                    else if(AI_Last && !AI_enable)
                        speed = motionController.params.speedAI / 2;

					motionController.speedController(true, controlCenterCal);
                    float speed_ctrl = motionController.motorSpeed;
					if(AI_enable)
					{
                        speed += 0.02f;
                        if(speed > motionController.params.speedAI)
                            speed = motionController.params.speedAI;
					}
                    else
                    {
                        speed += 0.1f;
                        if(speed > speed_ctrl);
                            speed = speed_ctrl;
                    }
                    motionController.motorSpeed = speed;
                    AI_Last = AI_enable;
					break;
				}
			}

            // 串口通信，姿态与速度控制
            serialInterface.set_control(motionController.motorSpeed, motionController.servoPwm);
        }
        else
        {
            counterRunBegin++;
        }

        /*图像显示*/
        if (motionController.params.Debug)
        {
            Mat imageAI = frame.clone();
            if(AI_enable)
            {
                detection.drawbox(imageAI, ai_results->predictor_results);
            }
            controlCenterCal.drawImage(trackRecognition, imageTrack); // 绘制中线
            trackRecognition.drawImage(imageTrack);                   // 图像显示赛道线识别结果
            imshow("imageTrack", imageTrack);
            imshow("imageAI", imageAI);

            {
                char key = waitKey(1);
                if (key == 32) // 空格
                {
                    serialInterface.set_control(0, motionController.servoPwm);
                    key = waitKey(0);
                }
                if (key == 13) // 回车
                {
                    callbackSignal(0);
                }
            }
        }
		// 存图使能
		if (motionController.params.SaveImage)
		{
            // savePicture(frame, roadType);
            switch(roadType)
            {
            case RoadType::BaseHandle:
                trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果
                if(AI_enable)
                {
                    detection.drawbox(imageTrack, ai_results->predictor_results);
                }
                break;
            case RoadType::RingHandle:
                ringRecognition.drawImage(trackRecognition, imageTrack);
                break;
            case RoadType::CrossHandle:
                crossroadRecognition.drawImage(trackRecognition, imageTrack);
                break;
            case RoadType::BridgeHandle:
                bridgeDetection.drawImage(trackRecognition, imageTrack);
                break;
            case RoadType::DepotHandle:
                depotDetection.drawImage(trackRecognition, imageTrack);
                break;
            case RoadType::FarmlandHandle:
                farmlandAvoidance.drawImage(trackRecognition, imageTrack);
                break;
            case RoadType::GarageHandle:
                garageRecognition.drawImage(trackRecognition, imageTrack);
                break;
            case RoadType::SlowzoneHandle:
                slowZoneDetection.drawImage(trackRecognition, imageTrack);
                break;
            }
            // 绘制中心点集
            for (int i = 0; i < controlCenterCal.centerEdge.size(); i++)
            {
                circle(imageTrack, Point(controlCenterCal.centerEdge[i].y, controlCenterCal.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
            }
            putText(imageTrack,
                    "FPS: " + formatDoble2String(detFPS, 2),
                    Point(20, 20), FONT_HERSHEY_PLAIN, 1,
                    Scalar(0, 0, 255), 1); // 车速
            savePicture(imageTrack, roadType);
		}
    }

    serialInterface.set_control(0, PWMSERVOMID); // 智能车停止运动
    detection.Stop();
    captureInterface.Stop();
    serialInterface.Stop();
    destroyAllWindows();

    return 0;
}

/**
 * @brief 系统信号回调函数：系统退出
 * @param signum 信号量
 */
void callbackSignal(int signum)
{
    serialInterface.set_control(0, PWMSERVOMID); // 智能车停止运动
    detection.Stop();
    captureInterface.Stop();
    serialInterface.Stop();
    cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
    exit(signum);
}

/**
 * @brief OpenCV图像显示窗口初始化（详细参数/Debug模式）
 */
void displayWindowInit(void)
{
    //[1] 赛道边缘图像：RGB
    string windowName = "imageTrack";
    cv::namedWindow(windowName, WINDOW_NORMAL);         // 图像名称
    cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE); // 分辨率
    cv::moveWindow(windowName, 1000, 50);               // 布局位置

    //[2] AI图像/矫正后：RGB
    windowName = "imageAI";
    cv::namedWindow(windowName, WINDOW_NORMAL);         // 图像名称
    cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE); // 分辨率
    cv::moveWindow(windowName, 1340, 50);               // 布局位置

    //[3] 二值化图像
    windowName = "imageRecognition";
    cv::namedWindow(windowName, WINDOW_NORMAL);         // 图像名称
    cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE); // 分辨率
    cv::moveWindow(windowName, 1000, 360);              // 布局位置
}

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image, RoadType roadType)
{
    // 存图
    string name = ".jpg";
    static int counter = 0;
    counter++;
    string img_path = "../res/train/";
    name = img_path + to_string(counter) + "-" + to_string(roadType) + ".jpg";
    imwrite(name, image);
}


void ClearFolder(const std::string& folderPath) {
    std::string command = "rm -rf " + folderPath + "/*";
    int result = std::system(command.c_str());
    if (result == 0) {
        std::cout << "文件夹已成功清空。\n";
    } else {
        std::cerr << "清空文件夹时发生错误。\n";
    }
}

/**
 * @brief 车辆减速使能
 */
void slowDownEnable(void)
{
    slowDown = true;
    counterSlowDown = 0;
}
