#include <iostream>
#include <memory>
#include <string>
#include <ctime>
#include <signal.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstdlib> // 包含system函数的头文件

#include "../include/stop_watch.hpp"
#include "../include/uart.hpp"                //串口通信
#include "./recognize/track_recognition.cpp"  //基础巡线
#include "./recognize/cross_recognition.cpp"  //十字赛道
#include "./recognize/ring_recognize.cpp"     //环岛赛道
#include "./recognize/garage_recognize.cpp"   //车库及斑马线识别类
#include "./detection/farmland_avoidance.cpp" //农田区

#include "image_preprocess.cpp" //图像预处理
#include "controlcenter_cal.cpp"
#include "motion_control.cpp"

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
void savePicture(Mat &image, RoadType roadType, bool flag);
void ClearFolder(const std::string &folderPath);

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
    GarageRecognition garageRecognition;       // 车库检测
    uint16_t counterRunBegin = 1;              // 智能车启动计数器：等待摄像头图像帧稳定
    uint16_t counterOutTrackA = 0;             // 车辆冲出赛道计数器A
    uint16_t counterOutTrackB = 0;             // 车辆冲出赛道计数器B

    if (motionController.params.SaveImage)
    {
        std::string folderPath = "../res/train/"; // 替换为你要清空的文件夹路径
        ClearFolder(folderPath);
    }

    // PPNC初始化
    if (!detection.init(motionController.params.pathModel)) // AI推理初始化
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
        serialInterface.set_PID(motionController.params.Kp_speed, motionController.params.Ki_speed, motionController.params.Kd_speed, 
                                motionController.params.Kp_current, motionController.params.Ki_current, motionController.params.Kd_current);
        cout << "Kp_speed = " << motionController.params.Kp_speed << endl;
        cout << "Ki_speed = " << motionController.params.Ki_speed << endl;
        cout << "Kd_speed = " << motionController.params.Kd_speed << endl;
        cout << "Kp_current = " << motionController.params.Kp_current << endl;
        cout << "Ki_current = " << motionController.params.Ki_current << endl;
        cout << "Kd_current = " << motionController.params.Kd_current << endl;
    }
    else
    {
        serialInterface.set_PID(0, 0, 0, 0, 0, 0);
        cout << "-------- 速度开环控制 -------" << endl;
    }
    serialInterface.Start();
    captureInterface.Start();

    signal(SIGINT, callbackSignal); // 程序退出信号

    imagePreprocess.imageCorrecteInit(); // 图像矫正参数初始化
    trackRecognition.rowCutUp = motionController.params.rowCutUp;
    trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
    // garageRecognition.disGarageEntry = motionController.params.disGarageEntry;

    /*****出入库使能******/
    if (motionController.params.GarageEnable)
    {
        roadType = RoadType::GarageHandle; // 初始赛道元素为出库
        garageRecognition.garage_reset();  // 出入库状态机，初始为出库准备cd
    }

    /*****调试模式初始化图像窗口*****/
    if (motionController.params.Debug)
    {
        displayWindowInit();
    }

    cout << "等待发车!!!" << endl;

    if (motionController.params.Button)
    {
        while (!serialInterface.wait_signal())
            ; // 串口接收下位机-比赛开始信号
    }
    else if (motionController.params.Debug)
    {
        while (1)
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
    }

    for (int i = 3; i > 0; i--) // 3秒后发车
    {
        cout << "------------- " << i << " -----------" << endl;
        serialInterface.set_control(0, PWMSERVOMID); // 智能车停止运动|建立下位机通信
        waitKey(1000);
    }
    detection.Start();
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
        // if (controlCenterCal.style != "STRIGHT")
        //     AI_enable = false;
        // if (/*roadType == 6 || */ roadType == 9)
        //     AI_enable = true;
        // else if (roadType == 1)
        //     AI_enable = false;
        if(roadType == RoadType::GranaryHandle && granaryDetection.granaryType == 0)
            detection.Startdetect = true;
        else if (roadType && roadType != RoadType::CrossHandle)
            detection.Startdetect = false;
        else
            detection.Startdetect = true;

        // AI_enable = false;
        // if (AI_enable)
        // {
        //     ai_results = detection.getLastFrame();
        //     frame = ai_results->rgb_frame;
        // }
        // else
        {
            ai_results = std::make_shared<DetectionResult>();
        }

        /*2.图像预处理*/
        // Mat imgaeCorrect = imagePreprocess.imageCorrection(frame);         // RGB
        Mat imgaeCorrect = frame.clone(); // RGB
        Mat imageBinary;
        if (AI_enable || roadType == RoadType::DepotHandle || roadType == RoadType::FarmlandHandle)
            imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect, false); // Gray
        else
            imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

        /*3.基础赛道识别*/
        trackRecognition.trackRecognition(imageBinary, 0); // 赛道线识别
        Mat imageTrack = imgaeCorrect.clone();             // RGB

        /*******4.出库和入库识别与路径规划*********/
        if (motionController.params.GarageEnable) // 出库元素是否使能开启，根据配置文件得到
        {
            // 道路类型在车库或者基础赛道，进行相关处理
            if (roadType == RoadType::GarageHandle || roadType == RoadType::BaseHandle)
            {
                // 进行斑马线检测，检测到斑马线，清空其他状态机
                if (garageRecognition.garage_contral(trackRecognition))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                    {
                        serialInterface.buzzerSound(1); // OK

                        // bridgeDetection.reset(); // 桥梁
                        // depotDetection.reset();	 // 维修
                        // farmlandAvoidance.reset();	// 农田区域
                        // slowZoneDetection.reset();	  // 慢行区
                        crossroadRecognition.reset(); // 十字道路
                        ringRecognition.reset();      // 环岛
                    }

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
        if (motionController.params.FarmlandEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::FarmlandHandle || roadType == RoadType::BaseHandle)
            {
                // if (farmlandAvoidance.farmlandAvoid(trackRecognition, ai_results->predictor_results, imgaeCorrect))
                if (farmlandAvoidance.farmlandAvoid(trackRecognition, imgaeCorrect))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                        serialInterface.buzzerSound(1);   // OK

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
                {
                    if(roadType != RoadType::BaseHandle)
                    {
                        roadType = RoadType::BaseHandle;
                        AI_enable = false;
                    }
                }
            }
        }

        /*维修厂检测*/
        if (motionController.params.DepotEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::DepotHandle || roadType == RoadType::BaseHandle)
            {
                if (depotDetection.depotDetection(trackRecognition, imgaeCorrect))
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
                {
                    if(roadType != RoadType::BaseHandle)
                    {
                        roadType = RoadType::BaseHandle;
                        AI_enable = false;
                    }
                }
            }
        }

        /*粮仓检测*/
        if (motionController.params.GranaryEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::GranaryHandle || roadType == RoadType::BaseHandle)
            {
                if (granaryDetection.granaryDetection(trackRecognition, imgaeCorrect))
                {
                    if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
                        serialInterface.buzzerSound(1);   // OK

                    roadType = RoadType::GranaryHandle;
                    if (motionController.params.Debug)
                    {
                        Mat imageGranary = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
                        granaryDetection.drawImage(trackRecognition, imageGranary);
                        imshow("imageRecognition", imageGranary); // 添加需要显示的图像
                        imshowRec = true;
                    }
                }
                else
                {
                    if(roadType != RoadType::BaseHandle)
                    {
                        roadType = RoadType::BaseHandle;
                        AI_enable = false;
                    }
                }
            }
        }

        /*坡道（桥）检测与路径规划*/
        if (motionController.params.BridgeEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::BridgeHandle || roadType == RoadType::BaseHandle)
            {
                if (bridgeDetection.bridgeDetection(trackRecognition))
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
        if (motionController.params.SlowzoneEnable) // 赛道元素是否使能
        {
            if (roadType == RoadType::SlowzoneHandle || roadType == RoadType::BaseHandle)
            {
                if (slowZoneDetection.slowZoneDetection(trackRecognition))
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

        /**************智能车运动控制,通讯******************/
        if (counterRunBegin > 30)
        {
            // // 智能车方向控制
            motionController.Angle_Controller(controlCenterCal, trackRecognition, roadType);

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
                motionController.motorSpeed = farmlandAvoidance.get_speed(motionController.motorSpeed);
                break;
            }
            case RoadType::GranaryHandle:
            {
                motionController.motorSpeed = granaryDetection.get_speed(motionController.motorSpeed);
                break;
            }
            // case RoadType::RingHandle:
            // {
            //     motionController.speedControl(controlCenterCal);
            // 	motionController.motorSpeed = ringRecognition.get_speed(motionController.motorSpeed);
            // 	break;
            // }
            default:
            {
                // 智能车变速度控制
                if (AI_enable)
                {
                    motionController.motorSpeed -= 0.1f;
                    if (motionController.motorSpeed < motionController.params.speedAI)
                        motionController.motorSpeed = motionController.params.speedAI;
                }
                else
                {
                    // motionController.speedController(controlCenterCal, motionController.k);
                    motionController.speedControl(controlCenterCal);
                    // motionController.motorSpeed = motionController.params.speedLow;
                }
                break;
            }
            }

            // 串口通信，姿态与速度控制
            // motionController.motorSpeed = 1.2f;
            serialInterface.set_control(-motionController.motorSpeed, motionController.servoPwm);
        }
        else
        {
            counterRunBegin++;
        }

        // 存图使能
        if (motionController.params.SaveImage)
        {
            // savePicture(frame, roadType);
            switch (roadType)
            {
            case RoadType::BaseHandle:
                trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果
                if (AI_enable)
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
            case RoadType::GranaryHandle:
                granaryDetection.drawImage(trackRecognition, imageTrack);
                break;
            }
            // 绘制中心点集
            for (int i = 0; i < controlCenterCal.centerEdge.size(); i++)
            {
                circle(imageTrack, Point(controlCenterCal.centerEdge[i].y, controlCenterCal.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
            }
            rectangle(imageTrack, Rect(controlCenterCal.intersectionLeft.y, controlCenterCal.intersectionLeft.x, 10, 10), Scalar(0, 0, 200), 1);
            rectangle(imageTrack, Rect(controlCenterCal.intersectionRight.y, controlCenterCal.intersectionRight.x, 10, 10), Scalar(0, 0, 200), 1);
            // putText(imageTrack, to_string(ringRecognition.counterShield), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);

            putText(imageTrack, "FPS: " + formatDoble2String(detFPS, 2), Point(20, 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);                   // 车速
            putText(imageTrack, "PWM:" + formatDoble2String(motionController.servoPwm, 2), Point(20, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 下发的pwm值
            putText(imageTrack, "ERROR:" + formatDoble2String(motionController.error, 2), Point(20, 60), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, "K:" + formatDoble2String(motionController.CenterLine_k, 4), Point(20, 80), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, "Mid:" + formatDoble2String(motionController.Mid_line, 2), Point(20, 100), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, "Speed: " + formatDoble2String(motionController.motorSpeed, 2), Point(COLSIMAGE / 2 + 20, 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, controlCenterCal.style, Point(COLSIMAGE / 2 + 20, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型
            putText(imageTrack, "CERROR: " + formatDoble2String(motionController.compensation_error, 2), Point(COLSIMAGE / 2 + 20, 60), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, "PreSlope: " + formatDoble2String(motionController.Slope_previewPoint, 2), Point(COLSIMAGE / 2 + 20, 80), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, "Sigma: " + formatDoble2String(controlCenterCal.sigmaCenter, 2), Point(COLSIMAGE / 2 + 20, 100), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
            putText(imageTrack, "Plan: " + to_string(motionController.speed_plan), Point(COLSIMAGE / 2 + 20, 120), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

            line(imageTrack, Point(motionController.Mid_line, 0), Point(motionController.Mid_line, ROWSIMAGE - 1), Scalar(200, 200, 200), 1);
            // line(imageTrack, Point(0, ROWSIMAGE - motionController.params.Control_Up_set), Point(COLSIMAGE - 1, ROWSIMAGE - motionController.params.Control_Up_set), Scalar(200, 200, 200), 1);
            // line(imageTrack, Point(0, ROWSIMAGE - motionController.params.Control_foreword_down), Point(COLSIMAGE - 1, ROWSIMAGE - motionController.params.Control_foreword_down), Scalar(255, 0, 0), 1);
            // line(imageTrack, Point(0, ROWSIMAGE - motionController.params.Control_foreword_up), Point(COLSIMAGE - 1, ROWSIMAGE - motionController.params.Control_foreword_up), Scalar(255, 0, 0), 1);
            savePicture(imageTrack, roadType, AI_enable);
        }

        /**********图像显示************/
        if (motionController.params.Debug)
        {
            imshow("imageTrack", imageTrack);
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
    serialInterface.set_control(0, PWMSERVOMID);                // 智能车停止运动
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 延时20ms确保停止信号发出

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
    // windowName = "imageAI";
    // cv::namedWindow(windowName, WINDOW_NORMAL);         // 图像名称
    // cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE); // 分辨率
    // cv::moveWindow(windowName, 1340, 50);               // 布局位置

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
void savePicture(Mat &image, RoadType roadType, bool flag)
{
    // 存图
    string name = ".jpg";
    static int counter = 0;
    counter++;
    string img_path = "../res/train/";
    name = img_path + to_string(counter) + "-" + to_string(roadType) + "_" + to_string(flag) + ".jpg";
    imwrite(name, image);
}

void ClearFolder(const std::string &folderPath)
{
    std::string command = "rm -rf " + folderPath + "/*";
    int result = std::system(command.c_str());
    if (result == 0)
    {
        std::cout << "文件夹已成功清空。\n";
    }
    else
    {
        std::cerr << "清空文件夹时发生错误。\n";
    }
}
