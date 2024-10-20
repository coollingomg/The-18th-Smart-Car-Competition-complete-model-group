
#include "../include/common.hpp"	//公共类方法文件
#include "../include/predictor.hpp" //PPNC目标检测
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

#include "../include/predictor.hpp"

using namespace std;
using namespace cv;

int m_exp = 20;
float exposure = (float)m_exp / 10000;
VideoCapture capture("/dev/video0");

void onTrackbar(int, void*)
{
    // 从滑块更新阈值参数
    exposure = (float)m_exp / 10000;
    capture.set(cv::CAP_PROP_EXPOSURE, exposure);
}

int main(int argc, char const *argv[])
{
	// PPNC初始化
	PPNCDetection predict;
	if (!predict.init("../res/model/yolov3_mobilenet_v1_0714")) // AI推理初始化
		return 1;

	// 摄像头初始化
	if (!capture.isOpened())
	{
		std::cout << "can not open video device " << std::endl;
		return 1;
	}
	capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	capture.set(cv::CAP_PROP_FPS, 90);
	capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
	capture.set(cv::CAP_PROP_ZOOM, 12);
	capture.set(cv::CAP_PROP_EXPOSURE, exposure);       //曝光时间

    cv::namedWindow("Result");
    cv::createTrackbar("Exposure: ", "Result", &m_exp, 100, onTrackbar);

	while (1)
	{
		Mat frame;
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}

		{
			static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			float detFPS = (float)1000.f / (startTime - preTime);
			cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << endl;
			preTime = startTime;
		}

		//[00] AI推理
		auto feeds = predict.preprocess(frame, {320, 320});
		predict.run(*feeds);
		// get result
		predict.render();

		Mat imageAi = frame.clone();
		predict.drawBox(imageAi);

		imshow("Result", imageAi);
		if(waitKey(5) == 13)
		{
			double rate = capture.get(CAP_PROP_FPS);
			double width = capture.get(CAP_PROP_FRAME_WIDTH);
			double height = capture.get(CAP_PROP_FRAME_HEIGHT);
			double exposure = capture.get(CAP_PROP_EXPOSURE);
			std::cout << "Camera Param: frame rate = " << rate << " width = " << width
				<< " height = " << height << " exposure = " << exposure << " ms" << std::endl;
			break;
		}

		cv::setTrackbarPos("Exposure: ", "Result", m_exp);
	}
	return 0;
}