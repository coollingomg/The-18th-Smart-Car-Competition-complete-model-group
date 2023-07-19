
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

int main(int argc, char const *argv[])
{
	// PPNC初始化
	PPNCDetection predict;
	if (!predict.init("../res/model/yolov3_mobilenet_v1_0714")) // AI推理初始化
		return 1;

	// 摄像头初始化
	std::string indexCapture = "/dev/video0";
	VideoCapture capture(indexCapture);
	if (!capture.isOpened())
	{
		std::cout << "can not open video device " << std::endl;
		return 1;
	}
	capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);

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

		imshow("AI", imageAi);
		waitKey(5);
	}
	return 0;
}