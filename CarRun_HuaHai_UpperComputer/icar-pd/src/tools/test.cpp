
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

#include "../include/detection.hpp"

using namespace std;
using namespace cv;

CaptureInterface captureInterface("/dev/video0");
Detection detection;

int main(int argc, char const *argv[])
{
	// PPNC初始化
	if (!detection.init("../res/model/yolov3_mobilenet_v1")) // AI推理初始化
		return 1;
	detection.Start();

	// 摄像头初始化
	captureInterface.Start();

	while (1)
	{
		{
			static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			float detFPS = (float)1000.f / (startTime - preTime);
			cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << endl;
			preTime = startTime;
		}

		Mat frame;
		frame = captureInterface.get_frame();
		detection.setFrame(frame);

		waitKey(30);

		std::shared_ptr<DetectionResult> result = detection.getLastFrame();
		Mat imageAi = result->rgb_frame;
		detection.drawbox(imageAi, result->predictor_results);

		imshow("AI", imageAi);
	}
	return 0;
}