
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include "../include/common.hpp"	//公共类方法文件

using namespace std;
using namespace cv;

std::vector<cv::Rect> searchCones(cv::Mat img_rgb);

int main(int argc, char const *argv[])
{
	
	Mat image;
	std::string indexCapture = "/dev/video0";
	VideoCapture capture("/dev/video0");
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
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}
		Mat image_cone = frame.clone();
		std::vector<cv::Rect> coneRects = searchCones(image_cone);
		for(const auto& rect : coneRects)
		{
			cv::rectangle(image_cone, rect, cv::Scalar(0, 255, 0), 2);
			circle(image_cone, Point(rect.x + rect.width / 2, rect.y + rect.height / 2), 5, Scalar(200, 200, 200), -1);
		}

		cv::imshow("Result", image_cone);

		waitKey(5);
	}
	return 0;
}

std::vector<cv::Rect> searchCones(cv::Mat img_rgb)
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
