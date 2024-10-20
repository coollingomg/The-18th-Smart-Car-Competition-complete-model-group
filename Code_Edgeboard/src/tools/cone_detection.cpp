
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include "../include/common.hpp"	//公共类方法文件

using namespace std;
using namespace cv;

std::vector<cv::Rect> searchCones(cv::Mat img_rgb, cv::Mat &mask);
void onTrackbar(int, void*);

int lowB = 0, lowG = 100, lowR = 100;
int highB = 100, highG = 255, highR = 255;
int kernel_size = 3;

// 设置锥桶颜色的RGB范围（黄色）
// cv::Scalar lowerThreshold(0, 100, 100);
// cv::Scalar upperThreshold(100, 255, 255);
cv::Scalar lowerThreshold(lowB, lowG, lowR);
cv::Scalar upperThreshold(highB, highG, highR);

bool colorThresholdSelected = true;

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

    // 创建一个空白图像作为"Threshold Settings"窗口的背景
    cv::Mat thresholdSettings = cv::Mat::ones(1, 480, CV_8UC3) * 255;
    // 创建“Threshold Settings”窗口
    cv::namedWindow("Threshold Settings");
	cv::resizeWindow("Threshold Settings", 1, 480); // 设置窗口大小为200x1像素
	cv::moveWindow("Threshold Settings", 0, 0); // 移动"Threshold Settings"窗口到图像结果窗口的上方    // 创建“Result”窗口
    cv::namedWindow("Result");
    // 为BGR通道创建滑块
    cv::createTrackbar("Low B", "Threshold Settings", &lowB, 255, onTrackbar);
    cv::createTrackbar("Low G", "Threshold Settings", &lowG, 255, onTrackbar);
    cv::createTrackbar("Low R", "Threshold Settings", &lowR, 255, onTrackbar);
    cv::createTrackbar("High B", "Threshold Settings", &highB, 255, onTrackbar);
    cv::createTrackbar("High G", "Threshold Settings", &highG, 255, onTrackbar);
    cv::createTrackbar("High R", "Threshold Settings", &highR, 255, onTrackbar);
    cv::createTrackbar("Kernel Size", "Threshold Settings", &kernel_size, 10, onTrackbar);


	while (1)
	{
		
		Mat frame;
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}
		Mat image_cone = frame.clone();
		Mat mask;

		if(colorThresholdSelected)
		{
			std::vector<cv::Rect> coneRects = searchCones(image_cone, mask);
			for(const auto& rect : coneRects)
			{
				cv::rectangle(image_cone, rect, cv::Scalar(0, 255, 0), 2);
				circle(image_cone, Point(rect.x + rect.width / 2, rect.y + rect.height / 2), 5, Scalar(200, 200, 200), -1);
			}
			{
				static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
				auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
				float detFPS = (float)1000.f / (startTime - preTime);
				std::cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << "  num: " << coneRects.size() << std::endl;
				preTime = startTime;
			}
			cv::imshow("Result", image_cone);
			cv::imshow("Mask", mask);
        	cv::imshow("Threshold Settings", thresholdSettings);
			if(waitKey(5) == 13) break;
			// 更新滑块显示的当前值
			cv::setTrackbarPos("Low B", "Threshold Settings", lowB);
			cv::setTrackbarPos("Low G", "Threshold Settings", lowG);
			cv::setTrackbarPos("Low R", "Threshold Settings", lowR);
			cv::setTrackbarPos("High B", "Threshold Settings", highB);
			cv::setTrackbarPos("High G", "Threshold Settings", highG);
			cv::setTrackbarPos("High R", "Threshold Settings", highR);
			cv::setTrackbarPos("Kernel Size", "Threshold Settings", kernel_size);
		}
		else
		{
			// 绘制方框在图像中心
            int centerX = image_cone.cols / 2;
            int centerY = image_cone.rows / 2;
            int boxSize = 20; // 方框的大小
            cv::Rect boxRect(centerX - boxSize / 2, centerY - boxSize / 2, boxSize, boxSize);

            // 在方框内提取颜色阈值参
            cv::Mat boxROI = image_cone(boxRect);
            cv::Scalar meanColor = cv::mean(boxROI);
            cv::rectangle(image_cone, boxRect, cv::Scalar(0, 255, 0), 2);

			// 绘制颜色阈值提示框在图像中心左下方
			int roiX = image_cone.cols / 2 - 10;
			int roiY = image_cone.rows / 2 + 10;
			cv::Rect roiRect(roiX- boxSize / 2, roiY - boxSize / 2, boxSize, boxSize);
			cv::rectangle(image_cone, roiRect, cv::Scalar(meanColor[0], meanColor[1], meanColor[2]), -1);

            cv::imshow("Select Color Threshold", image_cone);
			if(cv::waitKey(5) == 13)
			{
				// 设置锥桶颜色的RGB范围
				lowerThreshold = cv::Scalar(meanColor[0] - 10, meanColor[1] - 50, meanColor[2] - 50);
				upperThreshold = cv::Scalar(meanColor[0] + 10, meanColor[1] + 50, meanColor[2] + 50);
				colorThresholdSelected = true;
				cout << "Selected Thresholds: " << lowerThreshold << " - " << upperThreshold << endl;
			}
		}
	}
	return 0;
}

std::vector<cv::Rect> searchCones(cv::Mat img_rgb, cv::Mat &mask)
{
	std::vector<cv::Rect> coneRects;

	// 在RGB图像中根据颜色范围提取锥桶区域
	cv::inRange(img_rgb, lowerThreshold, upperThreshold, mask);

	// 进行形态学操作，去除噪声并提取锥桶区域的轮廓
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
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

void onTrackbar(int, void*)
{
    // 从滑块更新阈值参数
    lowerThreshold = cv::Scalar(lowB, lowG, lowR);
    upperThreshold = cv::Scalar(highB, highG, highR);
}
