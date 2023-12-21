#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "../src/recognize/track_recognition.cpp"

using namespace std;
using namespace cv;

void movingAverageFilter(std::vector<POINT>& data, int windowSize);
void line_extend(std::vector<POINT> &points);

bool compareContoursByCenterY(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2) {
    cv::Moments moments1 = cv::moments(contour1);
    cv::Moments moments2 = cv::moments(contour2);

    cv::Point center1(moments1.m10 / moments1.m00, moments1.m01 / moments1.m00);
    cv::Point center2(moments2.m10 / moments2.m00, moments2.m01 / moments2.m00);

    return center1.y < center2.y;
}

int main(int argc, char *argv[])
{
	TrackRecognition trackRecognition;

    std::string windowName = "frame";
    cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
    cv::resizeWindow(windowName, COLSIMAGE, ROWSIMAGE);     // 分辨率

    /*注意：
      使用 0 和 /dev/video0 的分辨率不同：
        0           : opencv 内部的采集，可能是基于 V4L2, 分辨率：1280 * 960
        /dev/video0 : 基于Gstreamer ， 分辨率：640 * 480
    */
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
    capture.set(cv::CAP_PROP_ZOOM, 14);
    capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.20);  //自动曝光开关

    double rate = capture.get(CAP_PROP_FPS);
    double width = capture.get(CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CAP_PROP_FRAME_HEIGHT);
    std::cout << "Camera Param: frame rate = " << rate << " width = " << width
              << " height = " << height << std::endl;

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

        POINT pointTop = POINT(ROWSIMAGE - 1, 0);
        cv::Mat img_rgb = frame.clone();
        cv::circle(img_rgb, cv::Point(0, ROWSIMAGE - trackRecognition.rowCutBottom), 5, cv::Scalar(20, 200, 200), -1);
        cv::circle(img_rgb, cv::Point(COLSIMAGE - 1, ROWSIMAGE - trackRecognition.rowCutBottom), 5, cv::Scalar(20, 200, 200), -1);
        // 设置锥桶颜色的RGB范围（黄色），提取掩膜
        cv::Mat mask;
        cv::Scalar lowerYellow(0, 100, 100);
        cv::Scalar upperYellow(100, 255, 255);
        cv::inRange(img_rgb, lowerYellow, upperYellow, mask);
        // 进行形态学操作，去除噪声并提取锥桶区域的轮廓
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // std::sort(contours.begin(), contours.end(), compareContoursByCenterY);

        //分离蓝色通道
        std::vector<cv::Mat> channels;
        split(frame, channels);
        cv::Mat blueChannel = channels[0];

        // 创建空白图像作为结果
        cv::Mat resultImage = cv::Mat::zeros(frame.size(), CV_8UC3);
        // 遍历每个轮廓
        std::vector<uint16_t> lined_conters; // 用于记录已经“被”连接的轮廓，每个轮廓只能被连接一次，远点为被连接点
        for (size_t i = 0; i < contours.size(); ++i) 
        {
            bool lined = false;
            // 遍历当前轮廓的点
            for (size_t j = 0; j < contours[i].size(); ++j) 
            {
                // 获取当前轮廓的当前点
                cv::Point currentPoint = contours[i][j];
                if(currentPoint.y < trackRecognition.rowCutUp)
                    continue;

                // 遍历其他轮廓
                for (size_t k = i; k < contours.size(); ++k)
                {
                    // 跳过当前轮廓
                    if (k == i)
                        continue;

                    // 遍历其他轮廓的点
                    for (size_t l = 0; l < contours[k].size(); ++l) 
                    {
                        // 获取其他轮廓的当前点
                        cv::Point otherPoint = contours[k][l];
                        if(otherPoint.y < 50)
                            continue;

                        if(abs(currentPoint.y - otherPoint.y) > 100 || abs(currentPoint.x - otherPoint.x) > 100)
                            continue;

                        // 计算两点之间的距离
                        double distance = cv::norm(currentPoint - otherPoint);
                        double x_dist, y_dist, distThreshold;
                        x_dist = std::max(currentPoint.y, otherPoint.y) * 0.6;
                        y_dist = std::abs(currentPoint.x - otherPoint.x) * (std::max(std::max(currentPoint.y, otherPoint.y), 80) / ROWSIMAGE);
                        distThreshold = x_dist + y_dist;

                        if(distThreshold < 50)
                            distThreshold = 50;

                        uint16_t lined_temp = currentPoint.y < otherPoint.y ? i : k;
                        auto it = std::find(lined_conters.begin(), lined_conters.end(), lined_temp);

                        // 如果距离小于阈值，使用线段将两点连接起来
                        if (distance < distThreshold) 
                        {
                            if(it == lined_conters.end())
                            {
                                lined_conters.push_back(lined_temp);
                                if(currentPoint.y < pointTop.x)
                                    pointTop = POINT(currentPoint.y, currentPoint.x);
                                if(otherPoint.y < pointTop.x)
                                    pointTop = POINT(otherPoint.y, otherPoint.x);
                                cv::line(resultImage, currentPoint, otherPoint, cv::Scalar(0, 255, 0), 5);
                                cv::line(blueChannel, currentPoint, otherPoint, cv::Scalar(60), 5);
                                lined = true;
                            }
                            else
                            {
                                cv::line(resultImage, currentPoint, otherPoint, cv::Scalar(0, 0, 255), 5);
                            }
                            break;
                        }
                    }
                }
                if(lined)
                    break;
            }
        }

		cv::Mat imageBinary, imageEnrode;

        cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));//创建结构元
		cv::Mat kernel_enrode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(40, 40));
		cv::morphologyEx(blueChannel, blueChannel, cv::MORPH_CLOSE, kernel_close, cv::Point(-1, -1));//闭运算
		cv::morphologyEx(blueChannel, imageEnrode, cv::MORPH_ERODE, kernel_enrode, cv::Point(-1, -1));//腐蚀运算
		cv::threshold(imageEnrode, imageBinary, 0, 255, cv::THRESH_OTSU);

        trackRecognition.trackRecognition(imageBinary, 0);
        movingAverageFilter(trackRecognition.pointsEdgeLeft, 10);
        movingAverageFilter(trackRecognition.pointsEdgeRight, 10);

        if(pointTop.y)
        {
            trackRecognition.pointsEdgeLeft.resize(abs(pointTop.x - trackRecognition.pointsEdgeLeft[0].x));
            trackRecognition.pointsEdgeRight.resize(abs(pointTop.x - trackRecognition.pointsEdgeRight[0].x));
        }

        trackRecognition.drawImage(img_rgb);
        cv::circle(img_rgb, Point(pointTop.y, pointTop.x), 5, Scalar(226, 43, 138), -1);
        
        cv::imshow("frame", img_rgb);
        cv::imshow("Mask", mask);
        cv::imshow("lineResult", resultImage);
        cv::imshow("blueChannel", blueChannel);
        cv::imshow("enrode", imageEnrode);
        cv::imshow("enrode_binary", imageBinary);
        cv::waitKey(1);
    }
    capture.release();
}

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

