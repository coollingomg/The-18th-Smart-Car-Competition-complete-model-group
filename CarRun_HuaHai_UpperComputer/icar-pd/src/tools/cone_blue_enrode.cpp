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
        Mat frame;
        if (!capture.read(frame))
        {
            std::cout << "no video frame" << std::endl;
            continue;
        }

        std::vector<cv::Mat> channels;
        split(frame, channels);
        cv::Mat blueChannel = channels[0];

		Mat imageGray, imageBinary, imageEnrode;

        Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3, 3));//创建结构元
		Mat kernel_enrode = getStructuringElement(MORPH_ELLIPSE, Size(60, 60));
		Mat kernel_open = getStructuringElement(MORPH_ELLIPSE, Size(50, 50));
		morphologyEx(blueChannel, blueChannel, MORPH_CLOSE, kernel_close, Point(-1, -1));//闭运算
		morphologyEx(blueChannel, imageEnrode, MORPH_ERODE, kernel_enrode, Point(-1, -1));//腐蚀运算
		morphologyEx(imageEnrode, imageGray, MORPH_OPEN, kernel_open, Point(-1, -1));//开运算
		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU);

        trackRecognition.trackRecognition(imageBinary, 0);
        movingAverageFilter(trackRecognition.pointsEdgeLeft, 10);
        movingAverageFilter(trackRecognition.pointsEdgeRight, 10);
        // line_extend(trackRecognition.pointsEdgeLeft);
        // line_extend(trackRecognition.pointsEdgeRight);
        // uint16_t size = MIN(trackRecognition.pointsEdgeLeft.size(), trackRecognition.pointsEdgeRight.size());
        // for(int i = 0; i < size; i++)
        // {
        //     uint16_t width = trackRecognition.pointsEdgeRight[i].y - trackRecognition.pointsEdgeLeft[i].y;
        //     if(width > COLSIMAGE / 10)
        //         trackRecognition.widthBlock.push_back(POINT(i, width));
        //     else
        //     {
        //         trackRecognition.pointsEdgeRight.resize(i);
        //         trackRecognition.pointsEdgeLeft.resize(i);
        //         break;
        //     }
        // }

        trackRecognition.drawImage(frame);
        
        imshow("frame", frame);
        imshow("blueChannel", blueChannel);
        imshow("enrode", imageEnrode);
        imshow("open", imageGray);
        imshow("enrode_binary", imageBinary);
        waitKey(5);
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

