#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "../src/image_preprocess.cpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	ImagePreprocess imagePreprocess;                // 图像预处理类

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
    capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);

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
        Mat imgbinary = imagePreprocess.imageBinaryzation(frame);

        // 绘制田字格：基准线
        uint16_t rows = ROWSIMAGE / 30; // 16
        uint16_t cols = COLSIMAGE / 32; // 20

        for (size_t i = 1; i < rows; i++)
        {
            line(frame, Point(0, 30 * i), Point(frame.cols - 1, 30 * i), Scalar(211, 211, 211), 1);
        }
        for (size_t i = 1; i < cols; i++)
        {
            if (i == cols / 2)
                line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1), Scalar(0, 0, 255), 1);
            else
                line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1), Scalar(211, 211, 211), 1);
        }
        
        imshow("frame", frame);
        imshow("binary", imgbinary);
        waitKey(10);
    }
    capture.release();
}
