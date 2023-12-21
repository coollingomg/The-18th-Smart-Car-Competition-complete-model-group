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
    {
        // 获取摄像头参数范围
        double minBrightness = capture.get(cv::CAP_PROP_BRIGHTNESS);
        double maxBrightness = capture.get(cv::CAP_PROP_BRIGHTNESS + 1);

        double minContrast = capture.get(cv::CAP_PROP_CONTRAST);
        double maxContrast = capture.get(cv::CAP_PROP_CONTRAST + 1);

        double minSaturation = capture.get(cv::CAP_PROP_SATURATION);
        double maxSaturation = capture.get(cv::CAP_PROP_SATURATION + 1);

        double minSharpness = capture.get(cv::CAP_PROP_SHARPNESS);
        double maxSharpness = capture.get(cv::CAP_PROP_SHARPNESS + 1);

        double minGain = capture.get(cv::CAP_PROP_GAIN);
        double maxGain = capture.get(cv::CAP_PROP_GAIN + 1);

        std::cout << "亮度范围：" << minBrightness << " - " << maxBrightness << std::endl;
        std::cout << "对比度范围：" << minContrast << " - " << maxContrast << std::endl;
        std::cout << "饱和度范围：" << minSaturation << " - " << maxSaturation << std::endl;
        std::cout << "清晰度范围：" << minSharpness << " - " << maxSharpness << std::endl;
        std::cout << "增益范围：" << minGain << " - " << maxGain << std::endl;
    }
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FPS, 90);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
    capture.set(cv::CAP_PROP_ZOOM, 12);
    // capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.15);  //自动曝光开关
	capture.set(cv::CAP_PROP_EXPOSURE, 0.002);
    {
        // capture.set(cv::CAP_PROP_BRIGHTNESS, 1);    //亮度
        // capture.set(cv::CAP_PROP_CONTRAST, 1);      //对比度
        // capture.set(cv::CAP_PROP_SATURATION, 0.75);    //饱和度
        // capture.set(cv::CAP_PROP_SHARPNESS, 3.6);     //清晰度
        // capture.set(cv::CAP_PROP_GAIN, 0.5);          //增益
    }
    // capture.set(cv::CAP_PROP_PAN, 20);
    // capture.set(cv::CAP_PROP_XI_OFFSET_X, 1);
    // capture.set(cv::CAP_PROP_XI_OFFSET_Y, 1);

    double rate = capture.get(CAP_PROP_FPS);
    double width = capture.get(CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CAP_PROP_FRAME_HEIGHT);
	double exposure = capture.get(CAP_PROP_EXPOSURE);
	std::cout << "Camera Param: frame rate = " << rate << " width = " << width
			<< " height = " << height << " exposure = " << exposure << " ms" << std::endl;

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
