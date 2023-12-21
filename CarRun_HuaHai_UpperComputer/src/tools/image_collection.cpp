#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace std;
using namespace cv;

string generateRandomFileName();

int main(int argc, char *argv[])
{
    std::string windowName = "frame";
    cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
    cv::resizeWindow(windowName, 640, 480);     // 分辨率

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
        imshow("frame", frame);

        char key = waitKey(10);
        if(key == 's')
        {
            static int counter = 5500;
            counter++;
            // string img_path = "/mnt/myUSB/pic/";
            string img_path = "../res/test/";
            string name = img_path + to_string(counter) + ".jpg";
            imwrite(name, frame);
			cout << "OK\t" << counter << endl;
        }
    }
    capture.release();
}

string generateRandomFileName()
{
	time_t now = time(0);
	tm* localtm = localtime(&now);

	char buffer[80];
	strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", localtm);

	srand(time(NULL));
	int randNum = rand() % 10000;

	string fileName = string(buffer) + "_" + to_string(randNum);

	return fileName;
}
