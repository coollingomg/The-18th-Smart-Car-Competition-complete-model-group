#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "../src/image_preprocess.cpp"
#include "../include/capture.hpp"
#include "../include/stop_watch.hpp"

using namespace std;
using namespace cv;

CaptureInterface captureInterface("/dev/video0");

int main(int argc, char *argv[])
{
	ImagePreprocess imagePreprocess;                // 图像预处理类
    StopWatch watch;

    ipm.init(Size(COLSIMAGE, ROWSIMAGE),
             Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM逆透视变换初始化

    captureInterface.Start();

    watch.tic();
    while (1)
    {
        double frame_time = watch.toc();
        if(frame_time < 50)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        else
        {
            float detFPS = (float)1000.f / frame_time;
            std::cout << "run frame time: " << std::fixed << std::setprecision(3) << frame_time << "ms  ";
            std::cout << "FPS: " << (int)detFPS << std::endl;
            watch.tic();
        }

        Mat frame, remapImg;
        frame = captureInterface.get_frame();
        ipm.homography(frame, remapImg);

        imshow("frame", frame);
        imshow("remap", remapImg);
        
        waitKey(2);
    }
    captureInterface.Stop();
    return 0;
}
