#pragma once

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "uart.hpp"
#include "common.hpp"
#include "stop_watch.hpp"

class CaptureInterface
{
public:
    CaptureInterface(std::string camera_path = "/dev/video0") 
    : _row(ROWSIMAGE)
    , _col(COLSIMAGE)
    , _rate(60)
    {
        _camera_path = camera_path;
    }
    ~CaptureInterface() {}

    void Start()
    {
        _loop = true;
        run();
    }

    void Stop()
    {
        _loop = false;
        _thread->join();
        if(_cap->isOpened())
        {
            _cap->release();
        }
    }

    void run()
    {
        int ret = _open();
        if(ret != 0)
        {
            exit(-1);
        }
        _cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        _cap->set(cv::CAP_PROP_FPS, _rate);
        _cap->set(cv::CAP_PROP_FRAME_WIDTH, _col);
        _cap->set(cv::CAP_PROP_FRAME_HEIGHT, _row);
        CheckCap();

        _thread = std::make_unique<std::thread>([this](){
            while(_loop)
            {
                std::shared_ptr<cv::Mat> frame = std::make_shared<cv::Mat>(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3);
                *_cap >> *frame;
                if(frame->empty())
                {
                    std::cout << "faild to capture frame" << std::endl;
                    break;
                }

                std::unique_lock<std::mutex> lock(_mutex);
                _frame = frame;
                cond_.notify_all();
            }
        });
    }

    cv::Mat get_frame(void)
    {
        cv::Mat frame;
        std::unique_lock<std::mutex> lock(_mutex);
        while(_frame == nullptr)
        {
            cond_.wait(lock);
        }
        frame = _frame->clone();
        _frame = nullptr;
        return frame;
    }

    void CheckCap(void)
    {
        double rate = _cap->get(CAP_PROP_FPS);
        double width = _cap->get(CAP_PROP_FRAME_WIDTH);
        double height = _cap->get(CAP_PROP_FRAME_HEIGHT);
        std::cout << "Camera Param: frame rate = " << rate << " width = " << width
                << " height = " << height << std::endl;
    }

    void SetCap(uint16_t row, uint16_t col, uint16_t rate, std::string format_out)
    {
        if(format_out == "MJPG");
        {
            _cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        }
        _row = row;
        _col = col;
        _rate = rate;
        _cap->set(cv::CAP_PROP_FPS, _rate);
        _cap->set(cv::CAP_PROP_FRAME_WIDTH, _col);
        _cap->set(cv::CAP_PROP_FRAME_HEIGHT, _row);
    }

private:
    int _open()
    {
        _cap = std::make_shared<cv::VideoCapture>();
        if(_cap == nullptr)
        {
            cout << "Camera create failed!" << std::endl;
            return -1;
        }
        _cap->open(_camera_path);
        if(!_cap->isOpened())
        {
            cout << "Camera open failed!" << std::endl;
            return -1;
        }
        return 0;
    }

    // BlockingQueue<cv::Mat> _queue;
    bool _loop = false;
    std::string _camera_path;
    uint16_t _row;
    uint16_t _col;
    uint16_t _rate;
    std::shared_ptr<cv::VideoCapture> _cap;
    std::shared_ptr<cv::Mat> _frame = nullptr;

    std::mutex _mutex;
    std::condition_variable cond_;
    std::unique_ptr<std::thread> _thread;
};