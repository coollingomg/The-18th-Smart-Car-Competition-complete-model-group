#pragma once

#include "capture.hpp"
#include "predictor.hpp"
#include "stop_watch.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

#include "../src/detection/bridge_detection.cpp"
#include "../src/detection/slowzone_detection.cpp"
#include "../src/detection/depot_detection.cpp"
#include "../src/detection/farmland_avoidance.cpp"

BridgeDetection bridgeDetection;
SlowZoneDetection slowZoneDetection;
DepotDetection depotDetection;             // 车辆维修区检测
FarmlandAvoidance farmlandAvoidance;       // 农田断路区检测

struct DetectionResult
{
    cv::Mat rgb_frame;//摄像机原始图像  由主线程传入
    std::vector<PredictResult> predictor_results;
};

class Detection
{
public:
    bool Startdetect = false;
public:
    Detection() {}
    ~Detection() {}

    int init(std::string model_config_path)
    {
        return _init(model_config_path);
    }

    void Start()
    {
        _loop = true;
        run();
    }

    void Stop()
    {
        _loop = false;
        if(_thread->joinable())
            _thread->join();
        std::cout << "ai exit" << std::endl;
    }

    bool AI_Enable()
    {
        return AI_Captured;
    }

    void run()
    {
        _thread = std::make_unique<std::thread>([this]() {
            while (_loop)
            {
                std::shared_ptr<DetectionResult> result = std::make_shared<DetectionResult>();

                std::unique_lock<std::mutex> lock(_mutex);
                // while(_frame == nullptr)
                // {
                //     cond_.wait(lock);
                // }
                while (_frame == nullptr)
                {
                    // 设置超时时间
                    if (cond_.wait_for(lock, std::chrono::seconds(1)) == std::cv_status::timeout) {
                        // 超时处理逻辑
                        std::cout << "ai wait for frame time out" << std::endl;
                        break;
                    }
                }
                if(_frame != nullptr)
                {
                    result->rgb_frame = _frame->clone();
                    _frame = nullptr;
                    lock.unlock();
                }
                else 
                    continue;

                //ai推理

                if(Startdetect)
                {
                    auto feeds = _predictor->preprocess(result->rgb_frame, {320, 320});
                    _predictor->run(*feeds);
                    _predictor->render();
                    
                    bridgeDetection.bridgeCheck(_predictor->results);
                    slowZoneDetection.slowZoneCheck(_predictor->results);
                    depotDetection.depotDetection(_predictor->results);
                    farmlandAvoidance.farmdlandCheck(_predictor->results);
                }

                bool flag = false;
                for(int i = 0; i < _predictor->results.size(); i++)
                {
                    std::string label_name = _predictor->results[i].label;
                    if((label_name == "tractor" || label_name == "corn") && _predictor->results[i].score > 0.52
                        && _predictor->results[i].y + _predictor->results[i].height / 2 > 20)
                    {
                        flag = true;
                        break;
                    }
                    else if(label_name == "crosswalk")
                    {
                        flag == false;
                        break;
                    }
                }
                if(_Cnt == 0)
                    AI_Captured = flag;
                if(AI_Captured)
                {
                    _Cnt++;
                    if(_Cnt > 2)
                        _Cnt = 0;
                }
            
                //数据传递
                result->predictor_results = _predictor->results;
                std::unique_lock<std::mutex> lock2(_mutex);
                _lastResult = result;
                cond_.notify_all();
            }
        });
    }

    std::shared_ptr<DetectionResult> getLastFrame()
    {
        std::shared_ptr<DetectionResult> ret = nullptr;
        {
            std::unique_lock<std::mutex> lock(_mutex);

            while (_lastResult == nullptr)
            {
                cond_.wait(lock);
            }
            ret = _lastResult;
            _lastResult = nullptr;
        }
        return ret;
    }

    void setFrame(cv::Mat img)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _frame = std::make_shared<cv::Mat>(img.clone());
        cond_.notify_all();
    }

    void drawbox(cv::Mat& img, std::vector<PredictResult> results)
    {
        for(int i=0;i<results.size();i++)
        {
            PredictResult result = results[i];
        
            auto score = std::to_string(result.score);
            int pointY = result.y - 20;
            if (pointY < 0)
            pointY = 0;
            cv::Rect rectText(result.x, pointY, result.width, 20);
            cv::rectangle(img, rectText, _predictor->getCvcolor(result.type), -1);
            std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 3) + "]";
            cv::Rect rect(result.x, result.y, result.width, result.height);
            cv::rectangle(img, rect, _predictor->getCvcolor(result.type), 1);
            cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
        }
    }

private:
    int _init(std::string model_config_path)
    {
        _predictor = std::make_shared<PPNCDetection>();
        if (_predictor == nullptr)
        {
            std::cout << "Predictor Create failed." << std::endl;
            return -1;
        }
        int ret = _predictor->init(model_config_path);
        if (ret != 0)
        {
            return -1;
        }
        return 0;
    }
    bool _loop = false;
    std::shared_ptr<PPNCDetection> _predictor;
    std::shared_ptr<DetectionResult> _lastResult;
    std::shared_ptr<cv::Mat> _frame;
    
    uint16_t _Cnt = 0;//计数器
    bool AI_Captured = false;

    std::mutex _mutex;
    std::condition_variable cond_;
    std::unique_ptr<std::thread> _thread;
};