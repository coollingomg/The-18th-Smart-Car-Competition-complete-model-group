#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2021; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度最新板卡PPNCx), The specific details consult the
 *professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file motion_controller.cpp
 * @author Leo ()
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2022-02-22
 * @note PD控制器要求稳定的控制周期：~40ms
 * @copyright Copyright (c) 2022
 *
 */

#include "../include/json.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;

class MotionController 
{
public:
    /**
     * @brief 控制器核心参数
     */
    struct Params 
    {
        float speedLow = 1.0;       // 智能车最低速
        float speedHigh = 1.0;      // 智能车最高速
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, speedLow, speedHigh); // 添加构造函数
    };

    Params params;                   // 读取控制参数

    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/motion.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good()) 
        {
            std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try 
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e) 
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }
    }
};
