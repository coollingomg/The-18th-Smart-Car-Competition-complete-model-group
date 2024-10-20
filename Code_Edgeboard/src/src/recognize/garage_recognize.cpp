/**
 * @file garage_recognize.cpp
 * @author WHUT-wzl
 * @brief 车库识别
 * @version 0.1
 * @date 2023-05-21
 * @copyright Copyright (c) 2023
 * @note 车库识别步骤:
 *                  [01] 车库标志识别：斑马线
 *                  [02] 入库点搜索：拐点搜索确定
 *                  [03] 入库路径规划
 *                  [04] 入库完成判定
 */


#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "../../include/json.hpp"
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../motion_control.cpp"


using namespace cv;
using namespace std;


class GarageRecognition
{
public:
    GarageRecognition()
    {
        loadParams();
    }
private:
    /**
     * @brief 控制器核心参数
     */
    struct Params
    {
        uint16_t    garage_run = 0;         // 选择出库方式
        uint16_t    corner_lines = 0;       // 开始打角行数
        uint16_t    slowdown_condition = 0; // 入库开始减速的行数
        uint16_t    disGarageEntry = 0;     // 入库开始补线的行数
        uint16_t    stop_line = 0;          // 入库停车行数
        int         brakePicNum = 0;        // 刹车帧数
        int         first_PicNum = 0;       // 第一次经过车库补线帧数，直至斑马线消失
        float       speed_nor = 0;          // 刚检测到库，不减速
        float       speed_in = 0;           // 减速入库
        float       speed_out = 0;          // 出库速度
        int         shield_counter = 0;     // 屏蔽计数器
        float       speed_readyend = 0;     // 准备入库的速度
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, garage_run, corner_lines, slowdown_condition, disGarageEntry, stop_line, 
            brakePicNum, first_PicNum, speed_nor, speed_in, speed_out, shield_counter, speed_readyend); // 添加构造函数
    };


    /**
     * @brief 定义入库状态机枚举
     */
    enum flag_garage_e 
    {
        GARAGE_NONE = 0,    // 空闲状态
        GARAGE_DETECTION,   // 检测到斑马线
        GARAGE_OUT_READY,   // 出库准备
        GARAGE_OUT_LEFT,    // 出库 左
        GARAGE_OUT_RIGHT,   // 出库 右
        GARAGE_IN_LEFT,     // 入库 左
        GARAGE_IN_RIGHT,    // 入库 右
        GARAGE_PASS_LEFT,   // 不进库 左
        GARAGE_PASS_RIGHT,  // 不进库 右
        GARAGE_HALF_STOP,   // 半入库
        GARAGE_BRAKE,       // 停车
        GARAGE_STOP,        // 进库完毕, 停车
    };


    // 定义入库状态机枚举变量,初始值为none
    flag_garage_e flag_garage = GARAGE_NONE;
    // 定义用于绘图的点，描述布线的三个点，和ai识别框的中点
    POINT _pointRU;
    POINT _pointLU;
    POINT _pointRD;
    POINT _crosswalk;
    // 读取控制参数
    Params params;
    // 定义一个计数器，当连续几张图片中都检测到斑马线，才确认检测到
    uint16_t chack_picNum = 0;
    // 定义一个计数器，当第一张张检测到斑马线，开始计之后的图像数量
    uint16_t ensure_picNum = 0;


public:
    // 变量相关定义
    int     garage_num      = 0;                        // 记录当前第几次到达车库
    float   speed_ku        = 0;                        // 速度控制，默认0.8m
    int     picNum_brake    = 0;                        // 刹车帧数
    bool    Garage_screen   = false;                    // 车库屏蔽标志位
    int     garage_scre_num = params.shield_counter;    // 屏蔽计数器标志位
    int     site_cross_x    = 0;                        // 识别出来的斑马线在图像的位置
    int     site_cross_y    = 0;                        // 识别出来的斑马线在图像的位置
    int     cross_down_line = 0;                        // 识别出来的车库下拐点
    int     cross_up_line   = 0;                        // 识别出来的车库上拐点

private:
    /**
     * @brief 起点检测，只用于发车时判断是否在车库
     * @param predict
     * @return true
     * @return false
     */
    bool startingCheck(vector<PredictResult> predict)
    {
        // 定义静态变量计数器，一次执行完不能销毁变量
        static uint16_t counterCrosswalk = 0;                      // 起点检测计数器->检测到斑马线
        static uint16_t counterSessionTwo = 0;                     // 检测到斑马线后的帧数

        //遍历ai预测结果
        for (int i = 0; i < predict.size(); i++)
        {
            // 标志检测，ai检测到斑马线标识
            if (predict[i].label == LABEL_CROSSWALK)
            {
                // 检测斑马线的计数器自增，并且退出循环
                counterCrosswalk++;
                break;
            }
        }

        // 如果有检测到斑马线，开始判断之后N帧图像也检测到斑马线的数量，从而进行判断
        if (counterCrosswalk)
        {
            // 记录自从第一次检测到斑马线后，跑图像的帧数
            counterSessionTwo++;

            // 如果在8帧图像中，有4帧检测到斑马线，则视为在车库内。然后清零计数器
            if (counterCrosswalk >= 2 && counterSessionTwo <= 4)
            {
                counterSessionTwo = 0;
                counterCrosswalk = 0;
                return true;
            }
            // 不满足要求则清零计数器，并返回没有检测到起点车库
            else if (counterSessionTwo > 4)
            {
                counterSessionTwo = 0;
                counterCrosswalk = 0;
            }
        }

        // 返回假，没有检测到起点
        return false;
    }


    /**
     * @brief 搜索ai预测结构，是否识别到斑马线
     * @param predict
     * @return POINT
     */
    POINT searchCrosswalkSign(vector<PredictResult> predict)
    {
        // 遍历ai预测的结果，查看是否有斑马线的标识
        for (int i = 0; i < predict.size(); i++)
        {
            // 检测到斑马线
            if (predict[i].label == LABEL_CROSSWALK) 
            {
                // 应该是返回标记框的中心点
                return POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2);
            }
        }
        // 返回点（0，0），说明没有识别到斑马线
        return POINT(0, 0);
    }


    /**
     * @brief 搜索ai预测结构，是否识别到斑马线
     * @param predict
     * @return bool
     */
    bool searchCrosswalk_if_recognize(vector<PredictResult> predict)
    {
        // 遍历ai预测的结果，查看是否有斑马线的标识
        for (int i = 0; i < predict.size(); i++)
        {
            // 检测到斑马线
            if (predict[i].label == LABEL_CROSSWALK) 
            {
                // 返回真，识别到了斑马线
                return true;
            }
        }
        // 返回假，说明没有识别到斑马线
        return false;
    }


    /**
     * @brief 识别车库检测
     * @param track
     * @return bool
     */
    bool detect_garage(TrackRecognition track)
    {
        if (track.pointsEdgeRight.size() == 0 || track.pointsEdgeLeft.size() == 0) //环岛有效行限制
        {
            return false;
        }

        // 左
        if(params.garage_run == 0 || params.garage_run == 2)
        {
            // 有边线去除掉前1/5的线，然后斜率小于一定范围，既直线
            if(track.pointsEdgeRight.size() > 0)
            {
                if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x <= ROWSIMAGE / 5)
                {
                    int cnt = track.pointsEdgeRight.size() - 1;
                    // 去除图片上面固定行的点集
                    for(int i = track.pointsEdgeRight.size() - 1; track.pointsEdgeRight[i].x <= ROWSIMAGE / 5; i--)
                    {
                        cnt--;
                    }
                    if(cnt <= 0)
                        cnt = 0;
                    track.pointsEdgeRight.resize(cnt);
                }
            }
            // // 减小行数
            // if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x <= ROWSIMAGE / 3)
            // {
            //     int cnt = track.pointsEdgeLeft.size() - 1;
            //     // 去除图片上面固定行的点集
            //     for(int i = track.pointsEdgeLeft.size() - 1; track.pointsEdgeLeft[i].x <= ROWSIMAGE / 3; i--)
            //     {
            //         cnt--;
            //     }
            //     track.pointsEdgeLeft.resize(cnt);
            // }

            int searchLeft_down = garage_point_search_left(track.pointsEdgeLeft);
            int searchLeft_up = sBreakLeftUp(track.pointsEdgeLeft);

            cross_down_line = searchLeft_down;
            cross_up_line = searchLeft_up;

            // if(searchLeft_down != 0 && searchLeft_up != 0)
            // {
                site_cross_x = track.pointsEdgeLeft[searchLeft_down].x;
                site_cross_y = track.pointsEdgeLeft[searchLeft_up].x;
            // }

            if(searchLeft_up == track.pointsEdgeLeft.size() - 1)
                return false;
            if(searchLeft_down == searchLeft_up)
                return false;
            if(abs(site_cross_x - site_cross_y) < 25)
                return false;
            if((((searchLeft_down + searchLeft_up) / 2) < (track.pointsEdgeRight.size() - 1)) && (((searchLeft_up + searchLeft_down) / 2) > 0))
            {
                if(abs(track.pointsEdgeRight[(searchLeft_up + searchLeft_down) / 2].y - track.pointsEdgeLeft[(searchLeft_up + searchLeft_down) / 2].y) < 170)
                    return false;
            }
            else
                return false;

            if(track.stdevEdgeCal(track.pointsEdgeRight,40) <= 70 && searchLeft_up > searchLeft_down && 
               track.garageEnable.x == 1)
               //&& track.pointsEdgeLeft[track.garageEnable.y].x <= track.pointsEdgeLeft[searchLeft_up].x &&
               //track.pointsEdgeLeft[track.garageEnable.y].x >= track.pointsEdgeLeft[searchLeft_down].x)
                return true;
        }
        // 右
        else if(params.garage_run == 1 || params.garage_run == 3)
        {
            // 有边线去除掉前1/5的线，然后斜率小于一定范围，既直线
            if(track.pointsEdgeLeft.size() > 0)
            {
                if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x <= ROWSIMAGE / 5)
                {
                    int cnt = track.pointsEdgeLeft.size() - 1;
                    // 去除图片上面固定行的点集
                    for(int i = track.pointsEdgeLeft.size() - 1; track.pointsEdgeLeft[i].x <= ROWSIMAGE / 5; i--)
                    {
                        cnt--;
                    }
                    if(cnt <= 0)
                        cnt = 0;
                    track.pointsEdgeLeft.resize(cnt);
                }
            }
            // //减小行数
            // if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x <= ROWSIMAGE / 3)
            // {
            //     int cnt = track.pointsEdgeRight.size() - 1;
            //     // 去除图片上面固定行的点集
            //     for(int i = track.pointsEdgeRight.size() - 1; track.pointsEdgeRight[i].x <= ROWSIMAGE / 3; i--)
            //     {
            //         cnt--;
            //     }
            //     track.pointsEdgeRight.resize(cnt);
            // }

            int searchRight_down = garage_point_search_right(track.pointsEdgeRight);
            int searchRight_up = sBreakRightUp(track.pointsEdgeRight);

            cross_down_line = searchRight_down;
            cross_up_line = searchRight_down;

            // if(searchRight_down != 0 && searchRight_up != 0)
            // {
                site_cross_x = track.pointsEdgeLeft[searchRight_down].x;
                site_cross_y = track.pointsEdgeLeft[searchRight_up].x;
            // }

            if(searchRight_up == track.pointsEdgeRight.size() - 1 || searchRight_down == 0)
                return false;
            if(searchRight_down == searchRight_up)
                return false;
            if(abs(site_cross_x - site_cross_y) < 25)
                return false;
            if((((searchRight_down + searchRight_up) / 2) < (track.pointsEdgeRight.size() - 1)) && (((searchRight_up + searchRight_down) / 2) > 0))
            {
                if(abs(track.pointsEdgeLeft[(searchRight_up + searchRight_down) / 2].y - track.pointsEdgeRight[(searchRight_up + searchRight_down) / 2].y) < 170)
                    return false;
            }
            else
                return false;
            
            if(track.stdevEdgeCal(track.pointsEdgeLeft,40) <= 70 && searchRight_up > searchRight_down && 
               track.garageEnable.x == 1)
               //&& track.pointsEdgeRight[track.garageEnable.y].x <= track.pointsEdgeRight[searchRight_up].x &&
               //track.pointsEdgeRight[track.garageEnable.y].x >= track.pointsEdgeRight[searchRight_down].x)
                return true;
        }
        // 返回假，说明没有识别到斑马线
        return false;
    }


    /**
     * @brief 出库状态机识别
     * @param pointsEdgeLeft 左边线
     * @param pointsEdgeRight 右边线
     * @param spurroad 岔路集合
     * @return bool
     */
    bool search_garage_OutState(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight, vector<POINT> spurroad)
    {
        // 首先判断拐点是否满足要求，不满足要求则返回假
        if(spurroad.size() < 2)
        {
            return false;
        }

        // 搜寻突变点
        uint16_t rowBreakRight = searchBreakRightDown(pointsEdgeRight, 0, ROWSIMAGE);        // 右下补线点搜索,从0开始到240行
        uint16_t rowBreakLeft = searchBreakLeftDown(pointsEdgeLeft, 0, ROWSIMAGE);           // 左下点搜索,从0开始到240行
        // 满足条件，返回真
        if(rowBreakRight >= 20 || rowBreakLeft >= 20)
        {
            return true;
        }

        // 返回假，说明没有在库里面
        return false;
    }


    /**
     * @brief 冒泡法求取集合中值，由小到大
     * @param vec 输入集合
     * @param return_choise 返回值选择
     * @return int 中值
     */
    int getMiddleValue(vector<int> vec, char return_choise)
    {
        // 传入集合的合理性判断
        if (vec.size() < 1)
            return -1;
        if (vec.size() == 1)
            return vec[0];

        // 传入集合的长度
        int len = vec.size();
        for (int len = vec.size(); len > 0; len--)
        {
            // 是否进行排序操作标志
            bool sort = true; 
            // 遍历排序
            for (int i = 0; i < len - 1; ++i)
            {
                if (vec[i] > vec[i + 1])
                {
                    swap(vec[i], vec[i + 1]);
                    sort = false;
                }
            }
            // 如果sort一直时true，没有被改变，说明本次遍历所有值都是由小到大的，排序完成
            if (sort) 
            break;
        }

        // 返回中值
        if(return_choise == 'm')
        {
            return vec[(int)vec.size() / 2];
        }
        else if(return_choise == 'f')
        {
            return vec[0];
        }
    }


    /**
     * @brief 搜索最佳岔路
     * @param spurroad 岔路集合
     * @return POINT 岔路坐标
     */
    POINT searchBestSpurroad(vector<POINT> spurroad)
    {
        // 没有岔路点，返回图线左上第一个点
        if (spurroad.size() < 1)
            return POINT(0, 0);

        // 记录所有的y轴坐标值
        vector<int> cols;
        for (int i = 0; i < spurroad.size(); i++)
        {
            cols.push_back(spurroad[i].y);
        }
        // 对横坐标进行冒泡排序
        int colBest = getMiddleValue(cols, 'm'); 
        POINT spurroadBest = POINT(ROWSIMAGE, colBest);

        // 搜索最佳岔路：横向中心|纵向最高
        for (int i = 0; i < spurroad.size(); i++)
        {
            if (abs(spurroadBest.y - spurroad[i].y) < 70)
            {
                if (spurroad[i].x < spurroadBest.x)
                    spurroadBest.x = spurroad[i].x;
            }
        }

        // 返回最佳岔路点
        return spurroadBest;
    }


    /**
     * @brief 搜索最佳岔路,最高点
     * @param spurroad 岔路集合
     * @return POINT 岔路坐标
     */
    POINT searchBestSpurroad_High(vector<POINT> spurroad)
    {
        // 没有岔路点，返回图线左上第一个点
        if (spurroad.size() < 1)
            return POINT(0, 0);

        // 记录所有的x轴坐标值
        vector<int> rows;
        for (int i = 0; i < spurroad.size(); i++)
        {
            rows.push_back(spurroad[i].x);
        }

        // 对纵坐标进行冒泡排序，从小到大排序，选取最小值
        int rowBest = getMiddleValue(rows, 'f');
        // 定义一个点记录纵坐标，横坐标随便
        POINT spurroadBest = POINT(rowBest, 0);

        // 搜索最佳岔路：纵向最高
        for (int i = 0; i < spurroad.size(); i++)
        {
            if(spurroadBest.x == spurroad[i].x)
            {
                spurroadBest = spurroad[i];
            }
        }

        // 返回最佳岔路点
        return spurroadBest;
    }


    /**
     * @brief 搜索赛道突变行（左下）
     * @param pointsEdgeLeft 左边缘线
     * @param row_start 起始搜寻行
     * @param row_end 终止搜寻行
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        if(pointsEdgeLeft.size() == 0)
        {
            return 0;
        }
        
        // if(row_start == 0)
        // {
        //     row_start++;
        // }

        // 限幅判断
        if(row_end > pointsEdgeLeft.size())
        {
            row_end = pointsEdgeLeft.size();
        }
        if(row_start > pointsEdgeLeft.size())
        {
            row_start = pointsEdgeLeft.size();
        }

        // 寻找左边跳变点
        for (int i = row_start; i < row_end; i++)
        {
            if(pointsEdgeLeft[i].y > 5)
            {
                start = true;
            }
            if(start)
            {
                if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y)
                {
                    rowBreakLeft = i;
                    counter = 0;
                }
                // 突变点计数
                else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) 
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakLeft;
                }
            }
        }
        // 没有搜寻到，返回0
        return rowBreakLeft;
    }


    /**
     * @brief 搜索赛道突变行（右下）
     * @param pointsEdgeRight 右边缘线
     * @param row_start 起始搜寻行
     * @param row_end 终止搜寻行
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakRight = 0;
        uint16_t counter = 0;

        if(pointsEdgeRight.size() == 0)
        {
            return 0;
        }

        // if(row_start == 0)
        // {
        //     row_start++;
        // }

        if(row_end > pointsEdgeRight.size())
        {
            row_end = pointsEdgeRight.size();
        }
        if(row_start > pointsEdgeRight.size())
        {
            row_start = pointsEdgeRight.size();
        }
        // 寻找右边跳变点
        for (int i = row_start; i < row_end; i++) 
        {
            if(pointsEdgeRight[i].y < COLSIMAGE - 5)
            {
                start = true;
            }
            if(start)
            {
                if (pointsEdgeRight[i].y <= pointsEdgeRight[rowBreakRight].y)
                {
                    rowBreakRight = i;
                    counter = 0;
                }
                // 突变点计数
                else if (pointsEdgeRight[i].y > pointsEdgeRight[rowBreakRight].y)
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakRight;
                }
            }
        }
        // 没有搜寻到，返回
        return rowBreakRight;
    }


    /**
     * @brief 库特征点搜索（左下）
     * @param pointsEdgeLeft 左边缘线
     * @return uint16_t
     */
    uint16_t garage_point_search_left(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        if(pointsEdgeLeft.size() == 0)
        {
            return 0;
        }
        
        // 寻找左边跳变点
        for (int i = 0; i < pointsEdgeLeft.size() - 1; i++)
        {
            if(pointsEdgeLeft[i].y <= 10 && i < 6)
            {
                continue;
            }
            if (pointsEdgeLeft[i].y >= pointsEdgeLeft[rowBreakLeft].y || (abs(pointsEdgeLeft[i].y - pointsEdgeLeft[rowBreakLeft].y) < 10))
            {
                rowBreakLeft = i;
                counter = 0;
            }
            // 突变点计数
            else
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeft;
            }
        }

        // 没有搜寻到，返回0
        return rowBreakLeft;
    }


    /**
     * @brief 库特征点搜索（右下）
     * @param pointsEdgeRight 右边缘线
     * @return uint16_t
     */
    uint16_t garage_point_search_right(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRight = 0;
        uint16_t counter = 0;

        if(pointsEdgeRight.size() == 0)
        {
            return 0;
        }

        // 寻找右边跳变点
        for (int i = 0; i < pointsEdgeRight.size() - 1; i++) 
        {
            // if(pointsEdgeRight[i].y >= 310 && i < 6)
            // {
            //     continue;
            // }
            if (pointsEdgeRight[i].y <= pointsEdgeRight[rowBreakRight].y || (abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRight].y) < 10))
            {
                rowBreakRight = i;
                counter = 0;
            }
            // 突变点计数
            else
            {
                counter++;
                if (counter > 5)
                    return rowBreakRight;
            }
        }
        // 没有搜寻到，返回0
        return rowBreakRight;
    }


    /**
     * @brief 搜索赛道突变行（右上）
     * @param pointsEdgeRight 右边缘线
     * @return uint16_t
     */
    uint16_t sBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRight = pointsEdgeRight.size() - 1;
        uint16_t counter = 0;

        if(pointsEdgeRight.size() == 0)
        {
            return 0;
        }

        // 寻找右边跳变点
        for (int i = pointsEdgeRight.size() - 1; i > 0; i--) 
        {
            // if(pointsEdgeRight[i].y >= 310 && i > pointsEdgeRight.size() - 6)
            // {
            //     continue;
            // }
            if (pointsEdgeRight[i].y <= pointsEdgeRight[rowBreakRight].y || (abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRight].y) < 10))
            {
                rowBreakRight = i;
                counter = 0;
            }
            // 突变点计数
            else
            {
                counter++;
                if (counter > 5)
                    return rowBreakRight;
            }
        }
        // 没有搜寻到，直接返回
        return rowBreakRight;
    }


    /**
     * @brief 搜索赛道突变行（左上）
     * @param pointsEdgeLeft 左边缘线
     * @return uint16_t
     */
    uint16_t sBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = pointsEdgeLeft.size() - 1;
        uint16_t counter = 0;

        if(pointsEdgeLeft.size() == 0)
        {
            return 0;
        }

        // 寻找左边跳变点
        for (int i = pointsEdgeLeft.size() - 1; i > 0; i--) 
        {
            // if(pointsEdgeLeft[i].y <= 10 && i > pointsEdgeLeft.size() - 6)
            // {
            //     continue;
            // }
            if (pointsEdgeLeft[i].y >= pointsEdgeLeft[rowBreakLeft].y || (abs(pointsEdgeLeft[i].y - pointsEdgeLeft[rowBreakLeft].y) < 10))
            {
                rowBreakLeft = i;
                counter = 0;
            }
            // 突变点计数
            else
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeft;
            }
        }
        // 没有搜寻到，直接返回
        return rowBreakLeft;
    }


    /**
     * @brief 搜索出库|赛道边缘突变（左上）
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeft(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = pointsEdgeLeft.size() - 1;
        uint16_t counter = 0;
        for (int i = pointsEdgeLeft.size() - 1; i > 50; i--)
        {
            if (pointsEdgeLeft[i].y < 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i + 3;
                }
            }
            else
            {
                counter = 0;
            }
        }
        return rowBreakLeft;
    }


    /**
     * @brief 搜索入库|赛道突变行（右上）
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRight(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRight = pointsEdgeRight.size() - 1;
        uint16_t counter = 0;
        for (int i = pointsEdgeRight.size() - 1; i > 50; i--)
        {
            if (pointsEdgeRight[i].y > COLSIMAGE - 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i + 3;
                }
            }
            else
            {
                counter = 0;
            }
        }
        return rowBreakRight;
    }


public:
    /**
     * @brief 车库初始化函数
     * @param void
     * @return void
     */
    void garage_reset(void)
    {
        // 出入库状态机，初始为出库准备
        flag_garage = flag_garage_e::GARAGE_OUT_READY;
    }


    /**
     * @brief 计数器清零函数
     * @param void
     * @return void
     */
    void counter_reset(void)
    {
        // 清零
        uint16_t chack_picNum = 0;
        uint16_t ensure_picNum = 0;
    }


    /**
     * @brief 得到目前控制速度
     * @param void
     * @return float
     */
    float get_speed(void)
    {
        return speed_ku;
    }


    /**
     * @brief 得到目前的状态机
     * @param void
     * @return bool
     */
    bool get_now_value(void)
    {
        // 返回真，停止信号
        if(flag_garage == flag_garage_e::GARAGE_STOP)
        {
            return true;
        }
        return false;
    }


    /**
     * @brief 车库控制函数
     * @param predict ai预测结果
     * @return bool
     */
    bool garage_contral(TrackRecognition &track) 
    {
        // 返回状态值
        bool status = false;

        // 检测斑马线
        status = check_garage(track);
        
        // 判断目前的赛道类型->检测到斑马线，进行出库入库操作
        if(flag_garage == flag_garage_e::GARAGE_OUT_LEFT || flag_garage == flag_garage_e::GARAGE_OUT_RIGHT ||
           flag_garage == flag_garage_e::GARAGE_IN_LEFT || flag_garage == flag_garage_e::GARAGE_IN_RIGHT ||
           flag_garage == flag_garage_e::GARAGE_PASS_LEFT || flag_garage == flag_garage_e::GARAGE_PASS_RIGHT || 
           flag_garage == flag_garage_e::GARAGE_HALF_STOP || flag_garage == flag_garage_e::GARAGE_BRAKE)
        {
            // 返回状态
            status = true;
            // 相关状态机的处理
            run_garage(track);
        }

        // 在完成相关操作后，检测是否已经停车，或者到达了基础赛道
        if(flag_garage == flag_garage_e::GARAGE_NONE)
        {
            // 返回状态
            status = false;
        }
        return status;
    }


    /**
     * @brief 检测斑马线
     * @param track 边线
     * @return bool
     */
    bool check_garage(TrackRecognition track) 
    {
        // 定义一个静态变量来进行计数，当超过20帧图像还没有检测到出库，则初始位置不在车库中
        static uint16_t count_to_CarNotInGarage = 0;
        // 定义一个局部变量，用于记录目前也没有检测到斑马线，返回出去
        bool if_garage = false;
        // 准备出库状态机，当检测到起点斑马线时，进入出库状态机，清零计数器
        if (flag_garage == flag_garage_e::GARAGE_OUT_READY) 
        {
            // // 返回真，说明检测到起点，进入出库状态机，局部变量返回真，检测到斑马线了
            // if(startingCheck(predict))
            // {
            //     // 根据配置文件，0为左库，跑两圈入库
            //     if(params.garage_run == 0)
            //     {
            //         flag_garage = flag_garage_e::GARAGE_OUT_LEFT;
            //     }
            //     // 根据配置文件，1为右库，跑两圈入库
            //     else if(params.garage_run == 1)
            //     {
            //         flag_garage = flag_garage_e::GARAGE_OUT_RIGHT;
            //     }
            //     count_to_CarNotInGarage = 0;
            //     speed_ku = params.speed_out;
            //     if_garage = true;
            // }

            if(search_garage_OutState(track.pointsEdgeLeft, track.pointsEdgeRight, track.spurroad))
            {
                // 标记识别到的斑马线位置
                site_cross_x = track.pointsEdgeRight[track.garageEnable.y].x;
                // 根据配置文件，0为左库，跑两圈入库
                if(params.garage_run == 0 || params.garage_run == 2)
                {
                    flag_garage = flag_garage_e::GARAGE_OUT_LEFT;
                }
                // 根据配置文件，1为右库，跑两圈入库
                else if(params.garage_run == 1 || params.garage_run == 3)
                {
                    flag_garage = flag_garage_e::GARAGE_OUT_RIGHT;
                }
                count_to_CarNotInGarage = 0;
                // speed_ku = params.speed_out;
                if_garage = true;
            }
            // 返回假，说明可能没有检测到8帧图像达到判断条件；或者车初始位置不在车库中
            else
            {
                // 计数器自增
                count_to_CarNotInGarage++;
                // 判断计数器值，如果超过20帧还没有检测到起点，说明不在车库中，改变状态机，默认已经完成出库，并清零计数器
                if(count_to_CarNotInGarage >= 60)
                {
                    flag_garage = flag_garage_e::GARAGE_NONE;
                    count_to_CarNotInGarage = 0;
                }
            }
        }

        /*******************************************************************/

        // 空闲状态机，完成出库后，回到空闲状态机
        else if (flag_garage == flag_garage_e::GARAGE_NONE)
        {
            // static bool flag_cross = false;
            // if(searchCrosswalk_if_recognize(predict))
            // {
            //     flag_cross = true;
            // }

            // // 返回真，说明检测到斑马线，改变状态机
            // if(searchCrosswalk_if_recognize(predict) || (track.garageEnable.x == 1 && flag_cross))
            // {
            //     chack_picNum++;
            //     site_cross_x = track.pointsEdgeRight[track.garageEnable.y].x;
            // }

            if(detect_garage(track))
            {
                chack_picNum++;
                // site_cross_x = track.pointsEdgeRight[track.garageEnable.y].x;
            }

            if(chack_picNum)
            {
                // 图片帧数计数
                ensure_picNum++;
                // 当在4帧图像中，有大于等于3帧图片检测到斑马线，则进入下一个状态
                if (chack_picNum >= 2)
                {
                    flag_garage = GARAGE_DETECTION;
                    // 返回真，检测到斑马线，用于清空跑完一圈其他
                    if_garage = true;

                    // 进入下一个状态机，清空计数器
                    ensure_picNum = 0;
                    chack_picNum = 0;
                    // flag_cross = false;
                }

                // 在第一次检测到斑马线之后，5真图片后，清空计数器
                if (ensure_picNum > 4)
                {
                    ensure_picNum = 0;
                    chack_picNum = 0;
                    // flag_cross = false;
                }
            }
        }

        /*******************************************************************/

        // 如果车库处于屏蔽状态，在一定时间内，再次识别到斑马线也不进入车库状态机
        if(Garage_screen)
        {
            flag_garage = flag_garage_e::GARAGE_NONE;
            if_garage = false;

            // 等待屏蔽计数器完成，重新幅值并清除标志位
            if((--garage_scre_num) <= 0)
            {
                Garage_screen = false;
                garage_scre_num = params.shield_counter;
            }
        }

        /*******************************************************************/

        // 当由空闲状态检测到斑马线时，是否入库判断
        if (flag_garage == flag_garage_e::GARAGE_DETECTION) 
        {
            // 配置文件读取为左库
            if (params.garage_run == 0) 
            {
                // 第二次经过车库入库，状态机改为进左库
                if (++garage_num >= 2) 
                {
                    flag_garage = GARAGE_IN_LEFT;
                }
                // 第一次经过车库不入库，状态机改为不进左库
                else
                {
                    flag_garage = GARAGE_PASS_LEFT;
                }
            }
            // 配置文件读取为右库
            else if (params.garage_run == 1) 
            {  
                // 第二次经过车库入库，状态机改为进右库
                if (++garage_num >= 2) 
                {
                    flag_garage = GARAGE_IN_RIGHT;  
                } 
                // 第一次经过车库不入库，状态机改为不进右库
                else
                {
                    flag_garage = GARAGE_PASS_RIGHT;  
                }
            } 
            // 配置文件读取为左库，一直不进左库
            else if (params.garage_run == 2) 
            {  
                flag_garage = GARAGE_PASS_LEFT;
            } 
            // 配置文件读取为右库，一直不进右库
            else if (params.garage_run == 3) 
            {  
                flag_garage = GARAGE_PASS_RIGHT;
            }
        }

        // 返回是否检测到斑马线
        return if_garage;
    }


    /**
     * @brief 执行出入库的具体操作
     * @param track 边缘路径线
     * @return void
     */
    void run_garage(TrackRecognition &track) 
    {
        switch (flag_garage)
        {
            // 出库，左库
            case GARAGE_OUT_LEFT:
                garageExitRecognition(track);
                break;
        /* ********************************************************* */
            // 出库，右库
            case GARAGE_OUT_RIGHT:
                garageExitRecognition(track);
                break;
        /* ********************************************************* */
            // 入库，左库
            case GARAGE_IN_LEFT:
                garageEntryRec(track);
                break;
        /* ********************************************************* */
            // 入库，右库
            case GARAGE_IN_RIGHT:
                garageEntryRec(track);
                break;
        /* ********************************************************* */
            // 左库，不入库
            case GARAGE_PASS_LEFT:
                no_processing(track);
                break;
        /* ********************************************************* */
            // 右库，不入库
            case GARAGE_PASS_RIGHT:
                no_processing(track);
                break;
        /* ********************************************************* */
            // 半入库
            case GARAGE_HALF_STOP:
                // 屏蔽图像前半部分的信息
                if(track.pointsEdgeLeft[0].x <= 60 || track.pointsEdgeRight[0].x <= 60)
                {
                    track.pointsEdgeLeft.resize(0);
                    track.pointsEdgeRight.resize(0);
                }

                // 使用基础巡线即可，达到条件，停车
                if((track.pointsEdgeLeft.size() <= params.stop_line && track.pointsEdgeRight.size() <= params.stop_line) || 
                   (track.pointsEdgeRight.size() == 0 && track.pointsEdgeLeft.size() == 0) || (track.pointsEdgeLeft[0].x >= 121 && track.pointsEdgeRight[0].x >= 121))
                {
                    flag_garage = GARAGE_BRAKE;
                    // speed_ku = -params.speed_nor;
                }
                break;
        /* ********************************************************* */
            // 刹车
            case GARAGE_BRAKE:
                // speed_ku = -params.speed_nor;
                picNum_brake++;
                if(picNum_brake >= params.brakePicNum)
                {
                    flag_garage = GARAGE_STOP;
                }
                break;
        /* ********************************************************* */
            default:
                break;
        }
    }


    /**
     * @brief 第一圈跑完，经过车库，是否完全过车库判断
     * @param track 路径
     */
    void no_processing(TrackRecognition &track)
    {
        // 定义静态变量，用于计数，完全经过车库
        static uint16_t counterExit = 0;
        // 第一圈正常速度
        speed_ku = params.speed_nor;
        // 检测斑马线，来判定是否出库，连续5帧图片都达到判定条件，则完成出库
        if (!detect_garage(track))
        {
            // 左库补线
            if(params.garage_run == 0 || params.garage_run == 2)
            {
                track.pointsEdgeLeft[0] = POINT(225, 10);
                if(cross_up_line != 0)
                    line(track.pointsEdgeLeft, 0, cross_up_line);
                if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y > 160)
                    track.pointsEdgeLeft = track.predictEdgeLeft(track.pointsEdgeRight);
            }
            // 右库补线
            if(params.garage_run == 1 || params.garage_run == 3)
            {
                track.pointsEdgeRight[0] = POINT(225, 310);
                if(cross_up_line != 0)
                    line(track.pointsEdgeRight, 0, cross_up_line);
                // 如果出现帧错误，靠逆透视补线
                if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y < 160)
                    track.pointsEdgeRight = track.predictEdgeRight(track.pointsEdgeLeft);
            }

            // 计数器计数，如果有9帧图片，则出库完成，进入空闲状态机，并且开启屏蔽
            counterExit++;
            if (counterExit >= params.first_PicNum)
            {
                Garage_screen = true;
                garage_scre_num = params.shield_counter;
                flag_garage = flag_garage_e::GARAGE_NONE; 
            }
        }
        // 检测到斑马线，计数器清零
        else
        {
            /***补线操作***/
            // 左库补线
            if(params.garage_run == 0 || params.garage_run == 2)
                line(track.pointsEdgeLeft, cross_down_line, cross_up_line);
            // 右库补线
            if(params.garage_run == 1 || params.garage_run == 3)
                line(track.pointsEdgeRight, cross_down_line, cross_up_line);
            // 计数器清零
            counterExit = 0;
        }
    }


    /**
     * @brief 出库识别与路径规划
     * @param track
     * @param predict ai预测
     */
    void garageExitRecognition(TrackRecognition &track)
    {
        // 设置出库速度
        speed_ku += 0.05f;
        if(speed_ku >= params.speed_out)
            speed_ku = params.speed_out;
        
        // 用于dubug绘图的三个点，清零
        _pointRU = POINT(0, 0);
        _pointLU = POINT(0, 0);
        _pointRD = POINT(0, 0);

        // 定义静态变量，用于计数出库完成状态
        static uint16_t counterExitOut = 0;

        // 定义补线要用的三个点
        POINT startPoint;
        POINT midPoint;
        POINT endPoint;

        // 出左库
        if (flag_garage == flag_garage_e::GARAGE_OUT_LEFT)
        {
            //中线拟合去掉上面1/3行，处理左边线
            if(track.pointsEdgeLeft.size() > 0)
            {
                if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x <= ROWSIMAGE / 3)
                {
                    int cnt = track.pointsEdgeLeft.size() - 1;
                    // 去除图片上面固定行的点集
                    for(int i = track.pointsEdgeLeft.size() - 1; track.pointsEdgeLeft[i].x <= ROWSIMAGE / 3; i--)
                    {
                        cnt--;
                    }
                    if(cnt <= 0)
                        cnt = 0;
                    track.pointsEdgeLeft.resize(cnt);
                }
            }
            //中线拟合去掉上面1/3行，处理右边线
            if(track.pointsEdgeRight.size() > 0)
            {
                if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x <= ROWSIMAGE / 3)
                {
                    int cnt = track.pointsEdgeRight.size() - 1;
                    // 去除图片上面固定行的点集
                    for(int i = track.pointsEdgeRight.size() - 1; track.pointsEdgeRight[i].x <= ROWSIMAGE / 3; i--)
                    {
                        cnt--;
                    }
                    if(cnt <= 0)
                        cnt = 0;
                    track.pointsEdgeRight.resize(cnt);
                }
            }

            // 判断是否出库判定是否出库，连续5帧图片都达到判定条件，则完成出库
            if (track.spurroad.size() < 3 && 
               ((track.stdevEdgeCal(track.pointsEdgeLeft,40) < 100 && track.stdevEdgeCal(track.pointsEdgeRight,40) == 1000) ||
                (track.stdevEdgeCal(track.pointsEdgeLeft,40) == 1000 && track.stdevEdgeCal(track.pointsEdgeRight,40) < 100) || 
                (track.stdevEdgeCal(track.pointsEdgeLeft,40) < 100 && track.stdevEdgeCal(track.pointsEdgeRight,40) < 100))) 
            // if ((track.stdevLeft < 90 && track.stdevRight == 1000) || (track.stdevLeft == 1000 && track.stdevRight < 90) || (track.stdevLeft < 90 && track.stdevRight < 90))
            {
                // 计数器计数，如果有2帧图片，则出库完成，进入空闲状态机
                counterExitOut++;
                if (counterExitOut >= 2)
                {
                    flag_garage = flag_garage_e::GARAGE_NONE;
                }
            }
            // 不满足判定条件，则计数器清零，要求连续5帧图片达到要求
            else
            {
                counterExitOut = 0;
            }

            // // 图像异常，退出该函数
            // if (track.pointsEdgeLeft.size() <= 2 || track.pointsEdgeRight.size() <= 2) 
            //     return;

            // 拐点的搜寻
            uint16_t rowBreakLeft = searchBreakLeft(track.pointsEdgeLeft);                       // 左上拐点搜索
            uint16_t rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, 180);        // 右下补线点搜索,从0开始到180行
            uint16_t rowBreakLeft_Down = searchBreakLeftDown(track.pointsEdgeLeft, 0, 180);      // 左下点搜索,从0开始到180行

            // 避免出库提前转向优化->当右下角或者左下角的拐点到达图像的下方20行的位置时，在开始转弯；参数在配置文件中，方便修改
            if (track.pointsEdgeRight[rowBreakRight].x <= params.corner_lines) 
            {
                // 优化右边的边线
                track.pointsEdgeRight.resize(rowBreakRight);
                if(rowBreakLeft_Down == 0 && rowBreakRight <= track.pointsEdgeLeft.size())
                    track.pointsEdgeLeft.resize(rowBreakRight);
                else
                    track.pointsEdgeLeft.resize(rowBreakLeft_Down);
                // 退出该函数，不进行补线
                return;
            }

            // // 如果左边的拐点比右边高，对齐
            // if (rowBreakLeft >= track.pointsEdgeRight.size())
            // {
            //     rowBreakLeft = track.pointsEdgeRight.size() - 1;
            // }

            // 开始进行出库补线，左库
            if(rowBreakRight == 0 || track.pointsEdgeRight[rowBreakRight].y < 270)
            {
                startPoint = POINT(225,315);
                track.pointsEdgeRight.clear();
            }
            else
            {
                startPoint = track.pointsEdgeRight[rowBreakRight];              // 入库补线起点
            }
            _pointLU = startPoint;                                              // dubug图中显示，补线的起点
            _pointRD = track.pointsEdgeLeft[rowBreakLeft];                      // dubug图中显示，补线的终点

            // // 依赖岔路补线
            // if (track.spurroad.size() >= 2)
            // {
            //     // 出库补线终点
            //     endPoint = searchBestSpurroad(track.spurroad);

            //     // 补线起点和终点正确性校验，判断起点和中点的位置关系是否正确
            //     if (startPoint.x > endPoint.x && startPoint.y > endPoint.y)
            //     {
            //         // 斑马线右边部分补线
            //         midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5);     // 出库补线中点
            //         _pointRU = endPoint;                                                                        // dubug图中显示，布线的中点
            //         // 三阶贝塞尔曲线拟合
            //         vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
            //         vector<POINT> modifyEdge = Bezier(0.04, repairPoints); 
            //         // 删除基础巡线找到的无效点
            //         track.pointsEdgeRight.resize(rowBreakRight);
            //         // 将拟合曲线加进去
            //         for (int i = 0; i < modifyEdge.size(); i++)
            //         {
            //             track.pointsEdgeRight.push_back(modifyEdge[i]);
            //         }

            //         // 斑马线左边部分补线
            //         startPoint = endPoint;
            //         endPoint = track.pointsEdgeLeft[rowBreakLeft];
            //         midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 入库补线中点
            //         // 三阶贝塞尔曲线拟合
            //         repairPoints = {startPoint, midPoint, endPoint};
            //         modifyEdge.resize(0);
            //         modifyEdge = Bezier(0.04, repairPoints);
            //         // 将拟合曲线加进去
            //         for (int i = 0; i < modifyEdge.size(); i++)
            //         {
            //             track.pointsEdgeRight.push_back(modifyEdge[i]);
            //         }
            //     }

            // 出库补线终点
            endPoint = POINT(10, 40);
            // 补线起点和终点正确性校验，判断起点和中点的位置关系是否正确
            if (startPoint.x > endPoint.x && startPoint.y > endPoint.y)
            {
                // 斑马线右边部分补线
                midPoint = POINT((startPoint.x + endPoint.x) * 0.35, (startPoint.y + endPoint.y) * 0.5);     // 出库补线中点
                _pointRU = endPoint;                                                                         // dubug图中显示，布线的中点
                // 三阶贝塞尔曲线拟合
                vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                vector<POINT> modifyEdge = Bezier(0.02, repairPoints);
                // 删除基础巡线找到的无效点
                if (track.pointsEdgeRight.size() != 0)
                    track.pointsEdgeRight.resize(rowBreakRight);
                // 将拟合曲线加进去
                for (int i = 0; i < modifyEdge.size(); i++)
                {
                    track.pointsEdgeRight.push_back(modifyEdge[i]);
                }
            }

            // 左边缘错误点优化
            track.pointsEdgeLeft.resize(5);
            // }
        }

        // 出右库
        if (flag_garage == flag_garage_e::GARAGE_OUT_RIGHT)
        {
            //中线拟合去掉上面1/3行，处理左边线
            if(track.pointsEdgeLeft.size() > 0)
            {
                if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x <= ROWSIMAGE / 3)
                {
                    int cnt = track.pointsEdgeLeft.size() - 1;
                    // 去除图片上面固定行的点集
                    for(int i = track.pointsEdgeLeft.size() - 1; track.pointsEdgeLeft[i].x <= ROWSIMAGE / 3; i--)
                    {
                        cnt--;
                    }
                    if(cnt <= 0)
                        cnt = 0;
                    track.pointsEdgeLeft.resize(cnt);
                }
            }
            //中线拟合去掉上面1/3行，处理右边线
            if(track.pointsEdgeRight.size() > 0)
            {
                if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x <= ROWSIMAGE / 3)
                {
                    int cnt = track.pointsEdgeRight.size() - 1;
                    // 去除图片上面固定行的点集
                    for(int i = track.pointsEdgeRight.size() - 1; track.pointsEdgeRight[i].x <= ROWSIMAGE / 3; i--)
                    {
                        cnt--;
                    }
                    if(cnt <= 0)
                        cnt = 0;
                    track.pointsEdgeRight.resize(cnt);
                }
            }

            // 判断是否出库判定是否出库，连续5帧图片都达到判定条件，则完成出库
            if (track.spurroad.size() < 3 && 
               ((track.stdevEdgeCal(track.pointsEdgeLeft,40) < 90 && track.stdevEdgeCal(track.pointsEdgeRight,40) == 1000) ||
                (track.stdevEdgeCal(track.pointsEdgeLeft,40) == 1000 && track.stdevEdgeCal(track.pointsEdgeRight,40) < 90) || 
                (track.stdevEdgeCal(track.pointsEdgeLeft,40) < 90 && track.stdevEdgeCal(track.pointsEdgeRight,40) < 90))) 
            {
                // 计数器计数，如果有4帧图片，则出库完成，进入空闲状态机
                counterExitOut++;
                if (counterExitOut >= 3)
                {
                    flag_garage = flag_garage_e::GARAGE_NONE;
                }
            }
            // 不满足判定条件，则计数器清零，要求连续5帧图片达到要求
            else
            {
                counterExitOut = 0;
            }

            // 图像异常，退出该函数
            if (track.pointsEdgeLeft.size() < 1 || track.pointsEdgeRight.size() < 1) 
                return;

            // 拐点的搜寻
            uint16_t rowBreakLeft = searchBreakLeftDown(track.pointsEdgeLeft, 0, 180);           // 左下补线点搜索，从0开始到180行
            uint16_t rowBreakRight_Down = searchBreakRightDown(track.pointsEdgeRight, 0, 180);   // 右下补线点搜索,从0开始到180行
            uint16_t rowBreakRight = searchBreakRight(track.pointsEdgeRight);                    // 右上拐点搜索

            // 避免出库提前转向优化->当右下角或者左下角的拐点到达图像的下方25行的位置时，在开始转弯
            if (track.pointsEdgeLeft[rowBreakLeft].x <= params.corner_lines) 
            {
                // 优化左右两边的边线
                track.pointsEdgeLeft.resize(rowBreakLeft);
                if(rowBreakRight_Down == 0 && rowBreakLeft <= track.pointsEdgeRight.size())
                    track.pointsEdgeRight.resize(rowBreakLeft);
                else
                    track.pointsEdgeRight.resize(rowBreakRight_Down);
                // 退出该函数，不进行补线
                return;
            }

            // // 如果左边的拐点比右边高，对齐
            // if (rowBreakRight >= track.pointsEdgeLeft.size())
            // {
            //     rowBreakRight = track.pointsEdgeLeft.size() - 1;
            // }

            // 开始进行出库补线，右库
            if(rowBreakLeft == 0 || track.pointsEdgeLeft[rowBreakLeft].y > 50)
            {
                startPoint = POINT(225,5);
                track.pointsEdgeLeft.clear();
            }
            else
            {
                startPoint = track.pointsEdgeLeft[rowBreakLeft];                // 入库补线起点
            }
            _pointLU = startPoint;                                              // dubug图中显示，补线的起点
            _pointRD = track.pointsEdgeLeft[rowBreakLeft];                      // dubug图中显示，补线的终点

            // // 依赖岔路补线
            // if (track.spurroad.size() > 2)
            // {
            //     // 出库补线终点
            //     endPoint = searchBestSpurroad(track.spurroad);

            //     // 补线起点和终点正确性校验，判断起点和中点的位置关系是否正确
            //     if (startPoint.x > endPoint.x && startPoint.y < endPoint.y)
            //     {
            //         // 斑马线右边部分补线
            //         midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5);     // 出库补线中点
            //         _pointRU = endPoint;                                                                        // dubug图中显示，布线的中点
            //         // 三阶贝塞尔曲线拟合
            //         vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
            //         vector<POINT> modifyEdge = Bezier(0.04, repairPoints); 
            //         // 删除基础巡线找到的无效点
            //         track.pointsEdgeLeft.resize(rowBreakLeft);
            //         // 将拟合曲线加进去
            //         for (int i = 0; i < modifyEdge.size(); i++)
            //         {
            //             track.pointsEdgeLeft.push_back(modifyEdge[i]);
            //         }

            //         // 斑马线左边部分补线
            //         startPoint = endPoint;
            //         endPoint = track.pointsEdgeRight[rowBreakRight];
            //         midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 入库补线中点
            //         // 三阶贝塞尔曲线拟合
            //         repairPoints = {startPoint, midPoint, endPoint};
            //         modifyEdge.resize(0);
            //         modifyEdge = Bezier(0.04, repairPoints);
            //         // 将拟合曲线加进去
            //         for (int i = 0; i < modifyEdge.size(); i++)
            //         {
            //             track.pointsEdgeLeft.push_back(modifyEdge[i]);
            //         }
            //     }

            // 出库补线终点
            endPoint = POINT(10, 280);
            // 补线起点和终点正确性校验，判断起点和中点的位置关系是否正确
            if (startPoint.x > endPoint.x && startPoint.y < endPoint.y)
            {
                // 斑马线右边部分补线
                midPoint = POINT((startPoint.x + endPoint.x) * 0.35, (startPoint.y + endPoint.y) * 0.5);     // 出库补线中点
                _pointRU = endPoint;                                                                         // dubug图中显示，布线的中点
                // 三阶贝塞尔曲线拟合
                vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                vector<POINT> modifyEdge = Bezier(0.02, repairPoints);
                // 删除基础巡线找到的无效点
                if (track.pointsEdgeLeft.size() != 0)
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                // 将拟合曲线加进去
                for (int i = 0; i < modifyEdge.size(); i++)
                {
                    track.pointsEdgeLeft.push_back(modifyEdge[i]);
                }
            }

            // 右边缘错误点优化
            track.pointsEdgeRight.resize(5);
            // }
        }
    }


	/**
	 * @brief 入库识别与路径规划
	 * @param track 基础赛道识别结果
	 */
	void garageEntryRec(TrackRecognition &track)
	{
        // 用于dubug绘图，识别入库距离
		_crosswalk = POINT(0, 0);
        // 定义是否准备入库的标志位
        static bool if_inStorage = false;
        // 第二圈准备入库的速度
        speed_ku = params.speed_readyend;

        // 识别拐点
        POINT crosswalk;
        if(flag_garage == GARAGE_IN_LEFT)
        {
            crosswalk = track.pointsEdgeLeft[garage_point_search_left(track.pointsEdgeLeft)];
        }
        else if(flag_garage == GARAGE_IN_RIGHT)
        {
            crosswalk = track.pointsEdgeRight[garage_point_search_right(track.pointsEdgeRight)];
        }
		_crosswalk = crosswalk;

        // 开始减速，准备入库
        if (crosswalk.x > params.slowdown_condition)
        {
            speed_ku = params.speed_in;
        }

        // 如果到大一定的行数，开始入库，进行入库补线
        if (crosswalk.x > params.disGarageEntry)
        {
            if_inStorage = true;
        }

        // 入左库
        if(flag_garage == GARAGE_IN_LEFT)
        {
            //到达一定距离，开始入库，重新规划路线
            if (if_inStorage)
            {
                // 定义一个计数器，用来使拐点消失后5帧继续补线
                static int num = 0;
                num++;

                // 拐点的搜寻        
                uint16_t edgeRightMid = sBreakLeftUp(track.pointsEdgeLeft);                                     // 右上拐点搜索

                // 入库补线
                POINT startPoint = POINT(ROWSIMAGE - 10, 315);                                                  // 入库补线起点:固定右下角
                POINT endPoint = track.pointsEdgeLeft[edgeRightMid];                                            // 入库补线终点
                POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5);   // 入库补线中点      
                // 三阶贝塞尔曲线拟合
                vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                vector<POINT> modifyEdge = Bezier(0.02, repairPoints); 
                // 清空基础赛道识别的路径，重新规划路径
                track.pointsEdgeRight.clear();
                track.pointsEdgeRight = modifyEdge;

                // 二次补线，左边完整补线
                startPoint = endPoint;
                endPoint = POINT(endPoint.x,1);                                                                        // 固定点左边中点补线
                midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);         // 入库补线中点
                // 三阶贝塞尔曲线拟合
                repairPoints = {startPoint, midPoint, endPoint};
                modifyEdge.resize(0);
                modifyEdge = Bezier(0.02, repairPoints);
                // 将拟合曲线加进去
                for (int i = 0; i < modifyEdge.size(); i++)
                {
                    track.pointsEdgeRight.push_back(modifyEdge[i]);
                }

                // 清空基础赛道识别的路径
                track.pointsEdgeLeft.clear();
                for(int i = 0;i < 5;i++)
                {
                    track.pointsEdgeLeft.push_back(POINT(ROWSIMAGE - track.rowCutBottom, i));
                }

                // 通过拐点判断是否已经大半进入车库
                if(track.spurroad.size() >= 1)
                {
                    // 计数器清零
                    num = 0;
                }
                // 判断已经大半进库，不用补线
                else
                {
                    if(num >= 3)
                    {
                        flag_garage = GARAGE_HALF_STOP;
                        num = 0;
                    }
                }
            }
        }
        else if(flag_garage == GARAGE_IN_RIGHT)
        {
            //到达一定距离，开始入库，重新规划路线
            if (if_inStorage)
            {
                // 定义一个计数器，用来使拐点消失后5帧继续补线
                static int num = 0;
                num++;

                // 拐点的搜寻        
                uint16_t edgeLeftMid = sBreakRightUp(track.pointsEdgeRight);                                    // 右上拐点搜索

                // 入库补线
                POINT startPoint = POINT(ROWSIMAGE - 10, 5);                                                    // 入库补线起点:固定左下角
                POINT endPoint = track.pointsEdgeRight[edgeLeftMid];                                            // 入库补线终点
                POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5);   // 入库补线中点      
                // 三阶贝塞尔曲线拟合
                vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                vector<POINT> modifyEdge = Bezier(0.02, repairPoints); 
                // 清空基础赛道识别的路径，重新规划路径
                track.pointsEdgeLeft.clear();
                track.pointsEdgeLeft = modifyEdge;

                // 二次补线，左边完整补线
                startPoint = endPoint;
                endPoint = POINT(endPoint.x,319);                                                                      // 固定点右边中点补线
                midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5);         // 入库补线中点
                // 三阶贝塞尔曲线拟合
                repairPoints = {startPoint, midPoint, endPoint};
                modifyEdge.resize(0);
                modifyEdge = Bezier(0.02, repairPoints);
                // 将拟合曲线加进去
                for (int i = 0; i < modifyEdge.size(); i++)
                {
                    track.pointsEdgeLeft.push_back(modifyEdge[i]);
                }

                // 清空基础赛道识别的路径
                track.pointsEdgeRight.clear();
                for(int i = 0;i < 5;i++)
                {
                    track.pointsEdgeRight.push_back(POINT(ROWSIMAGE - track.rowCutBottom, COLSIMAGE - i));
                }

                // 通过拐点判断是否已经大半进入车库
                if(track.spurroad.size() >= 1)
                {
                    // 计数器清零
                    num = 0;
                }
                // 判断已经大半进库，不用补线
                else
                {
                    if(num >= 3)
                    {
                        flag_garage = GARAGE_HALF_STOP;
                        num = 0;
                    }
                }
            }
        }
	}


    /**
     * @brief 车库识别结果图像绘制
     * @param track 赛道识别结果
     * @param trackImage 输入叠加图像
     */
    void drawImage(TrackRecognition &track, Mat &trackImage)
    {
        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(trackImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(trackImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        // 岔路点
        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(trackImage, Point(track.spurroad[i].y, track.spurroad[i].x), 3, Scalar(0, 0, 255), -1); // 红色点
        }

        // 绘制补线的三个关键节点
        circle(trackImage, Point(_pointRU.y, _pointRU.x), 5, Scalar(226, 43, 138), -1);
        circle(trackImage, Point(_pointRD.y, _pointRD.x), 5, Scalar(226, 43, 138), -1);
        circle(trackImage, Point(_pointLU.y, _pointLU.x), 5, Scalar(255, 0, 255), -1);

        // 显示相关信息
        // putText(trackImage, _Index, Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        putText(trackImage, "[1] Garage", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA); // 显示赛道识别类型

        string step = "GARAGE_OUT_READY";
        switch (flag_garage)
        {
            case flag_garage_e::GARAGE_NONE:
                step = "GARAGE_NONE";
                break;
            case flag_garage_e::GARAGE_DETECTION:
                step = "GARAGE_DETECTION";
                break;
            case flag_garage_e::GARAGE_OUT_READY:
                step = "GARAGE_OUT_READY";
                break;
            case flag_garage_e::GARAGE_OUT_LEFT:
                step = "GARAGE_OUT_LEFT!";
                break;
            case flag_garage_e::GARAGE_OUT_RIGHT:
                step = "GARAGE_OUT_RIGHT!";
                break;
            case flag_garage_e::GARAGE_IN_LEFT:
                step = "GARAGE_IN_LEFT!";
                break;
            case flag_garage_e::GARAGE_IN_RIGHT:
                step = "GARAGE_IN_RIGHT!";
                break;
            case flag_garage_e::GARAGE_PASS_LEFT:
                step = "GARAGE_PASS_LEFT!";
                break;
            case flag_garage_e::GARAGE_PASS_RIGHT:
                step = "GARAGE_PASS_RIGHT!";
                break;
            case flag_garage_e::GARAGE_STOP:
                step = "GARAGE_STOP!";
                break;
        }
        putText(trackImage, step, Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        // putText(trackImage, "Garage", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型

        // 传统视觉识别的车库下角点
        if(site_cross_x > 0)
        {
            vector<POINT> cross_site;
            for(int i = 0; i < COLSIMAGE; i++)
            {
                cross_site.push_back(POINT(site_cross_x, i));
            }
            for (int i = 0; i < cross_site.size(); i++)
            {
                circle(trackImage, Point(cross_site[i].y, cross_site[i].x), 1, Scalar(255, 255, 255), -1); // 白色点
            }
        }

        // 传统视觉识别的车库上角点
        if(site_cross_y > 0)
        {
            vector<POINT> cross_site;
            for(int i = 0; i < COLSIMAGE; i++)
            {
                cross_site.push_back(POINT(site_cross_y, i));
            }
            for (int i = 0; i < cross_site.size(); i++)
            {
                circle(trackImage, Point(cross_site[i].y, cross_site[i].x), 1, Scalar(255, 255, 255), -1); // 白色点
            }
        }

        // 传统视觉识别的斑马线中点
        if(track.garageEnable.x == 1)
        {
            vector<POINT> cross_site;
            for(int i = 0; i < COLSIMAGE; i++)
            {
                cross_site.push_back(POINT(track.garageEnable.y, i));
            }
            for (int i = 0; i < cross_site.size(); i++)
            {
                circle(trackImage, Point(cross_site[i].y, cross_site[i].x), 1, Scalar(255, 255, 255), -1); // 白色点
            }
        }

        // ai识别框的中点绘图
        if (_crosswalk.x > 0)
        {
            string str = to_string(_crosswalk.x);
            putText(trackImage, str, Point(COLSIMAGE / 2, ROWSIMAGE - 100), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        }
    }


    /**
     * @brief 加载配置参数Json
     */
    void loadParams(void) 
    {
        string jsonPath = "../src/config/garage.json";
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

