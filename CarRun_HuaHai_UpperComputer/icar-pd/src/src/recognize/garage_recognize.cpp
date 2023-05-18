#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "../../include/json.hpp"
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../motion_controller.cpp"


using namespace cv;
using namespace std;


class GarageRecognition
{
private:
    /**
     * @brief 控制器核心参数
     */
    struct Params
    {
        uint16_t garage_run = 0;    // 选择出库方式
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, garage_run); // 添加构造函数
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
        GARAGE_STOP,        // 进库完毕, 停车
    };


    // 定义入库状态机枚举变量,初始值为none
    flag_garage_e flag_garage = GARAGE_NONE;

    // 定义用于绘图的点，描述布线的三个点
    POINT _pointRU;
    POINT _pointLU;
    POINT _pointRD;

    // 读取控制参数
    Params params;


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
            if (counterCrosswalk >= 4 && counterSessionTwo <= 8)
            {
                counterSessionTwo = 0;
                counterCrosswalk = 0;
                return true;
            }
            // 不满足要求则清零计数器，并返回没有检测到起点车库
            else if (counterSessionTwo > 8)
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
     * @brief 冒泡法求取集合中值，由小到大
     * @param vec 输入集合
     * @return int 中值
     */
    int getMiddleValue(vector<int> vec)
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
        return vec[(int)vec.size() / 2];
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
        int colBest = getMiddleValue(cols); 
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
     * @brief 搜索环岛赛道突变行（左下）
     * @param pointsEdgeLeft 左边缘线
     * @param row_start 起始搜寻行
     * @param row_end 终止搜寻行
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakLeft = 1;
        uint16_t counter = 0;

        if(pointsEdgeLeft.size() == 0)
        {
            return 0;
        }
        
        if(row_start == 0)
        {
            row_start++;
        }
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
        return 0;
    }


    /**
     * @brief 搜索环岛赛道突变行（右下）
     * @param pointsEdgeRight 右边缘线
     * @param row_start 起始搜寻行
     * @param row_end 终止搜寻行
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakRight = 1;
        uint16_t counter = 0;

        if(pointsEdgeRight.size() == 0)
        {
            return 0;
        }

        if(row_start == 0)
        {
            row_start++;
        }
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
        // 没有搜寻到，返回0
        return 0;
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
    // 变量相关定义
    float   garage_route    = 0;                    // 车库编码器积分
    bool    garage_stop     = false;                // 车库停车标志位
    int     garage_num      = 0;                    // 记录当前第几次到达车库
    int     none_left       = 0, none_right = 0;
    int     have_left       = 0, have_right = 0;


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
    bool garage_contral(vector<PredictResult> predict, TrackRecognition &track) 
    {
        // 返回状态值
        bool status = false;

        // 检测斑马线
        status = check_garage(predict);

        // 判断目前的赛道类型->检测到斑马线，进行出库入库操作
        if(flag_garage == flag_garage_e::GARAGE_OUT_LEFT || flag_garage == flag_garage_e::GARAGE_OUT_RIGHT ||
           flag_garage == flag_garage_e::GARAGE_IN_LEFT || flag_garage == flag_garage_e::GARAGE_IN_RIGHT ||
           flag_garage == flag_garage_e::GARAGE_PASS_LEFT || flag_garage == flag_garage_e::GARAGE_PASS_RIGHT)
        {
            // 返回状态
            status = true;
            // 相关状态机的处理
            run_garage(track, predict);
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
     * @param predict ai预测结果
     * @return bool
     */
    bool check_garage(vector<PredictResult> predict) 
    {
        // 定义一个静态变量来进行计数，当超过20帧图像还没有检测到出库，则初始位置不在车库中
        static uint16_t count_to_CarNotInGarage = 0;
        // 定义一个局部变量，用于记录目前也没有检测到斑马线，返回出去
        bool if_garage = false;
        // 准备出库状态机，当检测到起点斑马线时，进入出库状态机，清零计数器
        if (flag_garage == flag_garage_e::GARAGE_OUT_READY) 
        {
            // 返回真，说明检测到起点，进入出库状态机，局部变量返回真，检测到斑马线了
            if(startingCheck(predict))
            {
                // 根据配置文件，0为左库，跑两圈入库
                if(params.garage_run == 0)
                {
                    flag_garage = flag_garage_e::GARAGE_OUT_LEFT;
                }
                // 根据配置文件，1为右库，跑两圈入库
                else if(params.garage_run == 1)
                {
                    flag_garage = flag_garage_e::GARAGE_OUT_RIGHT;
                }
                count_to_CarNotInGarage = 0;
                if_garage = true;
            }
            // 返回假，说明可能没有检测到8帧图像达到判断条件；或者车初始位置不在车库中
            else
            {
                // 计数器自增
                count_to_CarNotInGarage++;
                // 判断计数器值，如果超过20帧还没有检测到起点，说明不在车库中，改变状态机，默认已经完成出库，并清零计数器
                if(count_to_CarNotInGarage >= 20)
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
            // 返回真，说明检测到斑马线，改变状态机
            if(searchCrosswalk_if_recognize(predict))
            {
                flag_garage = GARAGE_DETECTION;
                // 返回真，检测到斑马线，用于清空跑完一圈其他
                if_garage = true;
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
     * @param predict ai预测结果
     * @return void
     */
    void run_garage(TrackRecognition &track, vector<PredictResult> predict) 
    {
        switch (flag_garage)
        {
            // 出库，左库
            case GARAGE_OUT_LEFT:
                garageExitRecognition(track, predict);
                break;

        /* ********************************************************* */
            // 出库，右库
            case GARAGE_OUT_RIGHT:
                garageExitRecognition(track, predict);
                break;

        /* ********************************************************* */
            // 入库，左库
            case GARAGE_IN_LEFT:

                break;

        /* ********************************************************* */
            // 入库，右库
            case GARAGE_IN_RIGHT:

                break;

        /* ********************************************************* */
            // 左库，不入库
            case GARAGE_PASS_LEFT:

                break;

        /* ********************************************************* */
            // 右库，不入库
            case GARAGE_PASS_RIGHT:

                break;

        /* ********************************************************* */
            default:
                break;
        }
    }


    /**
     * @brief 出库识别与路径规划
     * @param track
     * @param predict ai预测
     */
    void garageExitRecognition(TrackRecognition &track, vector<PredictResult> predict)
    {
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
            // 调用ai来检测斑马线，来判定是否出库，连续5帧图片都达到判定条件，则完成出库
            if ((track.spurroad.size() < 3 || (track.stdevLeft < 100 && track.stdevRight < 100)) && !searchCrosswalk_if_recognize(predict)) 
            {
                // 计数器计数，如果有5帧图片，则出库完成，进入空闲状态机
                counterExitOut++;
                if (counterExitOut >= 5)
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
            if (track.pointsEdgeLeft.size() <= 1 || track.pointsEdgeRight.size() <= 1) 
                return;

            // 拐点的搜寻
            uint16_t rowBreakLeft = searchBreakLeft(track.pointsEdgeLeft);                      // 左上拐点搜索
            uint16_t rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, 45);        // 右下补线点搜索,从0开始到20行
            uint16_t rowBreakLeft_Down = searchBreakLeftDown(track.pointsEdgeLeft, 0, 45);      // 左下点搜索,从0开始到20行

            // 避免出库提前转向优化->当右下角或者左下角的拐点到达图像的下方20行的位置时，在开始转弯
            if (track.pointsEdgeRight[rowBreakRight].x <= ROWSIMAGE - 40) 
            {
                // 优化右边的边线
                track.pointsEdgeRight.resize(rowBreakRight);
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
            startPoint = track.pointsEdgeRight[rowBreakRight];                  // 入库补线起点
            _pointLU = startPoint;                                              // dubug图中显示，补线的起点
            _pointRD = track.pointsEdgeLeft[rowBreakLeft];                      // dubug图中显示，补线的终点

            // 依赖岔路补线
            if (track.spurroad.size() > 2)
            {
                // 入库补线终点
                endPoint = searchBestSpurroad(track.spurroad);

                // 补线起点和终点正确性校验，判断起点和中点的位置关系是否正确
                if (startPoint.x > endPoint.x && startPoint.y > endPoint.y)
                {
                    // 斑马线右边部分补线
                    midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5);     // 入库补线中点
                    _pointRU = endPoint;                                                                        // dubug图中显示，布线的中点
                    // 三阶贝塞尔曲线拟合
                    vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                    vector<POINT> modifyEdge = Bezier(0.04, repairPoints); 
                    // 删除基础巡线找到的无效点
                    track.pointsEdgeRight.resize(rowBreakRight);
                    // 将拟合曲线加进去
                    for (int i = 0; i < modifyEdge.size(); i++)
                    {
                        track.pointsEdgeRight.push_back(modifyEdge[i]);
                    }

                    // 斑马线左边部分补线
                    startPoint = endPoint;
                    endPoint = track.pointsEdgeLeft[rowBreakLeft];
                    midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 入库补线中点
                    // 三阶贝塞尔曲线拟合
                    repairPoints = {startPoint, midPoint, endPoint};
                    modifyEdge.resize(0);
                    modifyEdge = Bezier(0.04, repairPoints);
                    // 将拟合曲线加进去
                    for (int i = 0; i < modifyEdge.size(); i++)
                    {
                        track.pointsEdgeRight.push_back(modifyEdge[i]);
                    }
                }

                // 左边缘错误点优化
                track.pointsEdgeLeft.resize(rowBreakLeft);
            }
        }

        // 出右库
        if (flag_garage == flag_garage_e::GARAGE_OUT_RIGHT)
        {
            // 调用ai来检测斑马线，来判定是否出库，连续5帧图片都达到判定条件，则完成出库
            if (!searchCrosswalk_if_recognize(predict)) 
            {
                // 计数器计数，如果有5帧图片，则出库完成，进入空闲状态机
                counterExitOut++;
                if (counterExitOut >= 5)
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
            if (track.pointsEdgeLeft.size() <= 1 || track.pointsEdgeRight.size() <= 1) 
                return;

            // 拐点的搜寻
            uint16_t rowBreakLeft = searchBreakLeftDown(track.pointsEdgeRight, 0, 45);          // 左下补线点搜索，从0开始到20行
            uint16_t rowBreakRight_Down = searchBreakRightDown(track.pointsEdgeRight, 0, 45);   // 右下补线点搜索,从0开始到20行
            uint16_t rowBreakRight = searchBreakRight(track.pointsEdgeLeft);                    // 右上拐点搜索

            // 避免出库提前转向优化->当右下角或者左下角的拐点到达图像的下方25行的位置时，在开始转弯
            if (track.pointsEdgeLeft[rowBreakLeft].x <= ROWSIMAGE - 25) 
            {
                // 优化左右两边的边线
                track.pointsEdgeLeft.resize(rowBreakLeft);
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
            startPoint = track.pointsEdgeLeft[rowBreakLeft];                    // 入库补线起点
            _pointLU = startPoint;                                              // dubug图中显示，补线的起点
            _pointRD = track.pointsEdgeLeft[rowBreakLeft];                      // dubug图中显示，补线的终点

            // 依赖岔路补线
            if (track.spurroad.size() > 2)
            {
                // 入库补线终点
                endPoint = searchBestSpurroad(track.spurroad);

                // 补线起点和终点正确性校验，判断起点和中点的位置关系是否正确
                if (startPoint.x > endPoint.x && startPoint.y < endPoint.y)
                {
                    // 斑马线右边部分补线
                    midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5);     // 入库补线中点
                    _pointRU = endPoint;                                                                        // dubug图中显示，布线的中点
                    // 三阶贝塞尔曲线拟合
                    vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                    vector<POINT> modifyEdge = Bezier(0.04, repairPoints); 
                    // 删除基础巡线找到的无效点
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    // 将拟合曲线加进去
                    for (int i = 0; i < modifyEdge.size(); i++)
                    {
                        track.pointsEdgeLeft.push_back(modifyEdge[i]);
                    }

                    // 斑马线左边部分补线
                    startPoint = endPoint;
                    endPoint = track.pointsEdgeRight[rowBreakRight];
                    midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 入库补线中点
                    // 三阶贝塞尔曲线拟合
                    repairPoints = {startPoint, midPoint, endPoint};
                    modifyEdge.resize(0);
                    modifyEdge = Bezier(0.04, repairPoints);
                    // 将拟合曲线加进去
                    for (int i = 0; i < modifyEdge.size(); i++)
                    {
                        track.pointsEdgeLeft.push_back(modifyEdge[i]);
                    }
                }

                // 右边缘错误点优化
                track.pointsEdgeRight.resize(rowBreakRight);
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

        // if (_crosswalk.x > 0)
        // {
        //     string str = to_string(_crosswalk.x);
        //     putText(trackImage, str, Point(COLSIMAGE / 2, ROWSIMAGE - 100), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        // }
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

