#pragma once
/**
 * @file ring_recognition.cpp
 * @author pxx ()
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2023-05-09
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"

using namespace cv;
using namespace std;

class RingRecognition
{
public:
    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void)
    {
        ringType = RingType::RingNone; // 环岛类型
        ringStep = RingStep::None;     // 环岛处理阶段
        ring_cnt = 0;
        counterSpurroad = 0;
    }
    void print(void)
    {
        cout << ringStep << endl;
    }

    /**
     * @brief 环岛识别与行径规划
     *
     * @param track 基础赛道识别结果
     * @param imagePath 赛道路径图像
     */
    bool ringRecognition(TrackRecognition &track, Mat img_bin)
    {
        if (track.pointsEdgeRight.size() < ROWSIMAGE / 8 || track.pointsEdgeLeft.size() < ROWSIMAGE / 8) //环岛有效行限制
        {
            return ringType;
        }

        static uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测
        if (track.garageEnable.x)
        {
            counterShield = 0;
        }
        if (counterShield < 40)
        {
            counterShield++;
            return false;
        }


        pointBreakD = POINT(0, 0);
        pointBreakU = POINT(0, 0);

        //[1]左右环岛判断
        if(ringType == RingType::RingNone && ringStep == RingStep::None)
        {
            uint16_t rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE / 2);
            uint16_t rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft, 0, ROWSIMAGE / 2);

            if(rowBreakLeftDown != 0 && rowBreakRightDown == 0 && track.stdevLeft > 120 && track.stdevRight < 50
                && abs(track.pointsEdgeRight[0].y - track.pointsEdgeRight[ROWSIMAGE / 2].y) > 5
                && track.widthBlock[rowBreakLeftDown + 5].y > COLSIMAGE / 2)
            {
                ring_cnt++;
                pointBreakD = track.pointsEdgeLeft[rowBreakLeftDown];
                if(ring_cnt > 2)
                {
                    ringType = RingType::RingLeft;
                }
            }
            else if(rowBreakLeftDown == 0 && rowBreakRightDown != 0 && track.stdevLeft < 50 && track.stdevRight > 120
                && abs(track.pointsEdgeLeft[0].y - track.pointsEdgeLeft[ROWSIMAGE / 2].y) > 5
                && track.widthBlock[rowBreakRightDown + 5].y > COLSIMAGE / 2)
            {
                ring_cnt++;
                pointBreakD = track.pointsEdgeRight[rowBreakLeftDown];
                if(ring_cnt > 2)
                {
                    ringType = RingType::RingRight;
                }
            }
            else
            {
                ring_cnt = 0;
            }

            {
                //暂时感觉不需要处理
            }

        }
        else if(ringType != RingType::RingNone && ringStep == RingStep::None)
        {
			// counterExit++;
			// if (counterExit > 60) {
			//   reset();
			//   return false;
			// }

            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, ROWSIMAGE / 2);
                uint16_t rowBreakLeftU = searchBreakLeftDown(track.pointsEdgeLeft, rowBreakLeftD + 15, track.pointsEdgeLeft.size());
                
                pointBreakD = track.pointsEdgeLeft[rowBreakLeftD];
                if(rowBreakLeftD && rowBreakLeftU)
                {
                    counterSpurroad++;
                    pointBreakD = track.pointsEdgeLeft[rowBreakLeftD];
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeftU];
                    line(track.pointsEdgeLeft, rowBreakLeftD, rowBreakLeftU);
                }
                else if(counterSpurroad > 1)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Entering;
                }
                
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE / 2);
                uint16_t rowBreakRightU = searchBreakRightDown(track.pointsEdgeRight, rowBreakRightD + 15, track.pointsEdgeRight.size());
                

                if(rowBreakRightD && rowBreakRightU)
                {
                    counterSpurroad++;
                    pointBreakD = track.pointsEdgeRight[rowBreakRightD];
                    pointBreakU = track.pointsEdgeRight[rowBreakRightU];
                    line(track.pointsEdgeRight, rowBreakRightD, rowBreakRightU);
                }
                else if(counterSpurroad > 1)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Entering;
                }
            }

        }
        else if(ringStep == RingStep::Entering)
        {
            if(ringType == RingType::RingLeft)
            {
                POINT corner(0, 0);
                uint16_t rowBreakLeftD = 0;
                uint16_t spurroad_item = 0;
                if(track.spurroad.size() == 0)
                {
                    rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size());
                }
                else if(track.spurroad.size() == 1)
                {
                    //寻找岔路行对应边线下标
                    corner = track.spurroad[0];
                    spurroad_item = abs(corner.x - track.pointsEdgeLeft[0].x);
                    rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, spurroad_item);
                }
                else
                {
                    for(int i = 0; i < track.spurroad.size(); i++)
                    {
                        if(track.spurroad[i].x > corner.x)
                        {
                            corner = track.spurroad[i];
                        }
                        spurroad_item = abs(corner.x - track.pointsEdgeLeft[0].x);
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, spurroad_item);
                    }
                }

                pointBreakD = track.pointsEdgeLeft[0];
                pointBreakU = track.pointsEdgeLeft[rowBreakLeftD];
                line(track.pointsEdgeLeft, 0, rowBreakLeftD);
                if(corner.x)
                {
                    counterSpurroad++;
                    line(track.pointsEdgeRight, rowBreakLeftD, corner);
                    track.pointsEdgeLeft.resize(spurroad_item);
                    track.pointsEdgeRight.resize(spurroad_item);

                    const uint16_t width_thresh = track.pointsEdgeRight[spurroad_item - 1].y - track.pointsEdgeLeft[spurroad_item - 1].y;
                    uint16_t mid = (track.pointsEdgeLeft[spurroad_item - 1].y + track.pointsEdgeRight[spurroad_item - 1].y) / 2;
                    POINT left(0, 0);
                    POINT right(0, 0);
                    for(int i = corner.x - 1; i > ROWSIMAGE / 3; i--)
                    {
                        int j = mid;
                        for(j = mid; j < COLSIMAGE - 5; j++)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j + 1) == 0 
                                && img_bin.at<uchar>(i, j + 2) == 0 && img_bin.at<uchar>(i, j + 3) == 0)
                            {
                                right = POINT(i, j);
                                break;
                            }
                        }
                        if(j == COLSIMAGE - 5)
                        {
                            right = POINT(i, COLSIMAGE - 1);
                        }

                        for(j = mid; j > 5; j--)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j - 1) == 0 
                                && img_bin.at<uchar>(i, j - 2) == 0 && img_bin.at<uchar>(i, j - 3) == 0)
                            {
                                left = POINT(i, j);
                                break;
                            }
                        }
                        if(j == 5)
                        {
                            left = POINT(i, 0);
                        }
                        uint16_t width = abs(right.y - left.y);
                        if(width < COLSIMAGE / 10 || width > width_thresh)
                        {
                            break;
                        }
                        track.pointsEdgeLeft.push_back(left);
                        track.pointsEdgeRight.push_back(right);
                        track.widthBlock.push_back(POINT(i, width));

                        mid = (right.y + left.y) / 2;
                    }
                }
                else if(track.spurroad.size() == 0 && counterSpurroad > 1)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Inside;
                }
            }
            else if(ringType == RingType::RingRight)
            {
                POINT corner(0, 0);
                uint16_t spurroad_item = 0;
                uint16_t rowBreakRightD = 0;
                if(track.spurroad.size() == 0)
                {
                    rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size());
                }
                else if(track.spurroad.size() == 1)
                {
                    //寻找岔路行对应边线下标
                    corner = track.spurroad[0];
                    spurroad_item = abs(corner.x - track.pointsEdgeRight[0].x);
                    rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, spurroad_item);
                }
                else
                {
                    for(int i = 0; i < track.spurroad.size(); i++)
                    {
                        if(track.spurroad[i].x > corner.x)
                        {
                            corner = track.spurroad[i];
                        }
                        spurroad_item = abs(corner.x - track.pointsEdgeRight[0].x);
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, spurroad_item);
                    }
                }

                pointBreakD = track.pointsEdgeRight[0];
                pointBreakU = track.pointsEdgeRight[rowBreakRightD];
                line(track.pointsEdgeRight, 0, rowBreakRightD);
                if(corner.x)
                {
                    counterSpurroad++;
                    line(track.pointsEdgeLeft, rowBreakRightD, corner);
                    track.pointsEdgeLeft.resize(spurroad_item);
                    track.pointsEdgeRight.resize(spurroad_item);

                    const uint16_t width_thresh = track.pointsEdgeRight[spurroad_item - 1].y - track.pointsEdgeLeft[spurroad_item - 1].y;
                    uint16_t mid = (track.pointsEdgeLeft[spurroad_item - 1].y + track.pointsEdgeRight[spurroad_item - 1].y) / 2;
                    POINT left(0, 0);
                    POINT right(0, 0);
                    for(int i = corner.x - 1; i > ROWSIMAGE / 3; i--)
                    {
                        int j = mid;
                        for(j = mid; j < COLSIMAGE - 5; j++)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j + 1) == 0 
                                && img_bin.at<uchar>(i, j + 2) == 0 && img_bin.at<uchar>(i, j + 3) == 0)
                            {
                                right = POINT(i, j);
                                break;
                            }
                        }
                        if(j == COLSIMAGE - 5)
                        {
                            right = POINT(i, COLSIMAGE - 1);
                        }

                        for(j = mid; j > 5; j--)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j - 1) == 0 
                                && img_bin.at<uchar>(i, j - 2) == 0 && img_bin.at<uchar>(i, j - 3) == 0)
                            {
                                left = POINT(i, j);
                                break;
                            }
                        }
                        if(j == 5)
                        {
                            left = POINT(i, 0);
                        }
                        uint16_t width = abs(right.y - left.y);
                        if(width < COLSIMAGE / 10 || width > width_thresh)
                        {
                            break;
                        }
                        track.pointsEdgeLeft.push_back(left);
                        track.pointsEdgeRight.push_back(right);
                        track.widthBlock.push_back(POINT(i, width));

                        mid = (right.y + left.y) / 2;
                    }
                }
                else if(track.spurroad.size() == 0 && counterSpurroad > 1)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Inside;
                }
            }
        }
        else if(ringStep == RingStep::Inside)
        {
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size() - 10);
                //寻找左边跳变点
                uint16_t rowBreakLeft = 0;
                uint16_t counter = 0;
                for(int i = track.pointsEdgeLeft.size() - 3; i > 60; i--)
                {
                    if(track.pointsEdgeLeft[i].y < 5)
                    {
                        counter++;
                    }
                    else
                    {
                        counter = 0;
                    }
                    if(counter > 5)
                    {
                        rowBreakLeft = i + 5;
                        break;
                    }
                }
                if(rowBreakRight)
                {
                    counterSpurroad++;

                    pointBreakD = track.pointsEdgeRight[rowBreakRight];
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                    line(track.pointsEdgeRight, rowBreakRight, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    track.pointsEdgeRight.resize(rowBreakLeft);
                }
                else if(!rowBreakRight && counterSpurroad > 2)
                {
                    pointBreakD = track.pointsEdgeRight[0];
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                    line(track.pointsEdgeRight, rowBreakRight, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    track.pointsEdgeRight.resize(rowBreakLeft);

                    ringStep = RingStep::Exiting;
                    counterSpurroad = 0;
                }
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakLeft = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size() - 10);
                //寻找右边跳变点
                uint16_t rowBreakRight = 0;
                uint16_t counter = 0;
                for(int i = track.pointsEdgeRight.size() - 3; i > 60; i--)
                {
                    if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                    {
                        counter++;
                    }
                    else
                    {
                        counter = 0;
                    }
                    if(counter > 5)
                    {
                        rowBreakRight = i + 5;
                        break;
                    }
                }
                if(rowBreakLeft)
                {
                    counterSpurroad++;

                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    pointBreakD = track.pointsEdgeLeft[rowBreakLeft];
                    line(track.pointsEdgeLeft, rowBreakLeft, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakRight);
                    track.pointsEdgeRight.resize(rowBreakRight);
                }
                else if(!rowBreakLeft && counterSpurroad > 2)
                {
                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    pointBreakD = track.pointsEdgeLeft[0];
                    line(track.pointsEdgeLeft, rowBreakLeft, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakRight);
                    track.pointsEdgeRight.resize(rowBreakRight);

                    ringStep = RingStep::Exiting;
                    counterSpurroad = 0;
                }
            }
        }
        else if(ringStep == RingStep::Exiting)
        {
            //寻找左边跳变点
            uint16_t rowBreakLeft = 0;
            uint16_t counter = 0;
            for(int i = track.pointsEdgeLeft.size() - 3; i > 60; i--)
            {
                if(track.pointsEdgeLeft[i].y < 5)
                {
                    counter++;
                }
                else
                {
                    counter = 0;
                }
                if(counter > 5)
                {
                    rowBreakLeft = i + 5;
                    break;
                }
            }
            //寻找右边跳变点
            uint16_t rowBreakRight = 0;
            counter = 0;
            for(int i = track.pointsEdgeRight.size() - 3; i > 60; i--)
            {
                if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                {
                    counter++;
                }
                else
                {
                    counter = 0;
                }
                if(counter > 5)
                {
                    rowBreakRight = i + 5;
                    break;
                }
            }

            if(ringType == RingType::RingLeft)
            {
                if(rowBreakRight > ROWSIMAGE / 4)
                {
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                    pointBreakU.y += COLSIMAGE / 10;
                    pointBreakD = track.pointsEdgeRight[0];
                    line(track.pointsEdgeRight, 0, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    track.pointsEdgeRight.resize(rowBreakLeft);
                }
                else
                {
                    ringStep = RingStep::Finish;
                }
            }
            else if(ringType == RingType::RingRight)
            {
                if(rowBreakLeft > ROWSIMAGE)
                {
                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    pointBreakU.y -= COLSIMAGE / 10;
                    pointBreakD = track.pointsEdgeLeft[0];
                    line(track.pointsEdgeLeft, 0, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakRight);
                    track.pointsEdgeRight.resize(rowBreakRight);
                }
                else
                {
                    ringStep = RingStep::Finish;
                }
            }
        }
        else if(ringStep == RingStep::Finish)
        {
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeft = searchBreakLeftUp(track.pointsEdgeLeft);
                if(rowBreakLeft)
                {
                    counterSpurroad++;
                    pointBreakD = track.pointsEdgeLeft[0];
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                    line(track.pointsEdgeLeft, 0, pointBreakU);
                }
                else if(!rowBreakLeft && counterSpurroad > 3)
                {
                    reset();
                }
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakRight = searchBreakRightUp(track.pointsEdgeRight);
                if(rowBreakRight)
                {
                    counterSpurroad++;
                    pointBreakD = track.pointsEdgeRight[0];
                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    line(track.pointsEdgeRight, 0, pointBreakU);
                }
                else if(!rowBreakRight && counterSpurroad > 3)
                {
                    reset();
                }
            }
        }

        if (ringType == RingType::RingNone)
            return false;
        else
            return true;
    }

    void drawImage(TrackRecognition track, Mat &Image)
    {
        // 绘制边缘点
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制岔路点
        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6, Scalar(0, 0, 255), -1); // 红色点
        }

        // 绘制补线点
        {
            circle(Image, Point(pointBreakU.y, pointBreakU.x), 5, Scalar(255, 0, 255), -1); // 上补线点：粉色
            circle(Image, Point(pointBreakD.y, pointBreakD.x), 5, Scalar(226, 43, 138), -1); // 下补线点：紫色
        }
        putText(Image, to_string(ringStep), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);
        if(ringType == RingType::RingLeft)
        {
            putText(Image, "Ring L", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型
        }
        else if(ringType == RingType::RingRight)
        {
            putText(Image, "Ring R", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型
        }
    }

private:

    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Exiting,  // 出环
        Finish    // 环任务结束
    };

    RingType ringType = RingType::RingNone; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段

    /**
     * @brief 搜索环岛赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakLeft = 1;
        uint16_t counter = 0;

        if(row_start == 0)
        {
            row_start++;
        }
        if(row_end > pointsEdgeLeft.size())
        {
            row_end = pointsEdgeLeft.size();
        }
        if(row_start > pointsEdgeLeft.size())
        {
            row_start = pointsEdgeLeft.size();
        }
        for (int i = row_start; i < row_end; i++) // 寻找左边跳变点
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
                else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakLeft;
                }
            }
        }

        return 0;
    }

    /**
     * @brief 搜索环岛赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakRight = 1;
        uint16_t counter = 0;

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
        for (int i = row_start; i < row_end; i++) // 寻找右边跳变点
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
                else if (pointsEdgeRight[i].y > pointsEdgeRight[rowBreakRight].y) // 突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakRight;
                }
            }
        }

        return 0;
    }

    /**
     * @brief 搜索环岛赛道突变行（左上）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = 0;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeLeft.size() - 5; i > 0; i--)
        {
            if (pointsEdgeLeft[i].y > 5 & abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 5)
            {
                rowBreakLeftUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeLeft[i].y <= 5 && counterFilter > 10)
            {
                counter++;
                if (counter > 3)
                    return rowBreakLeftUp;
            }
        }

        return 0;
    }

    /**
     * @brief 搜索环岛赛道突变行（右上）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightUp = 0;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeRight.size() - 5; i > 0; i--)
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2 & abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 5)
            {
                rowBreakRightUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 3)
                    return rowBreakRightUp;
            }
        }

        return 0;
    }

    uint16_t counterSpurroad = 0; // 岔路计数器
    uint16_t ring_cnt = 0; // 环岛检测确认计数器
	uint16_t counterExit = 0;	  // 异常退出计数器

    POINT pointBreakU;
    POINT pointBreakD;
};
