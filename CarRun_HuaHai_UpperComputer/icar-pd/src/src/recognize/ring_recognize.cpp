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
    RingRecognition()
    {
        ;
    }
    ~RingRecognition()
    {
        ;
    }
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
        counterShield = 0;
    }
    void print(void)
    {
        cout << "RingStep: " << ringStep << endl;
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

        if (track.garageEnable.y > ROWSIMAGE / 3 && ringType == RingType::RingNone)
        {
            counterShield = 0;
        }
        if (counterShield < 30)
        {
            counterShield++;
            return false;
        }


        pointBreakD = POINT(0, 0);
        pointBreakU = POINT(0, 0);

        //[1]左右环岛判断
        if(ringType == RingType::RingNone && ringStep == RingStep::None)
        {
            uint16_t rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight, 0, COLSIMAGE / 2);
            uint16_t rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft, 0, COLSIMAGE / 2);

            if(rowBreakLeftDown != 0 && rowBreakRightDown == 0
                && ((track.stdevLeft > 120 && track.stdevRight < 60) || (track.stdevLeft > 200 && track.stdevRight < 80))
                && abs(track.pointsEdgeRight[0].y - track.pointsEdgeRight[track.pointsEdgeRight.size() / 2].y) > 5
                && track.widthBlock[rowBreakLeftDown + 5].y > COLSIMAGE / 2 
                && track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y > COLSIMAGE / 2)
            {
                // for(int i = rowBreakLeftDown; i < rowBreakLeftDown + 50; i++)
                // {
                //     uint16_t counter = 0;
                //     if(track.pointsEdgeLeft[i].y < 5)
                //         counter++;
                //     if(counter> 30)
                //     {
                //         ring_cnt++;
                //         break;
                //     }
                // }
                ring_cnt++;
                if(ring_cnt > 1)
                {
                    // for(int i = 0; i < track.pointsEdgeLeft.size() / 2; i++)
                    // {
                    //     if(track.pointsEdgeLeft[i].y < 5)
                    //         counterSpurroad++;
                    // }
                    // if(counterSpurroad > 30)
                    //     ringStep = RingStep::Entering;

                    counterSpurroad = 0;
                    ringType = RingType::RingLeft;
                }
            }
            else if(rowBreakLeftDown == 0 && rowBreakRightDown != 0 
                && ((track.stdevLeft < 60 && track.stdevRight > 120) || (track.stdevLeft < 80 && track.stdevRight > 200))
                && abs(track.pointsEdgeLeft[0].y - track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 2].y) > 5
                && track.widthBlock[rowBreakRightDown + 5].y > COLSIMAGE / 2
                && track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y < COLSIMAGE / 2)
            {
                // for(int i = rowBreakRightDown; i < rowBreakRightDown + 50; i++)
                // {
                //     uint16_t counter = 0;
                //     if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                //         counter++;
                //     if(counter> 30)
                //     {
                //         ring_cnt++;
                //         break;
                //     }
                // }
                ring_cnt++;
                if(ring_cnt > 1)
                {
                    // for(int i = 0; i < track.pointsEdgeRight.size() / 2; i++)
                    // {
                    //     if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                    //         counterSpurroad++;
                    // }
                    // if(counterSpurroad > 30)
                    //     ringStep = RingStep::Entering;

                    counterSpurroad = 0;
                    ringType = RingType::RingRight;
                }
            }
            else
            {
                ring_cnt = 0;
            }

            // /*环岛识别方法2*/
            // if(track.stdevLeft > 120 && track.stdevRight < 60 && abs(track.pointsEdgeRight[0].y - track.pointsEdgeRight[ROWSIMAGE / 2].y) > 5)
            // {
            //     uint16_t rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft, 0, ROWSIMAGE / 2);
            //     for(int i = rowBreakLeftDown; i < track.pointsEdgeLeft.size(); i++)
            //     {
            //         uint16_t counter = 0, step = 0;
            //         if(track.pointsEdgeLeft[i].y < 5)
            //         {
            //             if(step % 2 == 0)
            //                 counter++;
            //             else if(step % 2 == 1)
            //                 counter = 0;
            //         }
            //         else
            //         {
            //             if(step % 2 == 0)
            //                 counter = 0;
            //             else if(step % 2 == 1)
            //                 counter++;
            //         }
            //         if(counter > 5)
            //         {
            //             step += 1;
            //             counter = 0;
            //             if(step == 3 && rowBreakLeftDown != 0)
            //             {
            //                 ringType = RingType::RingLeft;
            //             }
            //         }
            //     }
            // }
            // else if(track.stdevLeft < 60 && track.stdevRight > 120 && abs(track.pointsEdgeLeft[0].y - track.pointsEdgeLeft[ROWSIMAGE / 2].y) > 5)
            // {
            //     uint16_t rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE / 2);
            //     for(int i = rowBreakRightDown; i < track.pointsEdgeRight.size(); i++)
            //     {
            //         uint16_t counter = 0, step = 0;
            //         if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
            //         {
            //             if(step % 2 == 0)
            //                 counter++;
            //             else if(step % 2 == 1)
            //                 counter = 0;
            //         }
            //         else
            //         {
            //             if(step % 2 == 0)
            //                 counter = 0;
            //             else if(step % 2 == 1)
            //                 counter++;
            //         }
            //         if(counter > 5)
            //         {
            //             step += 1;
            //             counter = 0;
            //             if(step == 3 && rowBreakRightDown != 0)
            //             {
            //                 ringType = RingType::RingRight;
            //             }
            //         }
            // }
        }
        else if(ringType != RingType::RingNone && ringStep == RingStep::None)
        {
			// counterExit++;
			// if (counterExit > 40) {
			//   reset();
			//   return false;
			// }

            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, ROWSIMAGE / 2);
                uint16_t rowBreakLeftU = searchBreakLeftDown(track.pointsEdgeLeft, rowBreakLeftD + 30, track.pointsEdgeLeft.size());
                
                pointBreakD = track.pointsEdgeLeft[rowBreakLeftD];
                if(rowBreakLeftD && rowBreakLeftU)///////////////////////////////
                {
                    counterSpurroad++;
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeftU];
                    line(track.pointsEdgeLeft, rowBreakLeftD, rowBreakLeftU);
                }
                else if(rowBreakLeftD && !rowBreakLeftU)
                {
                    counterSpurroad++;
                    line(track.pointsEdgeLeft, rowBreakLeftD, track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1]);
                }
                else if(counterSpurroad > 2)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Entering;
                }
                // else
                // {
                //     for(int i = 0; i < track.pointsEdgeLeft.size() / 2; i++)
                //     {
                //         if(track.pointsEdgeLeft[i].y < 5)
                //             counterSpurroad++;
                //     }
                //     if(counterSpurroad > 30)
                //         ringStep = RingStep::Entering;
                // }             
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE / 2);
                uint16_t rowBreakRightU = searchBreakRightDown(track.pointsEdgeRight, rowBreakRightD + 30, track.pointsEdgeRight.size());

                pointBreakD = track.pointsEdgeRight[rowBreakRightD];
                if(rowBreakRightD && rowBreakRightU)//////////////////////
                {
                    counterSpurroad++;
                    pointBreakU = track.pointsEdgeRight[rowBreakRightU];
                    line(track.pointsEdgeRight, rowBreakRightD, rowBreakRightU);
                }
                else if(rowBreakRightD && !rowBreakRightU)
                {
                    counterSpurroad++;
                    line(track.pointsEdgeRight, rowBreakRightD, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1]);
                }
                else if(counterSpurroad > 2)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Entering;
                }
                // else
                // {
                //     for(int i = 0; i < track.pointsEdgeRight.size() / 2; i++)
                //     {
                //         if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                //             counterSpurroad++;
                //     }
                //     if(counterSpurroad > 30)
                //         ringStep = RingStep::Entering;
                // }             
            }

        }
        else if(ringStep == RingStep::Entering)
        {
            _corner = POINT(0, 0);
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeftD = 0;
                uint16_t spurroad_item = 0;
                if(track.spurroad.size() == 0)
                {
                    rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size());
                    uint16_t rowBreakLeftU = searchBreakLeftUp(track.pointsEdgeLeft);
                    if(rowBreakLeftU > ROWSIMAGE / 2 && track.pointsEdgeLeft[rowBreakLeftU].y > 70)
                    {
                        spurroad_item = rowBreakLeftU;
                        _corner = track.pointsEdgeLeft[rowBreakLeftU];
                    }
                }
                else if(track.spurroad.size() == 1)
                {
                    //寻找岔路行对应边线下标
                    if(track.spurroad[0].y > 70)
                    {
                        _corner = track.spurroad[0];
                        spurroad_item = abs(_corner.x - track.pointsEdgeLeft[0].x);
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, spurroad_item);
                    }
                    else
                    {
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size());
                        uint16_t rowBreakLeftU = searchBreakLeftUp(track.pointsEdgeLeft);
                        if(rowBreakLeftU > ROWSIMAGE / 2 && rowBreakLeftU > rowBreakLeftD)
                        {
                            spurroad_item = rowBreakLeftU;
                            _corner = track.pointsEdgeLeft[rowBreakLeftU];
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < track.spurroad.size(); i++)
                    {
                        if(track.spurroad[i].x > _corner.x)
                        {
                            _corner = track.spurroad[i];
                        }
                        spurroad_item = abs(_corner.x - track.pointsEdgeLeft[0].x);
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, spurroad_item);
                    }
                }

                pointBreakD = track.pointsEdgeLeft[0];
                pointBreakU = track.pointsEdgeLeft[rowBreakLeftD];
                line(track.pointsEdgeLeft, 0, rowBreakLeftD);
                if(_corner.x)
                {
                    if(_corner.x > ROWSIMAGE / 2)
                        counterSpurroad++;
                    // line(track.pointsEdgeRight, rowBreakLeftD, _corner);
                    {
                        POINT startPoint = track.pointsEdgeRight[rowBreakLeftD];
                        POINT endPoint = _corner;
                        POINT midPoint = POINT((0.3*startPoint.x + 0.7*endPoint.x), (0.3*startPoint.y + 0.7*endPoint.y));
                        midPoint.y += abs(startPoint.y - endPoint.y) / 4;
                        uint16_t rowBreakMid = abs(midPoint.x - startPoint.x) + rowBreakLeftD - 1;
                        line(track.pointsEdgeRight, rowBreakLeftD, midPoint);
                        line(track.pointsEdgeRight, rowBreakMid, endPoint);
                    }
                    track.pointsEdgeLeft.resize(spurroad_item);
                    track.pointsEdgeRight.resize(spurroad_item);

                    const uint16_t width_thresh = track.pointsEdgeRight[spurroad_item - 1].y - track.pointsEdgeLeft[spurroad_item - 1].y;
                    uint16_t mid = (track.pointsEdgeLeft[spurroad_item - 1].y + track.pointsEdgeRight[spurroad_item - 1].y) / 2;
                    POINT left(0, 0);
                    POINT right(0, 0);
                    for(int i = _corner.x - 1; i > ROWSIMAGE / 3; i--)
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
                else if(_corner.x == 0 && counterSpurroad > 3)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Inside;
                }
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t spurroad_item = 0;
                uint16_t rowBreakRightD = 0;
                if(track.spurroad.size() == 0)
                {
                    rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size());
                    uint16_t rowBreakRightU = searchBreakRightUp(track.pointsEdgeRight);
                    if(rowBreakRightU > ROWSIMAGE / 2 && track.pointsEdgeRight[rowBreakRightU].y < COLSIMAGE - 70)
                    {
                        spurroad_item = rowBreakRightU;
                        _corner = track.pointsEdgeRight[rowBreakRightU];
                    }
                }
                else if(track.spurroad.size() == 1)
                {
                    if(track.spurroad[0].y < COLSIMAGE - 70)
                    {
                        //寻找岔路行对应边线下标
                        _corner = track.spurroad[0];
                        spurroad_item = abs(_corner.x - track.pointsEdgeRight[0].x);
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, spurroad_item);
                    }
                    else
                    {
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size());
                        uint16_t rowBreakRightU = searchBreakRightUp(track.pointsEdgeRight);
                        if(rowBreakRightU > ROWSIMAGE / 2 && rowBreakRightU > rowBreakRightD)
                        {
                            _corner = track.pointsEdgeRight[rowBreakRightU];
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < track.spurroad.size(); i++)
                    {
                        if(track.spurroad[i].x > _corner.x)
                        {
                            _corner = track.spurroad[i];
                        }
                        spurroad_item = abs(_corner.x - track.pointsEdgeRight[0].x);
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, spurroad_item);
                    }
                }

                pointBreakD = track.pointsEdgeRight[0];
                pointBreakU = track.pointsEdgeRight[rowBreakRightD];
                line(track.pointsEdgeRight, 0, rowBreakRightD);
                if(_corner.x)
                {
                    if(_corner.x > ROWSIMAGE / 2)
                        counterSpurroad++;
                    // line(track.pointsEdgeLeft, rowBreakRightD, _corner);
                    {
                        POINT startPoint = track.pointsEdgeLeft[rowBreakRightD];
                        POINT endPoint = _corner;
                        POINT midPoint = POINT((0.3*startPoint.x + 0.7*endPoint.x), (0.3*startPoint.y + 0.7*endPoint.y));
                        midPoint.y -= abs(startPoint.y - endPoint.y) / 4;
                        uint16_t rowBreakMid = abs(midPoint.x - startPoint.x) + rowBreakRightD - 1;
                        line(track.pointsEdgeLeft, rowBreakRightD, midPoint);
                        line(track.pointsEdgeLeft, rowBreakMid, endPoint);
                    }
                    track.pointsEdgeLeft.resize(spurroad_item);
                    track.pointsEdgeRight.resize(spurroad_item);

                    const uint16_t width_thresh = track.pointsEdgeRight[spurroad_item - 1].y - track.pointsEdgeLeft[spurroad_item - 1].y;
                    uint16_t mid = (track.pointsEdgeLeft[spurroad_item - 1].y + track.pointsEdgeRight[spurroad_item - 1].y) / 2;
                    POINT left(0, 0);
                    POINT right(0, 0);
                    for(int i = _corner.x - 1; i > ROWSIMAGE / 3; i--)
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
                else if(_corner.x == 0 && counterSpurroad > 3)
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
                    pointBreakU.y += COLSIMAGE / 8;
                    line(track.pointsEdgeRight, rowBreakRight, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    track.pointsEdgeRight.resize(rowBreakLeft);
                }
                else if(!rowBreakRight && counterSpurroad > 2)
                {
                    pointBreakD = track.pointsEdgeRight[0];
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                    pointBreakU.y += COLSIMAGE / 8;
                    line(track.pointsEdgeRight, rowBreakRight, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    track.pointsEdgeRight.resize(rowBreakLeft);

                    if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y < COLSIMAGE / 2)
                    {
                        ringStep = RingStep::Exiting;
                        counterSpurroad = 0;
                    }
                }
                else
                {
                    if(track.pointsEdgeLeft.size() < 100 && track.pointsEdgeRight.size() < 100)
                    {
                        track.pointsEdgeLeft = pointsEdgeLeftLast;
                        track.pointsEdgeRight = pointsEdgeRightLast;
                        line(track.pointsEdgeRight, 0, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1]);
                    }
                    else
                    {
                        // track.pointsEdgeLeft.resize(0);
                        pointsEdgeLeftLast = track.pointsEdgeLeft;
                        pointsEdgeRightLast = track.pointsEdgeRight;
                    }
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
                    pointBreakU.y -= COLSIMAGE / 8;
                    line(track.pointsEdgeLeft, rowBreakLeft, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakRight);
                    track.pointsEdgeRight.resize(rowBreakRight);
                }
                else if(!rowBreakLeft && counterSpurroad > 2)
                {
                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    pointBreakD = track.pointsEdgeLeft[0];
                    pointBreakU.y -= COLSIMAGE / 8;
                    line(track.pointsEdgeLeft, rowBreakLeft, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakRight);
                    track.pointsEdgeRight.resize(rowBreakRight);

                    if(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y > COLSIMAGE / 2)
                    {
                        ringStep = RingStep::Exiting;
                        counterSpurroad = 0;
                    }
                }
                else
                {
                    if(track.pointsEdgeLeft.size() < 100 && track.pointsEdgeRight.size() < 100)
                    {
                        track.pointsEdgeLeft = pointsEdgeLeftLast;
                        track.pointsEdgeRight = pointsEdgeRightLast;
                        line(track.pointsEdgeLeft, 0, track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1]);
                    }
                    else
                    {
                        // track.pointsEdgeRight.resize(0);
                        pointsEdgeLeftLast = track.pointsEdgeLeft;
                        pointsEdgeRightLast = track.pointsEdgeRight;
                    }
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
                if(rowBreakRight < ROWSIMAGE / 4 || track.pointsEdgeLeft.size() > 140)
                {
                    ringStep = RingStep::Finish;
                }
                else
                {
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                    pointBreakU.y += COLSIMAGE / 8;
                    pointBreakD = track.pointsEdgeRight[0];
                    line(track.pointsEdgeRight, 0, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakLeft);
                    track.pointsEdgeRight.resize(rowBreakLeft);
                }
            }
            else if(ringType == RingType::RingRight)
            {
                if(rowBreakLeft < ROWSIMAGE / 4 || track.pointsEdgeRight.size() > 140)
                {
                    ringStep = RingStep::Finish;
                }
                else
                {
                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    pointBreakU.y -= COLSIMAGE / 8;
                    pointBreakD = track.pointsEdgeLeft[0];
                    line(track.pointsEdgeLeft, 0, pointBreakU);
                    track.pointsEdgeLeft.resize(rowBreakRight);
                    track.pointsEdgeRight.resize(rowBreakRight);
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
                else if(rowBreakLeft < ROWSIMAGE / 3 && counterSpurroad > 3)
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
                else if(rowBreakRight < ROWSIMAGE / 3 && counterSpurroad > 3)
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
            circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 3, Scalar(0, 0, 255), -1); // 红色点
        }
        circle(Image, Point(_corner.y, _corner.x), 6, Scalar(0, 0, 255), -1); // 红色点

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
        putText(Image, to_string(track.validRowsRight) + " " + to_string(track.stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1);
        putText(Image, to_string(track.validRowsLeft) + " " + to_string(track.stdevLeft), Point(20, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1);

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
            if(pointsEdgeLeft[i].y > pointsEdgeLeft[i - 1].y && start == false)
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
                    if(row_end <= COLSIMAGE / 2 || abs(pointsEdgeLeft[i].y - pointsEdgeLeft[rowBreakLeft].y) > 5)
                        counter++;
                    if (counter > 8)
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
            if(pointsEdgeRight[i].y < pointsEdgeRight[i - 1].y && start == false)
            {
                start = true;
            }
            if(start)
            {
                if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRight].y)
                {
                    rowBreakRight = i;
                    counter = 0;
                }
                else if (pointsEdgeRight[i].y >= pointsEdgeRight[rowBreakRight].y) // 突变点计数
                {
                    if(row_end <= COLSIMAGE / 2 || abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRight].y) > 5)
                        counter++;
                    if (counter > 8)
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
        uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 1;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeLeft.size() - 5; i > 0; i--)
        {
            if (pointsEdgeLeft[i].y > 5 && abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 5)
            {
                rowBreakLeftUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeLeft[i].y <= 5 && abs(pointsEdgeLeft[i].y - pointsEdgeLeft[rowBreakLeftUp].y) > 5 && counterFilter > 10)
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
        uint16_t rowBreakRightUp = pointsEdgeRight.size() - 1;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeRight.size() - 5; i > 0; i--)
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2 && abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 5)
            {
                rowBreakRightUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRightUp].y) > 5 && counterFilter > 10)
            {
                counter++;
                if (counter > 3)
                    return rowBreakRightUp;
            }
        }

        return 0;
    }
    POINT _corner;
    uint16_t counterSpurroad = 0; // 岔路计数器
    uint16_t ring_cnt = 0; // 环岛检测确认计数器
	uint16_t counterExit = 0;	  // 异常退出计数器
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器

    POINT pointBreakU;
    POINT pointBreakD;
    vector<POINT> pointsEdgeLeftLast;  // 记录前一场左边缘点集
    vector<POINT> pointsEdgeRightLast; // 记录前一场右边缘点集
};
