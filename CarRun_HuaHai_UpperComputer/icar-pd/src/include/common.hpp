#pragma once

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../src/perspective_mapping.cpp"

// using nlohmann::json;
using namespace std;
using namespace cv;

/*----------------------------------------方向控制相关参数宏定义------------------------------------------------*/
#define Weight_Ratio_parameter_above  1                 // 权重比例参数——上
#define Weight_Ratio_parameter_middle 160               // 权重比例参数——中
#define Weight_Ratio_parameter_under  80                // 权重比例参数——下

#define highly_control_point    90      //定义方向控制，拟合曲线的最高权重x轴区域点坐标
#define AMPLIFICATION_FACTOR    950     //拟合曲线放大系数
/*----------------------------------------方向控制相关参数宏定义------------------------------------------------*/



#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define COLSIMAGEIPM 320 // IPM图像的列数
#define ROWSIMAGEIPM 400 // IPM图像的行数
#define PWMSERVOMAX 1850 // 舵机PWM最大值（左）1850
#define PWMSERVOMID 1500 // 舵机PWM中值 1500
#define PWMSERVOMIN 1150 // 舵机PWM最小值（右）1150

#define LABEL_CONE "cone"           // AI标签：锥桶
#define LABEL_GRANARY "granary"     // AI标签：谷仓
#define LABEL_BRIDGE "bridge"       // AI标签：桥
#define LABEL_TRACTOR "tractor"     // AI标签：拖拉机
#define LABEL_CORN "corn"           // AI标签：玉米
#define LABEL_PIG "pig"             // AI标签：猪
#define LABEL_CROSSWALK "crosswalk" // AI标签：斑马线
#define LABEL_BUMP "bump"           // AI标签：减速带

PerspectiveMapping ipm; // 逆透视变换公共类

struct POINT
{
    int x = 0;
    int y = 0;
    float slope = 0.0f;

    POINT(){};
    POINT(int x, int y) : x(x), y(y){};
};

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image)
{
    // 存图
    string name = ".png";
    static int counter = 0;
    counter++;
    string img_path = "../res/samples/train/";
    name = img_path + to_string(counter) + ".jpg";
    imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<POINT> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
vector<POINT> Bezier(double dt, vector<POINT> input)
{
    vector<POINT> output;

    double t = 0;
    while (t <= 1)
    {
        POINT p;
        double x_sum = 0.0;
        double y_sum = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n)
        {
            double k =
                factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            x_sum += k * input[i].x;
            y_sum += k * input[i].y;
            i++;
        }
        p.x = x_sum;
        p.y = y_sum;
        output.push_back(p);
        t += dt;
    }
    return output;
}

auto formatDoble2String(double val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(POINT a, POINT b, POINT p)
{
    int d = 0; // 距离

    double ab_distance =
        sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    double ap_distance =
        sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
    double bp_distance =
        sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

    double half = (ab_distance + ap_distance + bp_distance) / 2;
    double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

    return (2 * area / ab_distance);
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief 画线算法-Bresenham算法
 *
 * @param 边线
 * @param 起点序号
 * @param 终点序号
 * @return 
 */
void line(vector<POINT> &line, uint16_t p0, uint16_t p1)
{
	int x0 = line[p0].x, y0 = line[p0].y;
	int x1 = line[p1].x, y1 = line[p1].y;
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int erro = (dx > dy ? dx : -dy) / 2;

	while (x0 != x1 || y0 != y1)
	{
        line[p0].x = x0;
        line[p0].y = y0;
        if(x0 != line[p0 - 1].x)//确保一行一个点
        {
            p0++;
        }
		int e2 = erro;
		if (e2 > -dx) { erro -= dy; x0 += sx; }
		if (e2 < dy) { erro += dx; y0 += sy; }
	}
}
/**
 * @brief 画线算法-Bresenham算法
 *
 * @param 边线
 * @param 起点序号
 * @param 终点
 * @return 
 */
void line(vector<POINT> &line, uint16_t p0, POINT p1)
{
	int x0 = line[p0].x, y0 = line[p0].y;
	int x1 = p1.x, y1 = p1.y;
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int erro = (dx > dy ? dx : -dy) / 2;

	while (x0 != x1 || y0 != y1)
	{
        line[p0].x = x0;
        line[p0].y = y0;
        if(x0 != line[p0 - 1].x)
        {
            p0++;
        }
		int e2 = erro;
		if (e2 > -dx) { erro -= dy; x0 += sx; }
		if (e2 < dy) { erro += dx; y0 += sy; }
	}
}

//生成随机图片名称
std::string generateRandomFileName(std::string img_path)
{
	time_t now = time(0);
	tm* localtm = localtime(&now);

	char buffer[80];
	strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", localtm);

	srand(time(NULL));
	int randNum = rand() % 10000;

	string fileName = string(buffer) + "_" + to_string(randNum);
    fileName = img_path + fileName;

	return fileName;
}
