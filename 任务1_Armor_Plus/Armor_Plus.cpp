/**
 * @file Armor.cpp
 * @author 张兆锋 (1294051350@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-10-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

Mat src, src_gray;          //源图像处理
Mat dst, dst_two;           //处理后图像
vector<Mat> channels;       //分离三通道
RotatedRect rotated_rect_1; //旋转矩形
RotatedRect rotated_rect_2;
Rect rect;                        //普通矩形
vector<vector<Point>> g_contours; //检测到的轮廓

vector<vector<Point>> light_contours; //检测到的轮廓
vector<Vec4i> g_hierarchy;            //输出向量

Point2f point_rec_1[4]; //旋转矩形的四个顶点
Point2f point_rec_2[4]; //旋转矩形的四个顶点

int main()
{
    VideoCapture capture;
    capture.open("装甲板_2.avi");
    if (!capture.isOpened())
    {
        cout << "视频无了" << endl;
    }

    //退出程序按Esc
    while (waitKey(0) != 27)
    {
        capture.read(src);
        vector<Point> g_rect; //轮廓的二维点集
        if (src.empty())
            cout << "帧无了" << endl;

        namedWindow("视频", WINDOW_AUTOSIZE); //命名窗口
        moveWindow("视频", 20, 60);           //移动窗口

        //分离三通道，通道相减获取红色灯条
        split(src, channels);
        src_gray = channels[2] - channels[0];

        threshold(src_gray, dst_two, 100, 255, 0);

        //寻找轮廓
        findContours(dst_two, g_contours, g_hierarchy,
                     RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

        Mat drawing = Mat::zeros(dst_two.size(), CV_8UC3);

        //遍历轮廓
        for (int i = 0; i < g_contours.size(); i++)
        {
            double light_area = contourArea(g_contours[i]);

            RotatedRect light_rec = minAreaRect(g_contours[i]);

            if (light_rec.size.area() < 100)
                continue;

            cout << i << endl;
            cout << "S:" << light_rec.size.area() << endl;
            //打印最小旋转矩形成员参数
            cout << "P:" << light_rec.size.aspectRatio() << endl;
            cout << "J:" << light_rec.angle << endl;
            cout << "I:" << light_rec.size.area() / light_area << endl;
            cout << "W:" << light_rec.size.width << endl;
            cout << "H:" << light_rec.size.height << endl;

            drawContours(drawing, g_contours, i, Scalar(255, 255, 255),
                         2, 8, g_hierarchy, 0, Point(0, 0));
            light_rec.size.height *= 1.1;
            light_rec.size.width *= 1.1;
            light_contours.push_back(g_contours[i]);
        }

        for (int i = 0; i < light_contours.size(); i++)
        {
            for (int j = i + 1; j < light_contours.size(); j++)
            {
                RotatedRect light_rec_1 = minAreaRect(g_contours[i]);
                RotatedRect light_rec_2 = minAreaRect(g_contours[j]);
                if (abs(light_rec_1.angle - light_rec_2.angle) < 5 &&
                    abs(light_rec_1.size.height - light_rec_2.size.height) < 5 &&
                    abs(light_rec_1.size.width - light_rec_2.size.width) < 5)
                {
                    cout << i << j << endl;
                    //分别读取四个顶点
                    light_rec_1.points(point_rec_1);
                    light_rec_2.points(point_rec_2);
                }
            }
        }
        light_contours.clear();
        imshow("视频", src);
        waitKey(100);
    }
    // for (; j >= 0; j--)
    // {
    //     RotatedRect light_rec_2 = minAreaRect(g_contours[j]);

    //     {
    //         //绘制最小包围旋转矩形

    //         cout << j << endl;
    //         cout << "A2" << light_rec_2.size.area() << endl;
    //         cout << "P2:" << light_rec_2.size.aspectRatio() << endl;
    //         cout << "J2:" << light_rec_2.angle << endl;
    //         cout << "I2:" << light_rec_2.size.area() / light_area << endl;
    //         cout << "S2:" << light_rec_2.size.area() << endl;
    //         cout << "W2:" << light_rec_2.size.width << endl;
    //         cout << "H2:" << light_rec_2.size.height << endl;
    //         cout << "X2:" << light_rec_2.center.x << endl;
    //         cout << "Y2:" << light_rec_2.center.y << endl;
    //         //分别读取四个顶点);
    //         rotated_rect_2.points(point_rect_2);

    //         //pointOrder(point_rect);
    //         //pointOrder(point_rect_2);

    //         //绘制对角线and中心点
    //         line(src, rotated_rect.center.x + 20, rotated_rect.center.x - 20, Scalar(255, 0, 0), 1);
    //         line(src, point_rect_2[1], point_rect_2[2], Scalar(255, 0, 0), 1);
    //         circle(src, rotated_rect.center, 1, Scalar(0, 255, 0), 1);
    //     }

    cout << "\n识别完成！" << endl;
    return 0;
}
