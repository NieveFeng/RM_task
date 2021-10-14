/**
 * @file Rune.cpp
 * @author 张兆锋 (1294051350@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace std;

Mat src, src_gray;
Mat dst_two;
vector<Mat> channels;
vector<vector<Point>> g_contours; //检测到的轮廓
vector<Vec4i> g_hierarchy;		  //输出向量
Point2f point_rect[4];			  //旋转矩形的四个顶点

int main()
{
	VideoCapture capture;
	capture.open("能量机关_1.avi");
	if (!capture.isOpened())
	{
		cout << "视频无了" << endl;
	}

	//退出程序按Esc
	while (capture.read(src) || (char)(waitKey(0)) != 27)
	{

		if (src.empty())
		{
			cout << "帧无了" << endl;
		}

		namedWindow("视频", WINDOW_AUTOSIZE); //命名窗口
		moveWindow("视频", 400, 200);		  //移动窗口

		//分离三通道，获取蓝色灯条
		split(src, channels);
		src_gray = channels[0] - channels[2];

		//二值化图像
		threshold(src_gray, dst_two, 75, 255, 0);

		//闭运算与膨胀内核大小
		Mat kernel_close = getStructuringElement(cv::MORPH_RECT, Size(3, 3));
		Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, Size(5, 5));

		//对轮廓进行膨胀
		dilate(dst_two, dst_two, kernel_dilate);
		//对轮廓进行闭运算，使得形状特征更加明显
		morphologyEx(dst_two, dst_two, MORPH_CLOSE, kernel_close, Point(-1, -1), 3);

		//绘制轮廓
		findContours(dst_two, g_contours, g_hierarchy,
					 RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat drawing = Mat::zeros(dst_two.size(), CV_8UC3);

		//遍历轮廓
		for (int i = 0; i < g_contours.size(); i++)
		{
			double light_area = contourArea(g_contours[i]);

			RotatedRect light_Rec = minAreaRect(g_contours[i]);

			cout << i << endl;
			cout << "子轮廓:" << g_hierarchy[i][2] << endl;
			cout << "父轮廓:" << g_hierarchy[i][3] << endl;
			cout << "A:" << light_Rec.size.area() << endl;
			cout << "S:" << light_area << endl;
			cout << "H:" << light_Rec.size.height << endl;
			cout << "W:" << light_Rec.size.width << endl;
			cout << "P:" << light_Rec.size.height / light_Rec.size.width << endl;
			//筛除条件：经典算法 子轮廓与父轮廓
			if (g_hierarchy[i][3] != -1 && g_hierarchy[i][2] == -1 &&
				g_hierarchy[i][1] == -1 && g_hierarchy[i][0] == -1)
			{
				//筛除条件：最小旋转矩形的长宽比与面积
				if (light_Rec.size.height / light_Rec.size.width < 2.5 &&
					light_Rec.size.height / light_Rec.size.width > 0.35 &&
					light_Rec.size.area() > 500)
				{
					RotatedRect right_Rec = minAreaRect(g_contours[i]);
					//分别读取四个顶点
					right_Rec.points(point_rect);
					//绘制矩形
					for (int i = 0; i < 4; i++)
						line(src, point_rect[i], point_rect[(i + 1) % 4], Scalar(0, 0, 255), 3);
					//绘制中心击打点
					circle(src, right_Rec.center, 1, Scalar(0, 255, 255), 2);
				}
			}
		}
		imshow("视频", src);
		waitKey(20);
	}
}