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

Mat src,src_gray;
Mat dst_two;
vector<Mat> channels;
vector<vector<Point>> g_contours;	//检测到的轮廓
vector<Vec4i> g_hierarchy;			//输出向量

int main()
{
    VideoCapture capture;
    capture.open("能量机关_1.avi");
	if (!capture.isOpened())
	{
		cout << "视频无了" << endl;
	}

	//退出程序按Esc
	while (capture.read(src)||(char)(waitKey(0)) != 27)
	{
		
		if (src.empty())
		{
			cout << "帧无了" << endl;
		}

		//分离三通道，获取蓝色灯条
		split(src,channels);
		src_gray=channels[0]-channels[2];

		//二值化图像
		threshold(src_gray, dst_two, 40, 255, 0);

		Mat kernel = getStructuringElement(cv::MORPH_RECT, Size(3, 3));

		//对轮廓进行闭运算，使得形状特征更加明显
		morphologyEx(dst_two, dst_two, MORPH_CLOSE, kernel, Point(-1, -1), 4);

		//绘制轮廓
		findContours(dst_two, g_contours, g_hierarchy,
			RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat drawing = Mat::zeros(dst_two.size(), CV_8UC3);
		
		//遍历轮廓
		for (int i = 0; i < g_contours.size(); i++)
		{
			double light_area = contourArea(g_contours[i]);
		    
			RotatedRect light_Rec = minAreaRect(g_contours[i]);

			if (g_hierarchy[i][3] >= 9)
				continue;

			//筛除条件：长宽比不在合适范围
			if (light_Rec.size.aspectRatio() > 2 || light_Rec.size.aspectRatio() < 1)
				continue;

			//筛除条件：旋转矩形面积
			if(light_Rec.size.area()<300)
				continue;

			//筛除条件：经典算法 子轮廓与父轮廓
			if (g_hierarchy[i][3] != -1 && g_hierarchy[i][2] == -1 &&
				g_hierarchy[i][1] == -1 && g_hierarchy[i][0] == -1)
			{
				cout << i << endl;
				cout << "子轮廓:" << g_hierarchy[i][2] << endl;
				cout << "父轮廓:" << g_hierarchy[i][3] << endl;
				cout << "A:" << light_Rec.size.area() << endl;
				cout << "S:" << light_area << endl;
				cout << "P:" << light_Rec.size.aspectRatio() << endl;
				drawContours(src, g_contours, i, Scalar(0, 0, 255), 4, 8);
				RotatedRect right_Rec = minAreaRect(g_contours[i]);
				
				//绘制中心击打点
				circle(src, right_Rec.center, 1, Scalar(0, 255, 255), 2);
			}
		}

		namedWindow("视频",WINDOW_AUTOSIZE);		//命名窗口
		moveWindow("视频", 400, 200);			//移动窗口
		imshow("视频",src);
		waitKey(50);
	}
}