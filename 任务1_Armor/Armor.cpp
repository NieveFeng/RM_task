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

Mat src, src_gray;				  //源图像处理
Mat dst, dst_two;				  //处理后图像
vector<Mat> channels;			  //分离三通道
RotatedRect rotated_rect;		  //旋转矩形
Rect rect;						  //普通矩形
vector<vector<Point>> g_contours; //检测到的轮廓
vector<Vec4i> g_hierarchy;		  //输出向量

Mat_<double> camera_matrix = (Mat_<double>(3, 3) << 1.2853517927598091e+03, 0., 3.1944768628958542e+02,
							  0., 1.2792339468697937e+03, 2.3929354061292258e+02,
							  0., 0., 1.);
Mat_<double> dist_coeffs = (Mat_<double>(1, 5) << -6.3687295852461456e-01, -1.9748008790347320e+00,
							3.0970703651800782e-02, 2.1944646842516919e-03, 0.);

// 输入物体上的点
vector<Point3d> world_points = {Point3d(0, 0, 0),
								Point3d(0, 26.5, 0),
								Point3d(67.5, 26.5, 0),
								Point3d(67.5, 0, 0)};

Point2f point_rect[4]; //旋转矩形的四个顶点

int main()
{
	VideoCapture capture;
	capture.open("装甲板_1.avi");
	if (!capture.isOpened())
	{
		cout << "视频无了" << endl;
	}

	//退出程序按Esc
	while (capture.read(src) || !src.empty() || waitKey(0) != 27)
	{

		vector<Point> g_rect; //轮廓的二维点集
		if (src.empty())
		{
			cout << "帧无了" << endl;
		}

		namedWindow("视频", WINDOW_AUTOSIZE); //命名窗口
		moveWindow("视频", 200, 60);		  //移动窗口

		split(src, channels);
		src_gray = channels[2] - channels[0];

		threshold(src_gray, dst_two, 100, 255, 0);

		//寻找轮廓
		findContours(dst_two, g_contours, g_hierarchy,
					 RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat drawing = Mat::zeros(dst_two.size(), CV_8UC3);

		//遍历轮廓
		for (int i = 0; i < g_contours.size(); i++)
		{
			double light_area = contourArea(g_contours[i]);

			RotatedRect light_rec = minAreaRect(g_contours[i]);
			cout << i << endl;
			cout << "A" << light_area << endl;

			if (light_area < 300)
				continue;

			cout << "P:" << light_rec.size.aspectRatio() << endl;
			cout << "J:" << light_rec.angle << endl;
			cout << "I:" << light_rec.size.area() / light_area << endl;
			cout << "S:" << light_rec.size.area() << endl;

			if (light_rec.size.area() < 500)
				continue;

			cout << "W:" << light_rec.size.width << endl;
			cout << "H:" << light_rec.size.height << endl;

			drawContours(drawing, g_contours, i, Scalar(255, 255, 255),
						 2, 8, g_hierarchy, 0, Point(0, 0));
			//将寻找到的轮廓点向量化为二维点集
			for (int j = 0; j < g_contours[i].size(); j++)
			{
				g_rect.push_back(g_contours[i][j]);
			}
		}

		if (g_rect.empty())
			continue;
		//绘制最小包围旋转矩形
		rotated_rect = minAreaRect(g_rect);

		//分别读取四个顶点
		rotated_rect.points(point_rect);

		//绘制矩形
		for (int i = 0; i < 4; i++)
		{
			line(src, point_rect[i], point_rect[(i + 1) % 4], Scalar(0, 0, 255), 3);
		}

		//绘制对角线and中心点
		line(src, point_rect[0], point_rect[2], Scalar(255, 0, 0), 3);
		line(src, point_rect[1], point_rect[3], Scalar(255, 0, 0), 3);
		circle(src, rotated_rect.center, 3, Scalar(0, 255, 0), 5);

		cout << "O点：" << point_rect[0].x << endl;
		cout << "1点：" << point_rect[1].x << endl;
		cout << "2点：" << point_rect[2].x << endl;
		cout << "3点：" << point_rect[3].x << endl;

		Point2f point_order[3];
		for (int i = 1; i < 4; i++)
		{
			if (abs(point_rect[0].x - point_rect[i].x) < 20)
			{
				cout<<abs(point_rect[0].x - point_rect[i].x)<<endl;
				point_order[0] = point_rect[0];
				point_order[1] = point_rect[i];
				if (i == 1)
				{
					point_order[2] = point_rect[2];
					point_order[3] = point_rect[3];
				}
				if (i == 2)
				{
					point_order[2] = point_rect[1];
					point_order[3] = point_rect[3];
				}
				if (i == 3)
				{
					point_order[2] = point_rect[1];
					point_order[3] = point_rect[2];
				}
			}
		}

		//图像上的点
		vector<Point2d> src_points = {Point2d(point_order[0].x, point_order[0].y),
									  Point2d(point_order[1].x, point_order[1].y),
									  Point2d(point_order[2].x, point_order[2].y),
									  Point2d(point_order[3].x, point_order[3].y)};

		cout << "O点：" << point_order[0].x << endl;
		cout << "1点：" << point_order[1].x << endl;
		cout << "2点：" << point_order[2].x << endl;
		cout << "3点：" << point_order[3].x << endl;
		Mat revc, tevc, revc_matrix;

		solvePnP(world_points, src_points, camera_matrix, dist_coeffs, revc, tevc, false, 2);

		Rodrigues(revc, revc_matrix);

		Mat p;
		p = -revc_matrix.inv() * tevc;
		cout << "相机世界坐标:" << endl
			 << p << endl;
		cout << "深度:" << tevc.at<double>(2, 0) << endl;

		double r11 = revc_matrix.ptr<double>(0)[0];
		double r12 = revc_matrix.ptr<double>(0)[1];
		double r13 = revc_matrix.ptr<double>(0)[2];
		double r21 = revc_matrix.ptr<double>(1)[0];
		double r22 = revc_matrix.ptr<double>(1)[1];
		double r23 = revc_matrix.ptr<double>(1)[2];
		double r31 = revc_matrix.ptr<double>(2)[0];
		double r32 = revc_matrix.ptr<double>(2)[1];
		double r33 = revc_matrix.ptr<double>(2)[2];
		/*************************************此处计算出相机的旋转角**********************************************/
		//计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系。
		//旋转顺序为z、y、x
		double thetaz = atan2(r21, r11) / CV_PI * 180;
		double thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
		double thetax = atan2(r32, r33) / CV_PI * 180;

		cout << "x轴角:" << thetax << endl;
		cout << "y轴角:" << thetay << endl;
		cout << "z轴角:" << thetaz << endl;

		imshow("视频", src);
		waitKey(20);
	}

	cout << "\n识别完成！" << endl;
	return 0;
}
