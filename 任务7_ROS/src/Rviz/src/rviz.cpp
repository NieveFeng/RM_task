/**
 * @file rviz.cpp
 * @author 张兆锋 (1294051350@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-10-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

Mat src, src_gray;			 	  //源图像处理
Mat dst, dst_two; 			 	  //处理后图像
vector<Mat> channels;			  //分离三通道
RotatedRect rotated_rect;		  //旋转矩形
Rect rect;						  //普通矩形
vector<vector<Point>> g_contours; //检测到的轮廓
vector<Vec4i> g_hierarchy;		  //输出向量

Mat_<double> camera_matrix = (Mat_<double>(3, 3) << 
							  1.2853517927598091e+03, 0., 3.1944768628958542e+02,
							  0., 1.2792339468697937e+03, 2.3929354061292258e+02,
							  0., 0., 1.);
Mat_<double> dist_coeffs = (Mat_<double>(1, 5) << 
							-6.3687295852461456e-01, -1.9748008790347320e+00,
							3.0970703651800782e-02, 2.1944646842516919e-03, 0.);

// 输入物体上的点
vector<Point3d> world_points = {Point3d(0, 0, 0),
								Point3d(0, 26.5, 0),
								Point3d(67.5, 26.5, 0),
								Point3d(67.5, 0, 0)};

Point2f point_rect[4];	  //旋转矩形的四个顶点

int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open("/home/nieve/feng/test/装甲板_1.avi");
    if (!capture.isOpened())
    {
        cout << "视频无了" << endl;
    }

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("src", 1);

    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok()||capture.read(src) || !src.empty() || waitKey(0) != 27)
    {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();

        capture.read(src);
        if (src.empty())
        {
            printf("open error\n");
        }
        		vector<Point> g_rect; //轮廓的二维点集
		if (src.empty())
		{
			cout << "帧无了" << endl;
		}

		namedWindow("视频", WINDOW_AUTOSIZE); //命名窗口
		moveWindow("视频", 200, 60);		//移动窗口

		split(src,channels);
		src_gray=channels[2]-channels[0];

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

		//图像上的点
		vector<Point2d> src_points = {  Point2d(point_rect[0].x, point_rect[0].y),
										Point2d(point_rect[1].x, point_rect[1].y),
										Point2d(point_rect[2].x, point_rect[2].y),
										Point2d(point_rect[3].x, point_rect[3].y)};

		Mat revc, tevc, revc_matrix;

		solvePnP(world_points, src_points, camera_matrix, dist_coeffs, revc, tevc, false, 2);

		Rodrigues(revc, revc_matrix);

		Mat p;
		p = -revc_matrix.inv() * tevc;
		cout << "相机世界坐标:" << endl << p << endl;
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
        pub.publish(msg);

        ros::Rate loop_rate(5);

        visualization_msgs::Marker marker;

        marker.header.frame_id = "armor";
        marker.header.stamp = ros::Time::now();

        marker.ns = "image_publisher";
        marker.id = 0;

        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = (500-tevc.at<double>(2,0))*0.01;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = thetax;
        marker.pose.orientation.y = thetay;
        marker.pose.orientation.z = thetaz;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.675;
        marker.scale.y = 0.1;
        marker.scale.z = 0.265;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0; //透明度

        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
