/**
 * @file serial_example.cc
 * @author 张兆锋 (1294051350@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-10-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::string;
using std::vector;
string result;

void my_sleep(unsigned long milliseconds)
{
  usleep(milliseconds * 1000); // 100 ms
}

//列出当前串口的信息
void enumerate_ports()
{
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end())
  {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
}

//打印错误信息
void print_usage()
{
  cerr << "Usage: test_serial {-e|<serial port address>} ";
  cerr << "<baudrate> [test string]" << endl;
}

//rosrun需要两个参数
int run(int argc, char **argv)
{
  if (argc < 2)
  {
    print_usage();
    return 0;
  }

  //第一个参数为串口号或者获取串口信息
  string port(argv[1]);

  //-e列出当前串口信息
  if (port == "-e")
  {
    enumerate_ports();
    return 0;
  }
  else if (argc < 3)
  {
    print_usage();
    return 1;
  }

  //第二个参数为波特率
  unsigned long baud = 0;
  sscanf(argv[2], "%lu", &baud);

  //串口号，波特率，超时时间
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  int count = 0;
  string test_string;

  //第三个参数是打印设定信息，或者打印默认信息
  if (argc == 4)
  {
    test_string = argv[3];
  }
  else
  {
    test_string = "Hello RM！";
  }
  //读写规定的超时时间
  my_serial.setTimeout(serial::Timeout::max(), 50, 0, 50, 0);

  //写入串口数据
  size_t bytes_wrote = my_serial.write(test_string);

  //读取串口数据
  result = my_serial.read(test_string.length() + 1);
  cout << "数据长度: " <<result.length() << " , read: " << result << endl;
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_example");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("uart1", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    run(argc, argv);
    ss << result;
    msg.data = ss.str();
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
